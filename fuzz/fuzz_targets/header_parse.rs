#![no_main]

//! Frame-header attacker surface: `FrameHeader::parse`, `find_sync`,
//! the §2.4.2.1 frame-length / slot-count arithmetic, and the
//! §2.4.3.1 free-format next-syncword probe
//! (`detect_free_format_frame_length`).
//!
//! Unlike the `decode` target (no-panic over the full decoder), this
//! target carries **invariant oracles** on the header layer:
//!
//! 1. `find_sync(buf) == Some(i)` ⇒ the four bytes at `i` parse, and
//!    no earlier offset parses (first-match contract).
//! 2. A parsed fixed-bitrate header must report a determinable frame
//!    length that is `(N + padding) · slot_bytes`, at least the
//!    4-byte header, and consistent with `slot_count()`; a free-format
//!    header must report `None` from both (§2.4.2.1 / §2.4.3.1: the
//!    length is uninvertible from the header alone).
//! 3. Probing an attacker-shaped tail after a crafted free-format
//!    header never panics, and on success the recovered
//!    `frame_length_bytes == (base_slot_count + padding) · slot_bytes`
//!    with a matching-stream-parameter header parseable at the
//!    recovered next-frame offset.
//! 4. **Planted-distance oracle**: synthesize a two-frame free-format
//!    stream whose next syncword sits at an attacker-chosen exact slot
//!    distance; the probe must recover exactly the planted slot count
//!    and byte length (§2.4.3.1: "N can be determined from the
//!    distance between consecutive syncwords and the value of the
//!    padding bit").
//!
//! Spec basis: ISO/IEC 11172-3 §2.4.1.3 / §2.4.2.3 (header fields),
//! §2.4.2.1 (slot arithmetic), §2.4.3.1 (sync + free-format prose);
//! ISO/IEC 13818-3 §2.4.2.3 (LSF ladders).

use libfuzzer_sys::fuzz_target;
use oxideav_mp1::{detect_free_format_frame_length, find_sync, Bitrate, FrameHeader, Layer};

/// Assemble a syntactically valid 4-byte header from attacker-chosen
/// fields. `br_sel == None` forces free format (`bitrate_index 0000`).
fn build_header(id: u8, layer: u8, sr: u8, br_sel: Option<u8>, mode: u8, flags: u8) -> [u8; 4] {
    let mut raw: u32 = 0xFFF << 20;
    raw |= u32::from(id & 1) << 19;
    let layer_bits = if layer & 1 == 0 { 0b11u32 } else { 0b10u32 };
    raw |= layer_bits << 17;
    // protection_bit: '0' = CRC present.
    raw |= u32::from((flags & 1) ^ 1) << 16;
    let br_idx = match br_sel {
        Some(b) => (b % 14) + 1,
        None => 0, // free format
    };
    raw |= u32::from(br_idx) << 12;
    raw |= u32::from(sr % 3) << 10;
    raw |= u32::from((flags >> 1) & 1) << 9; // padding
    raw |= u32::from((flags >> 2) & 1) << 8; // private
    raw |= u32::from(mode & 0b11) << 6;
    raw |= u32::from((flags >> 3) & 1) << 4; // mode extension (low bit)
    raw |= u32::from((flags >> 4) & 1) << 3; // copyright
    raw |= u32::from((flags >> 5) & 1) << 2; // original
    raw.to_be_bytes()
}

fn slot_bytes(layer: Layer) -> u32 {
    match layer {
        Layer::I => 4,
        Layer::II => 1,
    }
}

/// Invariant set 2: frame-length arithmetic on any parsed header.
fn check_length_arithmetic(h: FrameHeader) {
    match h.bitrate {
        Bitrate::Fixed(_) => {
            // `slot_count()` already includes the optional padding slot
            // (§2.4.2.1: N + padding_bit).
            let n = h.slot_count().expect("fixed bitrate must have slot count");
            let len = h
                .frame_length_bytes()
                .expect("fixed bitrate must have frame length");
            assert_eq!(
                len,
                n * slot_bytes(h.layer),
                "frame_length_bytes disagrees with slot_count: {h:?}"
            );
            assert!(len >= 4, "frame shorter than its own header: {h:?}");
        }
        Bitrate::Free => {
            assert!(h.slot_count().is_none(), "free format has no slot count");
            assert!(
                h.frame_length_bytes().is_none(),
                "free format length is uninvertible from the header alone"
            );
        }
        Bitrate::Forbidden => unreachable!("parse must reject bitrate_index 15"),
    }
}

fuzz_target!(|data: &[u8]| {
    // --- 1. Raw parse at offset 0 (any length, incl. < 4). ---
    if let Ok(h) = FrameHeader::parse(data) {
        check_length_arithmetic(h);
    }

    // --- 2. find_sync first-match contract over the whole buffer. ---
    match find_sync(data) {
        Some(i) => {
            let h = FrameHeader::parse(&data[i..i + 4])
                .expect("find_sync returned an offset that does not parse");
            check_length_arithmetic(h);
            for j in 0..i {
                assert!(
                    FrameHeader::parse(&data[j..j + 4]).is_err(),
                    "find_sync skipped an earlier valid header at {j}"
                );
            }
        }
        None => {
            if data.len() >= 4 {
                for j in 0..=data.len() - 4 {
                    assert!(
                        FrameHeader::parse(&data[j..j + 4]).is_err(),
                        "find_sync missed a valid header at {j}"
                    );
                }
            }
        }
    }

    if data.len() < 6 {
        return;
    }
    let ctl = data[0];
    let flags = data[1];

    // --- 3. Free-format probe over an attacker-shaped tail. ---
    let hdr = build_header(ctl & 1, (ctl >> 1) & 1, ctl >> 2, None, ctl >> 4, flags);
    let h = FrameHeader::parse(&hdr).expect("crafted free-format header must parse");
    assert!(matches!(h.bitrate, Bitrate::Free));
    let tail = &data[2..];
    if let Ok(f) = detect_free_format_frame_length(&h, tail) {
        let sb = slot_bytes(h.layer);
        let pad = u32::from(h.padding);
        assert_eq!(
            f.frame_length_bytes,
            (f.base_slot_count + pad) * sb,
            "probe length disagrees with recovered slot count"
        );
        assert!(
            f.frame_length_bytes >= 4,
            "probe recovered a frame shorter than its header"
        );
        // The recovered next-frame offset (relative to the tail) must
        // hold a header for the same stream.
        let off = (f.frame_length_bytes as usize) - 4;
        let nh = FrameHeader::parse(&tail[off..off + 4])
            .expect("probe offset does not hold a parsable header");
        assert_eq!(nh.id, h.id);
        assert_eq!(nh.layer, h.layer);
        assert_eq!(nh.sampling_frequency, h.sampling_frequency);
        assert_eq!(nh.mode, h.mode);
    }

    // --- 4. Planted-distance oracle. ---
    // Choose an exact slot distance and build [hdr][fill][next-hdr];
    // the probe must recover exactly the planted geometry. Keep the
    // fill free of accidental earlier syncwords by zeroing it.
    let sb = slot_bytes(h.layer);
    let pad = u32::from(h.padding);
    // Total slots for the *current* frame: base N + padding slot. The
    // minimum keeps the frame at least large enough for its header
    // plus the optional 16-bit CRC word (4 + 2 bytes, §2.4.2.1).
    let min_slots = match h.layer {
        Layer::I => 2,
        Layer::II => 6,
    };
    let n_slots = min_slots + u32::from(data[2] % 64) + pad;
    let frame_len = (n_slots * sb) as usize;
    let mut stream = vec![0u8; frame_len + 4];
    stream[..4].copy_from_slice(&hdr);
    // Next frame: same stream parameters (§2.4.3.1 requires the match);
    // its own padding bit is attacker-chosen and must not affect the
    // current frame's recovered length.
    let next = build_header(ctl & 1, (ctl >> 1) & 1, ctl >> 2, None, ctl >> 4, data[3]);
    stream[frame_len..].copy_from_slice(&next);
    let f = detect_free_format_frame_length(&h, &stream[4..])
        .expect("planted two-frame stream must probe successfully");
    assert_eq!(
        f.frame_length_bytes as usize, frame_len,
        "probe did not recover the planted frame length"
    );
    assert_eq!(
        f.base_slot_count,
        n_slots - pad,
        "probe did not recover the planted base slot count"
    );
});
