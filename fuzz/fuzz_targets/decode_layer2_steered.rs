#![no_main]

//! Steered Layer II decode target: pin every §2.4.3.3.1 bit-allocation
//! table selection (11172-3 Annex B Tables 3-B.2a / b / c / d and the
//! 13818-3 Annex B Table B.1 LSF substitute) so each per-table
//! allocation / scfsi / scalefactor / grouped-sample decode path is
//! exercised on every campaign, not just when the generic `decode`
//! target happens to draw the right (Fs, bitrate, mode) combination.
//!
//! The combo list below is fixed (indexed by 4 control bits) and reads
//! straight off the `layer2_bit_allocation_table` decision drivers:
//! sampling frequency × bitrate-per-channel (total / 2 in any
//! non-mono mode), plus the 13818-3 LSF substitution for `ID == 0`,
//! plus one free-format entry so the decoder's typed rejection path
//! stays covered.
//!
//! Two surfaces are driven per crafted frame:
//!
//! 1. the registered [`oxideav_core::Decoder`] (`send_packet` /
//!    `receive_frame`, multi-frame session so the synthesis-filterbank
//!    history and concealment state machine carry over), and
//! 2. the library-level [`oxideav_mp1::decode_layer2_audio_data`] +
//!    [`oxideav_mp1::verify_layer2_crc`] pair on the same body, which
//!    reaches the deep §2.4.1.6 / §2.4.3.3 parse even when the codec
//!    wrapper would conceal the frame after a CRC mismatch.
//!
//! Contract: no panic, no OOB, no debug-build overflow; outputs are
//! discarded.
//!
//! Spec basis: ISO/IEC 11172-3 §2.4.1.6 / §2.4.3.3 + Annex B Tables
//! 3-B.2a..d, 3-B.4; ISO/IEC 13818-3 §2.4.3.1 + Annex B Table B.1.

use libfuzzer_sys::fuzz_target;
use oxideav_core::{CodecId, CodecParameters, CodecRegistry, Decoder, Packet, Rational, TimeBase};
use oxideav_mp1::{decode_layer2_audio_data, verify_layer2_crc, FrameHeader};

const MAX_FRAME_BYTES: usize = 2048;
const MAX_FRAMES_PER_ITER: usize = 6;

/// (id, sampling-frequency index, bitrate index, mode) — one row per
/// table-selection branch. sr index (MPEG-1): 0 = 44.1 kHz, 1 = 48 kHz,
/// 2 = 32 kHz; (LSF): 0 = 22.05 kHz, 1 = 24 kHz, 2 = 16 kHz. Bitrate
/// index is the raw 4-bit header field (MPEG-1 Layer II ladder:
/// 1..=14 → 32..384 kbit/s); 0 = free format. Mode: 0 = stereo,
/// 1 = joint_stereo, 2 = dual_channel, 3 = single_channel.
const COMBOS: [(u8, u8, u8, u8); 16] = [
    // -- Table 3-B.2c: 48/44.1 kHz at 32 or 48 kbit/s per channel --
    (1, 1, 1, 3), // 48 kHz, 32 kbps mono
    (1, 0, 4, 0), // 44.1 kHz, 64 kbps stereo (32/ch)
    (1, 1, 6, 2), // 48 kHz, 96 kbps dual (48/ch)
    // -- Table 3-B.2d: 32 kHz at 32 or 48 kbit/s per channel --
    (1, 2, 1, 3), // 32 kHz, 32 kbps mono
    (1, 2, 6, 0), // 32 kHz, 96 kbps stereo (48/ch)
    // -- Table 3-B.2a: 48 kHz 56..192 /ch; 44.1/32 kHz 56..80 /ch --
    (1, 1, 8, 3),  // 48 kHz, 128 kbps mono
    (1, 1, 14, 0), // 48 kHz, 384 kbps stereo (192/ch)
    (1, 0, 3, 3),  // 44.1 kHz, 56 kbps mono
    (1, 0, 8, 1),  // 44.1 kHz, 128 kbps joint (64/ch)
    (1, 2, 5, 1),  // 32 kHz, 80... 80 kbps joint (40/ch → fallback)
    // -- Table 3-B.2b: 44.1/32 kHz at 96..192 kbit/s per channel --
    (1, 0, 10, 3), // 44.1 kHz, 192 kbps mono
    (1, 2, 13, 0), // 32 kHz, 320 kbps stereo (160/ch)
    // -- 13818-3 LSF Table B.1 (all rates, ID == 0) --
    (0, 0, 5, 3), // 22.05 kHz mono
    (0, 1, 8, 0), // 24 kHz stereo
    (0, 2, 3, 1), // 16 kHz joint
    // -- Free format: decoder-side typed rejection path --
    (1, 0, 0, 3),
];

fn build_header(combo: (u8, u8, u8, u8), ext: u8, flags: u8) -> [u8; 4] {
    let (id, sr, br, mode) = combo;
    let mut raw: u32 = 0xFFF << 20;
    raw |= u32::from(id & 1) << 19;
    raw |= 0b10 << 17; // Layer II
    raw |= u32::from(flags & 1) << 16; // protection ('0' = CRC present)
    raw |= u32::from(br & 0xF) << 12;
    raw |= u32::from(sr % 3) << 10;
    raw |= u32::from((flags >> 1) & 1) << 9; // padding
    raw |= u32::from(mode & 0b11) << 6;
    raw |= u32::from(ext & 0b11) << 4; // joint-stereo bound
    raw.to_be_bytes()
}

fn build_decoder() -> Option<Box<dyn Decoder>> {
    let mut reg = CodecRegistry::new();
    oxideav_mp1::register_codecs(&mut reg);
    let mut params = CodecParameters::audio(CodecId::new("mp1"));
    params.channels = Some(2);
    params.sample_rate = Some(44_100);
    reg.first_decoder(&params).ok()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let mut dec = match build_decoder() {
        Some(d) => d,
        None => return,
    };
    let tb = TimeBase(Rational::new(1, 44_100));

    let frame_count = ((data[0] as usize) % MAX_FRAMES_PER_ITER) + 1;
    let mut cursor = 1usize;
    for _ in 0..frame_count {
        if data.len() - cursor < 3 {
            break;
        }
        let combo = COMBOS[(data[cursor] & 0xF) as usize];
        let ext = data[cursor] >> 4;
        let flags = data[cursor + 1];
        let salt = data[cursor + 2] as usize;
        cursor += 3;

        let hdr = build_header(combo, ext, flags);
        let Ok(h) = FrameHeader::parse(&hdr) else {
            // Free-format / reserved combos may land here; feed the
            // bare header so the decoder's rejection path runs.
            let pkt = Packet::new(0, tb, hdr.to_vec());
            if dec.send_packet(&pkt).is_ok() {
                let _ = dec.receive_frame();
            }
            continue;
        };
        let frame_len = h
            .frame_length_bytes()
            .map(|l| l as usize)
            .unwrap_or(256) // free format: fixed attacker body
            .clamp(4, MAX_FRAME_BYTES);
        let mut frame = vec![0u8; frame_len];
        frame[..4].copy_from_slice(&hdr);
        let span = data.len().max(1);
        for (j, slot) in frame.iter_mut().enumerate().skip(4) {
            *slot = data[(cursor + j + salt) % span];
        }
        cursor = (cursor + 16).min(data.len());

        // Surface 2: library-level deep parse + CRC walk on the body.
        let body_start = if h.has_crc() { 6usize } else { 4usize };
        let _ = verify_layer2_crc(&h, &frame[..4], &frame[4..]);
        if frame.len() > body_start {
            let _ = decode_layer2_audio_data(&h, &frame[body_start..]);
        }

        // Surface 1: the registered decoder session.
        let pkt = Packet::new(0, tb, frame);
        if dec.send_packet(&pkt).is_ok() {
            let _ = dec.receive_frame();
        }
    }

    let _ = dec.flush();
    for _ in 0..4 {
        if dec.receive_frame().is_err() {
            break;
        }
    }
});
