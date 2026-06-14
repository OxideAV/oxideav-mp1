#![no_main]

//! Drive attacker-supplied bytes through `Decoder::send_packet` /
//! `receive_frame` on a fresh MPEG-1 / MPEG-2 LSF Audio Layer I /
//! Layer II decoder.
//!
//! Round 296 depth-mode lane: a panic-freedom fuzzer over the decoder's
//! attacker surface. The contract under test is purely that every
//! public entry point on the registered `Decoder` trait object returns
//! a `Result` (never panics, never integer-overflows in a debug build,
//! never indexes out of bounds) on arbitrary input, across a
//! multi-packet stream so the cross-frame synthesis-filterbank
//! overlap-add and the §2.4.3.1 concealment state machine are
//! exercised — not just a single cold-start frame.
//!
//! ## Why structurally-valid headers
//!
//! A free-form byte fuzzer almost never produces the 12-bit frame sync
//! (ISO/IEC 11172-3 §2.4.2.3), so it spends all its energy on the
//! `BadSync` rejection inside `find_sync` and never reaches header
//! parse, bit-allocation read, scalefactor decode, requantisation, or
//! the polyphase synthesis filterbank. This target instead *constructs*
//! a valid 4-byte header whose every field (ID/version, layer I vs II,
//! bitrate index, sample-rate index, channel mode, mode extension, CRC
//! flag, padding) is attacker-chosen, then sizes the remainder to the
//! header-implied frame length and fills it with attacker bytes for the
//! optional CRC slot, the allocation / scalefactor field, and the
//! sample slots. The deep decode chain is therefore reached on
//! essentially every iteration, with the bytes past the header fully
//! attacker-controlled.
//!
//! ## Cross-frame state
//!
//! The §2.4.3.2 polyphase synthesis filterbank carries overlap-add `V`
//! history across frames, and the §2.4.3.1 concealment path stores the
//! last successfully-decoded subbands for `RepeatPrevious`. A single
//! cold-start packet never exercises those carry-overs. This target
//! stitches several crafted frames into one decoder session so the
//! transitions are reachable, and injects a mid-stream `reset()` +
//! `flush()` so the state machine moves through both calls.
//!
//! ## Surfaces driven on every iteration
//!
//! 1. `oxideav_mp1::register_codecs` → `first_decoder` factory surface.
//! 2. `Decoder::send_packet` across a fuzzer-chosen sequence of crafted
//!    frames (valid Layer I or Layer II header + attacker body), plus
//!    raw-byte packets so the `< 4 bytes` / `BadSync` / short-frame
//!    rejections are hit.
//! 3. `Decoder::receive_frame` after each `send_packet` to drain the
//!    pending frame.
//! 4. `Decoder::reset` + `Decoder::flush` mid-stream, then
//!    `receive_frame` until `Eof`.
//!
//! Output PCM is discarded — the fuzzer only cares about *return*,
//! never sample-correctness (that is the integration tests' job).
//!
//! Spec basis: ISO/IEC 11172-3 §2.4.2.3 (frame header / sync),
//! §2.4.1.4 / §2.4.3.1 (CRC + concealment), §2.4.1.5 / §2.4.3.2
//! (Layer I requantisation + filterbank), §2.4.1.6 (Layer II
//! allocation / scfsi).

use libfuzzer_sys::fuzz_target;
use oxideav_core::{CodecId, CodecParameters, CodecRegistry, Decoder, Packet, Rational, TimeBase};

const SAMPLE_RATE: u32 = 44_100;
const MAX_PACKETS_PER_ITER: usize = 12;
/// Bound the crafted-frame body so a malicious bitrate/sample-rate
/// combination can't pin a worker on a multi-kilobyte allocation per
/// frame. The largest legal Layer II frame is ~1700 bytes (384 kbit/s
/// @ 32 kHz over 144-byte-equivalent slots); this cap covers it with
/// margin.
const MAX_FRAME_BYTES: usize = 2048;
/// Number of header-control bytes consumed per crafted frame.
const HEADER_CTL_BYTES: usize = 6;

/// Assemble a structurally-valid 4-byte Layer I / Layer II header from
/// attacker-chosen field bits, returning the four header bytes.
///
/// The field choices are constrained to the *accepted* ranges so the
/// header actually parses (`FrameHeader::parse` only returns the deep
/// chain on a parsing header); within those ranges every field is
/// attacker-controlled.
///
/// `id` selects MPEG-1 vs MPEG-2 LSF (the bitrate / sample-rate
/// ladders), `layer` selects Layer I vs Layer II, `sr` the sample-rate
/// index (0..=2; 3 is reserved/rejected), `br` the bitrate index
/// (1..=14, never 0 free-format or 15 forbidden), `mode` the channel
/// mode, and the remaining control bits drive the CRC, padding,
/// mode-extension, and private/copyright/original flags.
fn build_header(id: u8, layer: u8, sr: u8, br: u8, mode: u8, ext: u8, flags: u8) -> [u8; 4] {
    // 12-bit sync (bits 31..20 all ones).
    let mut raw: u32 = 0xFFF << 20;
    // ID: 1 = MPEG-1 (11172-3), 0 = MPEG-2 LSF (13818-3).
    raw |= u32::from(id & 1) << 19;
    // Layer: 0b11 = Layer I, 0b10 = Layer II (the only two this crate
    // accepts; 0b01 / 0b00 are rejected by `parse`).
    let layer_bits = if layer & 1 == 0 { 0b11u32 } else { 0b10u32 };
    raw |= layer_bits << 17;
    // protection_bit: '0' = CRC present, '1' = none.
    let crc = flags & 0b0000_0001;
    raw |= u32::from(crc ^ 1) << 16; // store '0' when CRC present
                                     // bitrate index 1..=14 (avoid 0 free-format and 15 forbidden so the
                                     // header-implied frame length is determinable).
    let br_idx = (br % 14) + 1;
    raw |= u32::from(br_idx) << 12;
    // sample-rate index 0..=2 (3 is reserved/rejected).
    let sr_idx = sr % 3;
    raw |= u32::from(sr_idx) << 10;
    // padding bit.
    raw |= u32::from((flags >> 1) & 1) << 9;
    // private bit.
    raw |= u32::from((flags >> 2) & 1) << 8;
    // mode 0..=3 (stereo / joint / dual / single).
    raw |= u32::from(mode & 0b11) << 6;
    // mode extension 0..=3 (Layer I/II joint-stereo subband bound).
    raw |= u32::from(ext & 0b11) << 4;
    // copyright / original / emphasis (emphasis 0b00 = none).
    raw |= u32::from((flags >> 3) & 1) << 3;
    raw |= u32::from((flags >> 4) & 1) << 2;
    raw.to_be_bytes()
}

fn build_decoder() -> Option<Box<dyn Decoder>> {
    let mut reg = CodecRegistry::new();
    oxideav_mp1::register_codecs(&mut reg);
    let mut params = CodecParameters::audio(CodecId::new("mp1"));
    params.channels = Some(2);
    params.sample_rate = Some(SAMPLE_RATE);
    reg.first_decoder(&params).ok()
}

fuzz_target!(|data: &[u8]| {
    let mut dec = match build_decoder() {
        Some(d) => d,
        None => return,
    };
    let tb = TimeBase(Rational::new(1, SAMPLE_RATE as i64));

    // A 0/1-byte buffer still exercises the `< 4 bytes` rejection on a
    // cold decoder.
    if data.is_empty() {
        let pkt = Packet::new(0, tb, Vec::new());
        if dec.send_packet(&pkt).is_ok() {
            let _ = dec.receive_frame();
        }
        return;
    }

    let packet_count = ((data[0] as usize) % MAX_PACKETS_PER_ITER) + 1;
    // Deterministic mid-stream reset / flush hooks (derived from
    // data[0] so libfuzzer's minimiser can still shrink reliably).
    let reset_at = if packet_count >= 3 {
        (data[0] as usize) % packet_count
    } else {
        usize::MAX
    };
    let flush_at = if packet_count >= 2 {
        ((data[0] >> 4) as usize) % packet_count
    } else {
        usize::MAX
    };

    let mut cursor = 1usize;
    for i in 0..packet_count {
        if cursor >= data.len() {
            break;
        }
        // One control byte selects how this packet is shaped.
        let ctl = data[cursor];
        cursor += 1;

        let body: Vec<u8> = if ctl & 0b1000_0000 != 0 {
            // Raw-byte packet: a short, attacker-chosen slice fed
            // verbatim so the `< 4 bytes` / `BadSync` / short-frame
            // rejections are reached with no constructed header.
            let want = (ctl as usize & 0x3f).min(data.len().saturating_sub(cursor));
            let slice = data[cursor..cursor + want].to_vec();
            cursor += want;
            slice
        } else if data.len() - cursor < HEADER_CTL_BYTES {
            // Not enough to craft a header; feed the tail raw.
            let slice = data[cursor..].to_vec();
            cursor = data.len();
            slice
        } else {
            // Crafted frame: valid header drawn from the next 6 bytes,
            // body filled from the bytes after. The layer is selected
            // by the low bit of `ctl` so both the Layer I and Layer II
            // decode chains are reachable; ID by the next bit so the
            // 13818-3 LSF ladders are also exercised.
            let id = (ctl >> 1) & 1;
            let layer = ctl & 1;
            let sr = data[cursor];
            let br = data[cursor + 1];
            let mode = data[cursor + 2];
            let ext = data[cursor + 3];
            let flags = data[cursor + 4];
            // data[cursor + 5] reserved as a body-stride salt.
            let salt = data[cursor + 5];
            cursor += HEADER_CTL_BYTES;
            let hdr = build_header(id, layer, sr, br, mode, ext, flags);

            // Re-derive the frame length the decoder will compute so the
            // body is the right size to satisfy its length check (else
            // the short-frame rejection fires first and the deep chain
            // is never reached). Fall back to a fixed length on the rare
            // parse miss.
            let frame_len = oxideav_mp1::FrameHeader::parse(&hdr)
                .ok()
                .and_then(|h| h.frame_length_bytes())
                .unwrap_or(256) as usize;
            let frame_len = frame_len.clamp(4, MAX_FRAME_BYTES);
            let mut frame = vec![0u8; frame_len];
            frame[..4].copy_from_slice(&hdr);
            // Fill the rest from attacker bytes, walking the buffer with
            // a per-frame salt so successive frames draw different body
            // shapes even from the same tail bytes.
            let span = data.len().max(1);
            for (j, slot) in frame.iter_mut().enumerate().skip(4) {
                let idx = (cursor + j + salt as usize) % span;
                *slot = data[idx];
            }
            // Advance the cursor by a stride so successive frames draw
            // fresh body bytes.
            cursor = (cursor + 16).min(data.len());
            frame
        };

        let pkt = Packet::new(0, tb, body);
        // send_packet stashes the packet; receive_frame runs the decode.
        // Either may legitimately Err on a truncated tail, an
        // unsupported header (free-format / reserved sample rate), or a
        // short frame — that's the contract. The output frame is drained
        // immediately to keep the pending slot clear for the next send.
        if dec.send_packet(&pkt).is_ok() {
            let _ = dec.receive_frame();
        }

        if i == flush_at {
            let _ = dec.flush();
            for _ in 0..4 {
                if dec.receive_frame().is_err() {
                    break;
                }
            }
        }
        if i == reset_at {
            let _ = dec.reset();
        }
    }

    // Final flush + drain, regardless of where the loop exited. A forced
    // double-flush exercises the idempotent post-flush `Eof` path.
    let _ = dec.flush();
    let _ = dec.flush();
    for _ in 0..4 {
        if dec.receive_frame().is_err() {
            break;
        }
    }
});
