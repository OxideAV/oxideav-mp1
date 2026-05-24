//! Integration test for the Layer II (`audio_data()`) decode path
//! against an ffmpeg-encoded mono `.mp2` reference.
//!
//! The fixture `tests/fixtures/mp2_mono.mp2` is a short 440 Hz sine
//! at 44.1 kHz mono encoded with ffmpeg's built-in `mp2` encoder at
//! 64 kbit/s. `tests/fixtures/mp2_mono_ffmpeg.s16le` is the ffmpeg
//! reference decode of the same file to raw S16LE PCM. Both files
//! were generated as black-box ffmpeg invocations — no ffmpeg source
//! was consulted — and the test only ever sees their bytes.
//!
//! The test confirms:
//!
//! 1. Each frame's stored bytes and header parse cleanly as Layer II
//!    (sblimit, bitrate, sampling frequency, mono).
//! 2. Decoding the first Layer II frame through the crate's
//!    `Mp1Decoder` yields **exactly 1152 PCM samples per channel** —
//!    the Layer II frame granularity (§2.4.2.1).
//! 3. The crate's PCM and ffmpeg's PCM for the same frame share their
//!    overall shape (low first-frame energy from the synthesis-filter
//!    warm-up; the bulk of the steady-state sine matches within a
//!    bounded RMS error budget that reflects only IEEE-754 ordering
//!    differences between the two implementations).
//!
//! The fixture is intentionally small (≈4 kB compressed, 20 frames /
//! 0.5 s of audio) so the test corpus stays light.

use std::path::PathBuf;

use oxideav_core::{CodecId, Decoder, Frame, Packet, TimeBase};
use oxideav_mp1::{find_sync, FrameHeader, Layer, Mode, Mp1Decoder};

/// Build a Packet wrapping the given compressed Layer II frame bytes.
fn pkt(data: Vec<u8>) -> Packet {
    Packet::new(0, TimeBase::new(1, 44_100), data)
}

fn fixture_path(name: &str) -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("tests");
    p.push("fixtures");
    p.push(name);
    p
}

fn read_fixture(name: &str) -> Vec<u8> {
    std::fs::read(fixture_path(name)).unwrap_or_else(|e| {
        panic!(
            "missing test fixture {}: {} (re-generate with `ffmpeg -f lavfi -i sine=440:r=44100:d=0.5 -ac 1 -c:a mp2 -b:a 64k tests/fixtures/mp2_mono.mp2` and `ffmpeg -i mp2_mono.mp2 -f s16le -acodec pcm_s16le tests/fixtures/mp2_mono_ffmpeg.s16le`)",
            fixture_path(name).display(),
            e
        )
    })
}

fn decode_s16_samples(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|w| i16::from_le_bytes([w[0], w[1]]))
        .collect()
}

/// Walk the Layer II `.mp2` stream and return `(frame_offsets,
/// header_of_first_frame)`.
fn scan_frames(stream: &[u8]) -> (Vec<usize>, FrameHeader) {
    let mut offsets = Vec::new();
    let mut cursor = 0;
    let mut first_header: Option<FrameHeader> = None;
    while cursor < stream.len() {
        let Some(rel) = find_sync(&stream[cursor..]) else {
            break;
        };
        let start = cursor + rel;
        let header = match FrameHeader::parse(&stream[start..]) {
            Ok(h) => h,
            Err(_) => {
                cursor = start + 1;
                continue;
            }
        };
        if !matches!(header.layer, Layer::II) {
            cursor = start + 1;
            continue;
        }
        let Some(len) = header.frame_length_bytes() else {
            // Free / forbidden bitrate: not in our fixture.
            cursor = start + 1;
            continue;
        };
        offsets.push(start);
        first_header.get_or_insert(header);
        cursor = start + len as usize;
    }
    (offsets, first_header.expect("at least one Layer II frame"))
}

#[test]
fn mono_fixture_header_is_layer2_mp1_44k1_mono_64kbps() {
    let stream = read_fixture("mp2_mono.mp2");
    let (offsets, header) = scan_frames(&stream);
    assert!(
        offsets.len() >= 10,
        "expected at least 10 Layer II frames, got {}",
        offsets.len()
    );
    assert!(matches!(header.layer, Layer::II));
    assert!(matches!(header.mode, Mode::SingleChannel));
    assert_eq!(header.sampling_frequency, 44_100);
    // 64 kbit/s mono at 44.1 kHz: floor(144 * 64000 / 44100) = 208.
    let len = header.frame_length_bytes().expect("fixed bitrate");
    // 208 or 209 depending on padding; both are valid (the encoder
    // chooses per frame).
    assert!(
        (208..=210).contains(&len),
        "unexpected frame length {len} for L2 64kbps 44.1kHz mono"
    );
}

/// Decode the first Layer II frame and confirm it produces 1152
/// samples per channel of S16 PCM (the §2.4.2.1 frame granularity).
#[test]
fn first_frame_decodes_to_1152_samples() {
    let stream = read_fixture("mp2_mono.mp2");
    let (offsets, header) = scan_frames(&stream);
    let first_len = header.frame_length_bytes().expect("fixed bitrate") as usize;
    let first = &stream[offsets[0]..offsets[0] + first_len];

    let mut dec = Mp1Decoder::new(CodecId::new("mp1"));
    let pkt = pkt(first.to_vec());
    dec.send_packet(&pkt).unwrap();
    match dec.receive_frame().unwrap() {
        Frame::Audio(af) => {
            assert_eq!(af.samples, 1152, "Layer II must emit 1152 samples/channel");
            assert_eq!(af.data.len(), 1, "interleaved single-plane S16 PCM");
            // 1152 samples × 1 channel × 2 bytes = 2304 bytes.
            assert_eq!(af.data[0].len(), 1152 * 2);
        }
        other => panic!("Layer II decoder returned non-audio: {other:?}"),
    }
}

/// Decode every Layer II frame in the fixture and compare against
/// ffmpeg's PCM frame-by-frame on a bounded RMS budget.
///
/// Both implementations run the same polyphase synthesis filterbank
/// from a zero V history, so the first frame ramps from silence
/// through the filterbank's 512-sample warm-up and the steady-state
/// remainder should agree to within a small numeric envelope. We
/// accept any per-frame RMS error <= a generous threshold that catches
/// outright structural decode bugs while tolerating IEEE-754 ordering
/// differences.
#[test]
fn matches_ffmpeg_reference_pcm_on_steady_state() {
    let stream = read_fixture("mp2_mono.mp2");
    let reference = decode_s16_samples(&read_fixture("mp2_mono_ffmpeg.s16le"));
    let (offsets, _) = scan_frames(&stream);
    assert!(!offsets.is_empty());

    let mut dec = Mp1Decoder::new(CodecId::new("mp1"));
    let mut decoded: Vec<i16> = Vec::new();

    // Decode every frame.
    for window in offsets.windows(2) {
        let start = window[0];
        let end = window[1];
        let pkt = pkt(stream[start..end].to_vec());
        dec.send_packet(&pkt).unwrap();
        match dec.receive_frame().unwrap() {
            Frame::Audio(af) => {
                assert_eq!(af.samples, 1152);
                decoded.extend(decode_s16_samples(&af.data[0]));
            }
            other => panic!("non-audio frame: {other:?}"),
        }
    }
    // Decode the final frame too (frame_length_bytes is reliable here).
    if let Some(&last_off) = offsets.last() {
        // Don't bother walking past the last sync; pass the remaining
        // tail and let the decoder consume one frame from it.
        let pkt = pkt(stream[last_off..].to_vec());
        dec.send_packet(&pkt).unwrap();
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            decoded.extend(decode_s16_samples(&af.data[0]));
        }
    }

    // Compare the steady-state region (skip 1 frame of synthesis
    // warm-up + a little extra to clear filter ringing).
    let warmup = 1152 * 2;
    let n = decoded.len().min(reference.len());
    assert!(
        n > warmup + 1152,
        "decoded {} samples, reference {} samples — too short to compare",
        decoded.len(),
        reference.len()
    );

    let steady = warmup..n;
    let mut sq_err = 0f64;
    let mut count = 0u64;
    let mut max_abs_err = 0i32;
    for i in steady.clone() {
        let d = decoded[i] as i32;
        let r = reference[i] as i32;
        let e = d - r;
        sq_err += (e * e) as f64;
        count += 1;
        max_abs_err = max_abs_err.max(e.unsigned_abs() as i32);
    }
    let rms = (sq_err / count as f64).sqrt();

    // Observed on the fixture: RMS = 0.50 LSB, max|err| = 1 LSB —
    // essentially bit-exact, the residual being IEEE-754 ordering
    // differences in the synthesis filter. A 4-LSB ceiling here keeps
    // headroom for harmless multiplication-order reorderings on other
    // hosts while still flagging an outright wrong allocation table or
    // a misdirected scfsi schedule (those would land in the hundreds
    // of LSBs immediately).
    eprintln!(
        "L2 mono PCM diff vs ffmpeg: rms = {rms:.2} LSB, max|err| = {max_abs_err} LSB, n = {count} samples"
    );
    assert!(
        rms < 4.0,
        "steady-state RMS error too large: {rms:.2} LSB (>= 4 LSB threshold)"
    );
    assert!(
        max_abs_err <= 16,
        "max sample error too large: {max_abs_err} LSB (>= 16 LSB cap)"
    );
}
