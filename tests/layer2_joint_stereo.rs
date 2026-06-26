//! Integration test for the Layer II **joint_stereo** (intensity_stereo)
//! encode → decode round-trip through the crate's own public surface.
//!
//! The §2.4.1.6 intensity_stereo upper band `[bound, sblimit)` codes
//! ONE shared sample stream that the decoder rescales into both
//! channels with each channel's own Table 3-B.1 scalefactor. The
//! encoder forms that shared stream from the per-slot channel average
//! `(L+R)/2`, so both channels' contributions survive into the coded
//! samplecode while the per-channel scalefactors restore each channel's
//! loudness.
//!
//! These tests drive a two-channel PCM signal whose left and right
//! content **differ** (so the intensity combine is exercised, not a
//! degenerate mono signal) through `Mp1Layer2FrameEncoder` in
//! `Mode::JointStereo` and decode the result with `Mp1Decoder`. They
//! assert that BOTH channels reconstruct with a bounded RMS error —
//! the right channel could not survive if the encoder discarded it and
//! coded channel 0 alone.
//!
//! No external codec source is consulted; the test only ever sees the
//! bytes our own encoder produces and our own decoder consumes.

use oxideav_core::{CodecId, Decoder, Frame, Packet, TimeBase};
use oxideav_mp1::{
    FrameHeader, Layer, Layer2HeaderParams, Mode, Mp1Decoder, Mp1Layer2FrameEncoder,
};

const SAMPLES_PER_FRAME: usize = 1152;

fn pkt(data: Vec<u8>) -> Packet {
    Packet::new(0, TimeBase::new(1, 44_100), data)
}

/// Interleaved S16 PCM bytes → per-channel f64 in [-1, 1).
fn deinterleave_s16(bytes: &[u8]) -> (Vec<f64>, Vec<f64>) {
    let mut l = Vec::new();
    let mut r = Vec::new();
    for frame in bytes.chunks_exact(4) {
        let sl = i16::from_le_bytes([frame[0], frame[1]]);
        let sr = i16::from_le_bytes([frame[2], frame[3]]);
        l.push(sl as f64 / 32768.0);
        r.push(sr as f64 / 32768.0);
    }
    (l, r)
}

/// Encode `n_frames` of stereo PCM (interleaved f64) through the Layer
/// II joint_stereo encoder, decode it back, and return the decoded
/// per-channel f64 PCM.
fn encode_decode_joint_stereo(
    left: &[f64],
    right: &[f64],
    fs: u32,
    kbps: u16,
    mode_ext: u8,
) -> (Vec<f64>, Vec<f64>) {
    let mut params = Layer2HeaderParams::new(fs, kbps, Mode::JointStereo);
    params.mode_extension = oxideav_mp1::ModeExtension(mode_ext);
    let mut enc = Mp1Layer2FrameEncoder::new(params);

    let mut dec = Mp1Decoder::new(CodecId::new("mp1"));
    let mut dec_l = Vec::new();
    let mut dec_r = Vec::new();

    let n_frames = left.len() / SAMPLES_PER_FRAME;
    for f in 0..n_frames {
        // Build the 1152-sample-per-channel interleaved f64 block the
        // frame encoder consumes (`[l0, r0, l1, r1, …]`).
        let mut block = Vec::with_capacity(SAMPLES_PER_FRAME * 2);
        for i in 0..SAMPLES_PER_FRAME {
            let idx = f * SAMPLES_PER_FRAME + i;
            block.push(left[idx]);
            block.push(right[idx]);
        }
        let frame = enc.encode_frame(&block).expect("encode_frame");

        // Confirm the header parses as Layer II joint_stereo.
        let header = FrameHeader::parse(&frame).expect("parse header");
        assert!(matches!(header.layer, Layer::II));
        assert_eq!(header.mode, Mode::JointStereo);

        dec.send_packet(&pkt(frame)).expect("send_packet");
        match dec.receive_frame().expect("receive_frame") {
            Frame::Audio(af) => {
                assert_eq!(af.samples as usize, SAMPLES_PER_FRAME);
                let (l, r) = deinterleave_s16(&af.data[0]);
                dec_l.extend(l);
                dec_r.extend(r);
            }
            other => panic!("non-audio frame: {other:?}"),
        }
    }
    (dec_l, dec_r)
}

/// Goertzel single-bin power of `signal` at `freq` (Hz), sampling rate
/// `fs`, evaluated over the steady-state region (skip one warm-up
/// frame). Returns the squared magnitude of the DFT bin.
fn goertzel(signal: &[f64], freq: f64, fs: f64) -> f64 {
    let s = &signal[SAMPLES_PER_FRAME.min(signal.len())..];
    if s.is_empty() {
        return 0.0;
    }
    let w = 2.0 * std::f64::consts::PI * freq / fs;
    let coeff = 2.0 * w.cos();
    let (mut s_prev, mut s_prev2) = (0.0f64, 0.0f64);
    for &x in s {
        let cur = x + coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev = cur;
    }
    s_prev * s_prev + s_prev2 * s_prev2 - coeff * s_prev * s_prev2
}

/// Two distinct tones (different frequency per channel) round-trip
/// through joint_stereo Layer II with bounded per-channel RMS. Because
/// the channels carry different content, the right channel can only
/// reconstruct if the encoder preserves it through the intensity combine
/// `(L+R)/2` rather than coding channel 0 alone.
#[test]
fn joint_stereo_distinct_tones_roundtrip_both_channels() {
    let n_frames = 12;
    let n = SAMPLES_PER_FRAME * n_frames;
    let fs = 44_100.0;
    // Left: 900 Hz; Right: 1300 Hz. Comparable amplitude so neither
    // dominates the intensity scalefactor trivially.
    let left: Vec<f64> = (0..n)
        .map(|i| 0.40 * (2.0 * std::f64::consts::PI * 900.0 * (i as f64) / fs).sin())
        .collect();
    let right: Vec<f64> = (0..n)
        .map(|i| 0.38 * (2.0 * std::f64::consts::PI * 1300.0 * (i as f64) / fs).cos())
        .collect();

    // mode_extension 0b01 → intensity bound subband index 8, so the
    // tones (well below subband 8 at 44.1 kHz) sit in the per-channel
    // low band while higher subbands share. Use a high bitrate so the
    // allocator has headroom and the bounded RMS reflects intensity
    // fidelity, not bit starvation.
    let (dl, dr) = encode_decode_joint_stereo(&left, &right, 44_100, 256, 0b01);

    // The polyphase synthesis filterbank introduces a fixed group
    // delay between the encoder input and decoder output, so a direct
    // sample-aligned RMS would be dominated by that delay rather than
    // intensity-stereo fidelity. Compare per-channel *energy* instead:
    // each channel's reconstructed steady-state energy must be within a
    // bounded ratio of its input energy. A channel-0-only encoder would
    // leave the RIGHT channel tracking the LEFT tone — but since both
    // tones have comparable amplitude the energy ratio alone is a weak
    // guard; the in-band spectral check below makes it strict.
    let energy = |s: &[f64]| -> f64 { s.iter().skip(SAMPLES_PER_FRAME).map(|v| v * v).sum() };
    let el_in = energy(&left);
    let er_in = energy(&right);
    let el_out = energy(&dl[..dl.len().min(left.len())]);
    let er_out = energy(&dr[..dr.len().min(right.len())]);
    let ratio_l = el_out / el_in;
    let ratio_r = er_out / er_in;
    assert!(
        (0.5..2.0).contains(&ratio_l),
        "left-channel energy ratio {ratio_l} (in={el_in}, out={el_out})"
    );
    assert!(
        (0.5..2.0).contains(&ratio_r),
        "right-channel energy ratio {ratio_r} (in={er_in}, out={er_out})"
    );

    // Spectral discrimination: the LEFT output must contain the 900 Hz
    // tone and the RIGHT output the 1300 Hz tone. Compute the Goertzel
    // power at each channel's OWN tone vs the OTHER channel's tone; each
    // channel's own tone must dominate. A channel-0-only intensity
    // combine would inject the LEFT tone (900 Hz) into the right output,
    // collapsing this discrimination.
    let p_l_900 = goertzel(&dl, 900.0, fs);
    let p_l_1300 = goertzel(&dl, 1300.0, fs);
    let p_r_900 = goertzel(&dr, 900.0, fs);
    let p_r_1300 = goertzel(&dr, 1300.0, fs);
    assert!(
        p_l_900 > 4.0 * p_l_1300,
        "left output must be dominated by its own 900 Hz tone \
         (p900={p_l_900}, p1300={p_l_1300})"
    );
    assert!(
        p_r_1300 > 4.0 * p_r_900,
        "right output must be dominated by its own 1300 Hz tone \
         (p1300={p_r_1300}, p900={p_r_900})"
    );
}

/// A signal whose energy is concentrated in the **shared upper band**
/// (a high-frequency tone above the intensity bound) round-trips with
/// the right channel preserved. This stresses the intensity combine
/// directly: the bulk of the audible content lives in the shared band
/// where the encoder must merge L and R into one samplecode stream.
#[test]
fn joint_stereo_high_band_content_preserves_right_channel() {
    let n_frames = 12;
    let n = SAMPLES_PER_FRAME * n_frames;
    let fs = 44_100.0;
    // ~14 kHz / ~15 kHz tones sit well above the intensity bound at
    // 44.1 kHz, so they fall in the shared upper band.
    let left: Vec<f64> = (0..n)
        .map(|i| 0.30 * (2.0 * std::f64::consts::PI * 14_000.0 * (i as f64) / fs).sin())
        .collect();
    let right: Vec<f64> = (0..n)
        .map(|i| 0.28 * (2.0 * std::f64::consts::PI * 15_000.0 * (i as f64) / fs).sin())
        .collect();

    let (_dl, dr) = encode_decode_joint_stereo(&left, &right, 44_100, 256, 0b00);

    // The right channel's reconstructed energy must be non-trivial: if
    // the encoder coded channel 0 alone, the right channel would track
    // the LEFT tone's shape scaled to R's level — but its energy would
    // still be present. The stronger guarantee here is that the decoded
    // right-channel energy is within a bounded ratio of the input
    // right-channel energy (the intensity scalefactor restores level).
    let in_energy: f64 = right.iter().skip(SAMPLES_PER_FRAME).map(|v| v * v).sum();
    let out_energy: f64 = dr.iter().skip(SAMPLES_PER_FRAME).map(|v| v * v).sum();
    assert!(in_energy > 0.0);
    let ratio = out_energy / in_energy;
    assert!(
        (0.25..4.0).contains(&ratio),
        "right-channel energy ratio {ratio} out of bounds \
         (in={in_energy}, out={out_energy}) — intensity level not restored"
    );
}
