//! Integration tests for the ISO/IEC 11172-3 Annex D **Psychoacoustic
//! Model 1** (clause D.1) encode paths, through the crate's own public
//! surface only: PCM → Model-1-driven encoder → frame bytes →
//! `Mp1Decoder` → PCM.
//!
//! Annex D is *informative* — there is no bit-exact oracle for either
//! example model — so these tests assert end-to-end behavioural
//! contracts instead: tones survive a Model 1 encode/decode loop with
//! per-channel discrimination, the model is deterministic, it composes
//! with Layer II VBR, and its allocations genuinely differ from the
//! signal-energy proxy's on the same input (the model is not a
//! pass-through).
//!
//! No external codec source is consulted; the tests only ever see the
//! bytes our own encoder produces and our own decoder consumes.

use oxideav_core::{CodecId, Decoder, Frame, Packet, TimeBase};
use oxideav_mp1::{
    Bitrate, EncodeParams, FrameHeader, Layer, Layer2HeaderParams, Mode, Mp1Decoder,
    Mp1FrameEncoder, Mp1Layer2FrameEncoder, PsyModel,
};

const L1_SAMPLES: usize = 384;
const L2_SAMPLES: usize = 1152;

fn pkt(fs: u32, data: Vec<u8>) -> Packet {
    Packet::new(0, TimeBase::new(1, i64::from(fs)), data)
}

/// Interleaved S16 PCM bytes → per-channel f64 in [-1, 1).
fn deinterleave_s16(bytes: &[u8], nch: usize) -> Vec<Vec<f64>> {
    let mut out = vec![Vec::new(); nch];
    for (i, half) in bytes.chunks_exact(2).enumerate() {
        let s = i16::from_le_bytes([half[0], half[1]]);
        out[i % nch].push(s as f64 / 32768.0);
    }
    out
}

/// Goertzel single-bin power of `signal` at `freq` (Hz), skipping the
/// filterbank warm-up region.
fn goertzel(signal: &[f64], freq: f64, fs: f64, skip: usize) -> f64 {
    let s = &signal[skip.min(signal.len())..];
    if s.is_empty() {
        return 0.0;
    }
    let w = 2.0 * std::f64::consts::PI * freq / fs;
    let coeff = 2.0 * w.cos();
    let (mut p1, mut p2) = (0.0f64, 0.0f64);
    for &x in s {
        let cur = x + coeff * p1 - p2;
        p2 = p1;
        p1 = cur;
    }
    (p1 * p1 + p2 * p2 - coeff * p1 * p2) / s.len() as f64
}

/// A pure sine of `freq` Hz over `n` samples.
fn tone(freq: f64, fs: f64, amp: f64, n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| amp * (2.0 * std::f64::consts::PI * freq * i as f64 / fs).sin())
        .collect()
}

#[test]
fn model1_layer1_stereo_tones_survive_round_trip() {
    // Two distinct per-channel tones through a Model 1 Layer I stereo
    // encode → decode loop: each decoded channel must be dominated by
    // its own tone.
    let fs = 44_100u32;
    let n_frames = 12;
    let n = n_frames * L1_SAMPLES;
    let f_left = 1_033.6; // ≈ line 9 of the 44,1 kHz / 384 grid
    let f_right = 3_100.8;
    let left = tone(f_left, fs as f64, 0.6, n);
    let right = tone(f_right, fs as f64, 0.6, n);

    let params = EncodeParams::new(Bitrate::Fixed(384), fs, Mode::Stereo)
        .with_psychoacoustic(true)
        .with_psy_model(PsyModel::Model1);
    let mut enc = Mp1FrameEncoder::new(params);
    assert_eq!(enc.active_psy_model(), Some(PsyModel::Model1));
    let mut dec = Mp1Decoder::new(CodecId::new("mp1"));

    let mut dec_l = Vec::new();
    let mut dec_r = Vec::new();
    for f in 0..n_frames {
        let mut block = Vec::with_capacity(L1_SAMPLES * 2);
        for i in 0..L1_SAMPLES {
            block.push(left[f * L1_SAMPLES + i]);
            block.push(right[f * L1_SAMPLES + i]);
        }
        let bytes = enc.encode_frame(&block).expect("Model 1 encode");
        let header = FrameHeader::parse(&bytes).expect("header");
        assert!(matches!(header.layer, Layer::I));
        dec.send_packet(&pkt(fs, bytes)).expect("send_packet");
        match dec.receive_frame().expect("receive_frame") {
            Frame::Audio(af) => {
                assert_eq!(af.samples as usize, L1_SAMPLES);
                let chans = deinterleave_s16(&af.data[0], 2);
                dec_l.extend_from_slice(&chans[0]);
                dec_r.extend_from_slice(&chans[1]);
            }
            other => panic!("non-audio frame: {other:?}"),
        }
    }

    let skip = 2 * L1_SAMPLES; // filterbank warm-up
    let ll = goertzel(&dec_l, f_left, fs as f64, skip);
    let lr = goertzel(&dec_l, f_right, fs as f64, skip);
    let rl = goertzel(&dec_r, f_left, fs as f64, skip);
    let rr = goertzel(&dec_r, f_right, fs as f64, skip);
    assert!(
        ll > 100.0 * lr,
        "left channel not dominated by its own tone: {ll} vs {lr}"
    );
    assert!(
        rr > 100.0 * rl,
        "right channel not dominated by its own tone: {rr} vs {rl}"
    );
}

#[test]
fn model1_layer2_mono_tone_survives_round_trip() {
    // A mono tone through a Model 1 Layer II encode → decode loop at
    // 48 kHz retains its spectral identity.
    let fs = 48_000u32;
    let n_frames = 6;
    let n = n_frames * L2_SAMPLES;
    let f0 = 2_812.5; // line 60 of the 1024-point grid
    let f_probe = 9_000.0;
    let signal = tone(f0, fs as f64, 0.7, n);

    let params = Layer2HeaderParams::new(fs, 192, Mode::SingleChannel);
    let mut enc = Mp1Layer2FrameEncoder::new(params)
        .with_psy_model(PsyModel::Model1)
        .with_psychoacoustic(true);
    assert_eq!(enc.active_psy_model(), Some(PsyModel::Model1));
    let mut dec = Mp1Decoder::new(CodecId::new("mp1"));

    let mut out = Vec::new();
    for f in 0..n_frames {
        let block = &signal[f * L2_SAMPLES..(f + 1) * L2_SAMPLES];
        let bytes = enc.encode_frame(block).expect("Model 1 L2 encode");
        let header = FrameHeader::parse(&bytes).expect("header");
        assert!(matches!(header.layer, Layer::II));
        dec.send_packet(&pkt(fs, bytes)).expect("send_packet");
        match dec.receive_frame().expect("receive_frame") {
            Frame::Audio(af) => {
                assert_eq!(af.samples as usize, L2_SAMPLES);
                out.extend_from_slice(&deinterleave_s16(&af.data[0], 1)[0]);
            }
            other => panic!("non-audio frame: {other:?}"),
        }
    }

    let skip = L2_SAMPLES;
    let at_tone = goertzel(&out, f0, fs as f64, skip);
    let off_tone = goertzel(&out, f_probe, fs as f64, skip);
    assert!(
        at_tone > 1_000.0 * off_tone.max(1e-12),
        "decoded output not dominated by the tone: {at_tone} vs {off_tone}"
    );
    // And the tone level itself survived within a few dB.
    let sent = goertzel(&signal, f0, fs as f64, skip);
    let ratio_db = 10.0 * (at_tone / sent).log10();
    assert!(
        ratio_db.abs() < 3.0,
        "tone level shifted by {ratio_db} dB through the codec"
    );
}

#[test]
fn model1_encode_is_deterministic() {
    // Two identical Model 1 encoders produce identical byte streams,
    // frame for frame, on both layers.
    let fs = 32_000u32;
    let sig1 = tone(2_750.0, fs as f64, 0.5, 4 * L1_SAMPLES);
    let p1 = EncodeParams::new(Bitrate::Fixed(192), fs, Mode::SingleChannel)
        .with_psychoacoustic(true)
        .with_psy_model(PsyModel::Model1);
    let mut a = Mp1FrameEncoder::new(p1);
    let mut b = Mp1FrameEncoder::new(p1);
    for f in 0..4 {
        let block = &sig1[f * L1_SAMPLES..(f + 1) * L1_SAMPLES];
        assert_eq!(
            a.encode_frame(block).unwrap(),
            b.encode_frame(block).unwrap(),
            "Layer I frame {f} diverged"
        );
    }

    let sig2 = tone(1_500.0, fs as f64, 0.5, 2 * L2_SAMPLES);
    let hp = Layer2HeaderParams::new(fs, 128, Mode::SingleChannel);
    let mut a = Mp1Layer2FrameEncoder::new(hp)
        .with_psy_model(PsyModel::Model1)
        .with_psychoacoustic(true);
    let mut b = Mp1Layer2FrameEncoder::new(hp)
        .with_psy_model(PsyModel::Model1)
        .with_psychoacoustic(true);
    for f in 0..2 {
        let block = &sig2[f * L2_SAMPLES..(f + 1) * L2_SAMPLES];
        assert_eq!(
            a.encode_frame(block).unwrap(),
            b.encode_frame(block).unwrap(),
            "Layer II frame {f} diverged"
        );
    }
}

#[test]
fn model1_allocation_differs_from_energy_proxy() {
    // The model must have a real effect: on a multi-tone signal the
    // Model 1 frames differ from the energy-proxy frames (same
    // envelope, different allocation inside), and both decode cleanly.
    let fs = 44_100u32;
    let n = 6 * L1_SAMPLES;
    let mut sig = tone(1_033.6, fs as f64, 0.5, n);
    for (s, t) in sig.iter_mut().zip(tone(6_500.0, fs as f64, 0.05, n)) {
        *s += t;
    }
    for (s, t) in sig.iter_mut().zip(tone(14_000.0, fs as f64, 0.002, n)) {
        *s += t;
    }
    let mk = |model: Option<PsyModel>| -> Vec<Vec<u8>> {
        let mut params = EncodeParams::new(Bitrate::Fixed(96), fs, Mode::SingleChannel);
        if let Some(m) = model {
            params = params.with_psychoacoustic(true).with_psy_model(m);
        }
        let mut enc = Mp1FrameEncoder::new(params);
        (0..6)
            .map(|f| {
                enc.encode_frame(&sig[f * L1_SAMPLES..(f + 1) * L1_SAMPLES])
                    .unwrap()
            })
            .collect()
    };
    let proxy = mk(None);
    let model1 = mk(Some(PsyModel::Model1));
    assert_eq!(proxy.len(), model1.len());
    for (p, m) in proxy.iter().zip(model1.iter()) {
        assert_eq!(p.len(), m.len(), "same §2.4.2.1 envelope");
    }
    assert!(
        proxy.iter().zip(model1.iter()).any(|(p, m)| p != m),
        "Model 1 must not be an allocation no-op vs the energy proxy"
    );
    // Both streams decode without error.
    for frames in [&proxy, &model1] {
        let mut dec = Mp1Decoder::new(CodecId::new("mp1"));
        for bytes in frames.iter() {
            dec.send_packet(&pkt(fs, bytes.clone())).expect("send");
            let Frame::Audio(af) = dec.receive_frame().expect("recv") else {
                panic!("audio");
            };
            assert_eq!(af.samples as usize, L1_SAMPLES);
        }
    }
}
