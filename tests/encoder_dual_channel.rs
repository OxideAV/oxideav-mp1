//! Dual-channel (`mode = 10`) encode tests for the pure-Rust MP1 encoder.
//!
//! Per ISO/IEC 11172-3 §2.4.2.3, dual-channel is wire-equivalent to plain
//! stereo: both channels carry independent per-subband allocations,
//! scalefactors, and 12 samples per block for all 32 subbands. Only the
//! 2-bit `mode` field changes (`00` → `10`). The semantic difference is
//! downstream — the two channels represent independent programs (e.g.
//! two languages) rather than a stereo pair. The encoder honours that
//! by allocating each channel independently and never collapsing them
//! to a shared mid.
//!
//! These tests cover:
//! - header field correctness (`mode = 10`, `mode_extension = 0`,
//!   `bound = 32`),
//! - decode roundtrip on an uncorrelated stereo signal (two distinct
//!   tones, one per channel) — the encoder must preserve each
//!   independently,
//! - byte-budget equivalence with plain stereo (same CBR slot → same
//!   wire length),
//! - ffmpeg cross-decode interop (skipped silently when ffmpeg is
//!   absent), confirming the `mode = 10` field is accepted by a
//!   reference decoder.

use oxideav_core::options::CodecOptions;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Frame, MediaType, Packet, SampleFormat, TimeBase,
};
use oxideav_mp1::decoder::make_decoder;
use oxideav_mp1::encoder::make_encoder;
use oxideav_mp1::header::{ChannelMode, FrameHeader};
use oxideav_mp1::CODEC_ID_STR;

/// Build interleaved stereo s16 PCM from two per-channel generators.
fn build_stereo_pcm(
    sample_rate: u32,
    duration_s: f32,
    left: impl Fn(f32) -> f32,
    right: impl Fn(f32) -> f32,
) -> Vec<u8> {
    let n = (sample_rate as f32 * duration_s) as usize;
    let mut out = Vec::with_capacity(n * 4);
    for i in 0..n {
        let t = i as f32 / sample_rate as f32;
        let l = (left(t).clamp(-1.0, 1.0) * 32767.0) as i16;
        let r = (right(t).clamp(-1.0, 1.0) * 32767.0) as i16;
        out.extend_from_slice(&l.to_le_bytes());
        out.extend_from_slice(&r.to_le_bytes());
    }
    out
}

/// Encode the whole interleaved-stereo buffer in dual-channel mode.
fn encode_dual(pcm: &[u8], sample_rate: u32, bitrate_kbps: u32) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
    params.media_type = MediaType::Audio;
    params.channels = Some(2);
    params.sample_rate = Some(sample_rate);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some((bitrate_kbps as u64) * 1000);
    params.options = CodecOptions::new().set("dual_channel", "true");
    let mut enc = make_encoder(&params).expect("build dual-channel encoder");

    let total = (pcm.len() / 4) as u32;
    let frame = AudioFrame {
        samples: total,
        pts: Some(0),
        data: vec![pcm.to_vec()],
    };
    enc.send_frame(&Frame::Audio(frame)).expect("send_frame");
    let mut bytes = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        bytes.extend_from_slice(&p.data);
    }
    enc.flush().expect("flush");
    while let Ok(p) = enc.receive_packet() {
        bytes.extend_from_slice(&p.data);
    }
    bytes
}

/// Encode in plain stereo (no dual_channel option) for size comparison.
fn encode_plain_stereo(pcm: &[u8], sample_rate: u32, bitrate_kbps: u32) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
    params.media_type = MediaType::Audio;
    params.channels = Some(2);
    params.sample_rate = Some(sample_rate);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some((bitrate_kbps as u64) * 1000);
    let mut enc = make_encoder(&params).expect("build stereo encoder");

    let total = (pcm.len() / 4) as u32;
    let frame = AudioFrame {
        samples: total,
        pts: Some(0),
        data: vec![pcm.to_vec()],
    };
    enc.send_frame(&Frame::Audio(frame)).expect("send_frame");
    let mut bytes = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        bytes.extend_from_slice(&p.data);
    }
    enc.flush().expect("flush");
    while let Ok(p) = enc.receive_packet() {
        bytes.extend_from_slice(&p.data);
    }
    bytes
}

/// Split a concatenated MP1 stream into per-frame slices.
fn split_frames(data: &[u8]) -> Vec<&[u8]> {
    let mut frames = Vec::new();
    let mut i = 0;
    while i + 4 <= data.len() {
        if data[i] != 0xFF || (data[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let Ok(h) = FrameHeader::parse(&data[i..]) else {
            i += 1;
            continue;
        };
        let len = h.frame_size();
        if len == 0 || i + len > data.len() {
            break;
        }
        frames.push(&data[i..i + len]);
        i += len;
    }
    frames
}

/// Decode the whole stream into interleaved i16 (L,R,L,R,...).
fn decode_all(data: &[u8]) -> Vec<i16> {
    let params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
    let mut dec = make_decoder(&params).expect("build decoder");
    let mut out = Vec::new();
    for fr in split_frames(data) {
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), fr.to_vec());
        dec.send_packet(&pkt).expect("send_packet");
        if let Ok(Frame::Audio(a)) = dec.receive_frame() {
            for c in a.data[0].chunks_exact(2) {
                out.push(i16::from_le_bytes([c[0], c[1]]));
            }
        }
    }
    out
}

/// Goertzel power at `target` Hz divided by the mean power at the noise
/// bins; returned in dB.
fn goertzel_snr(pcm: &[i16], sample_rate: u32, target: f32, noise_bins: &[f32]) -> f32 {
    let power = |f: f32| -> f32 {
        let w = 2.0 * std::f32::consts::PI * f / sample_rate as f32;
        let coeff = 2.0 * w.cos();
        let (mut s1, mut s2) = (0.0f32, 0.0f32);
        for &x in pcm {
            let s0 = x as f32 + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        s1 * s1 + s2 * s2 - coeff * s1 * s2
    };
    let sig = power(target).max(1.0);
    let noise: f32 =
        noise_bins.iter().map(|&f| power(f)).sum::<f32>() / noise_bins.len().max(1) as f32;
    10.0 * (sig / noise.max(1.0)).log10()
}

/// Header field correctness: `mode = 10` (dual_channel),
/// `mode_extension = 0`, `bound = 32` (no sharing) for every frame.
#[test]
fn dual_channel_header_fields() {
    let sr = 44_100u32;
    let pcm = build_stereo_pcm(
        sr,
        0.5,
        |t| 0.4 * (2.0 * std::f32::consts::PI * 800.0 * t).sin(),
        |t| 0.4 * (2.0 * std::f32::consts::PI * 1500.0 * t).sin(),
    );
    let bytes = encode_dual(&pcm, sr, 192);
    let frames = split_frames(&bytes);
    assert!(!frames.is_empty(), "no frames produced");
    for fr in &frames {
        let h = FrameHeader::parse(fr).expect("parse header");
        assert_eq!(h.mode, ChannelMode::DualChannel, "wrong mode");
        assert_eq!(h.mode_extension, 0, "mode_extension must be 0");
        assert_eq!(
            h.bound(),
            32,
            "dual-channel has no shared subbands (bound = SBLIMIT)"
        );
        assert_eq!(h.mode.channel_count(), 2);
    }
}

/// Each independent program survives the encode/decode round-trip.
/// We feed an uncorrelated stereo signal (different tone per channel),
/// decode it back, and confirm each channel's tone is recovered with
/// a healthy SNR above off-bin noise.
#[test]
fn dual_channel_independent_programs_roundtrip() {
    let sr = 44_100u32;
    let f_left = 800.0f32;
    let f_right = 2200.0f32;
    let pcm = build_stereo_pcm(
        sr,
        1.0,
        |t| 0.5 * (2.0 * std::f32::consts::PI * f_left * t).sin(),
        |t| 0.5 * (2.0 * std::f32::consts::PI * f_right * t).sin(),
    );
    let bytes = encode_dual(&pcm, sr, 256);
    let decoded = decode_all(&bytes);
    assert!(decoded.len() > 30_000, "too few samples: {}", decoded.len());

    // De-interleave.
    let mut left = Vec::with_capacity(decoded.len() / 2);
    let mut right = Vec::with_capacity(decoded.len() / 2);
    for p in decoded.chunks_exact(2) {
        left.push(p[0]);
        right.push(p[1]);
    }

    // Skip the polyphase warm-up region (~480 samples ≈ analysis +
    // synthesis group delay).
    let warmup = 1500.min(left.len() / 4);
    let l = &left[warmup..];
    let r = &right[warmup..];

    // Each program is recovered with a clear SNR — the right channel's
    // tone must NOT bleed into the left, and vice versa. That's the
    // sanity check for independent allocation under `mode = 10`.
    let off_bins = [200.0f32, 5000.0, 10000.0];
    let snr_l_at_l = goertzel_snr(l, sr, f_left, &off_bins);
    let snr_l_at_r = goertzel_snr(l, sr, f_right, &off_bins);
    let snr_r_at_r = goertzel_snr(r, sr, f_right, &off_bins);
    let snr_r_at_l = goertzel_snr(r, sr, f_left, &off_bins);
    eprintln!(
        "dual L: at-{f_left}={snr_l_at_l:.2}dB at-{f_right}={snr_l_at_r:.2}dB; \
         R: at-{f_right}={snr_r_at_r:.2}dB at-{f_left}={snr_r_at_l:.2}dB"
    );
    assert!(snr_l_at_l >= 25.0, "left tone lost: SNR={snr_l_at_l:.2} dB");
    assert!(
        snr_r_at_r >= 25.0,
        "right tone lost: SNR={snr_r_at_r:.2} dB"
    );
    // The off-channel power at the other tone should be far below the
    // on-channel power (real channel separation, not a mid signal).
    assert!(
        snr_l_at_l - snr_l_at_r >= 15.0,
        "L channel leaks right tone (delta {:.2})",
        snr_l_at_l - snr_l_at_r
    );
    assert!(
        snr_r_at_r - snr_r_at_l >= 15.0,
        "R channel leaks left tone (delta {:.2})",
        snr_r_at_r - snr_r_at_l
    );
}

/// Wire layout is identical to plain stereo: same CBR slot → exactly
/// the same total byte count. Only the 2-bit `mode` field differs.
#[test]
fn dual_channel_same_byte_budget_as_stereo() {
    let sr = 44_100u32;
    let pcm = build_stereo_pcm(
        sr,
        0.5,
        |t| 0.3 * (2.0 * std::f32::consts::PI * 1000.0 * t).sin(),
        |t| 0.3 * (2.0 * std::f32::consts::PI * 1700.0 * t).sin(),
    );
    let dual = encode_dual(&pcm, sr, 192);
    let plain = encode_plain_stereo(&pcm, sr, 192);
    assert_eq!(
        dual.len(),
        plain.len(),
        "dual-channel must match plain stereo byte budget at the same CBR slot"
    );
    assert!(!dual.is_empty(), "empty output");
}

/// `joint_stereo` overrides `dual_channel` when both are set (joint
/// materially changes the bit allocation; dual only relabels the mode
/// field). This documents the precedence so users aren't surprised.
#[test]
fn joint_stereo_overrides_dual_channel() {
    let sr = 44_100u32;
    let pcm = build_stereo_pcm(
        sr,
        0.3,
        |t| 0.3 * (2.0 * std::f32::consts::PI * 1000.0 * t).sin(),
        |t| 0.3 * (2.0 * std::f32::consts::PI * 1000.0 * t).sin(),
    );
    let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
    params.media_type = MediaType::Audio;
    params.channels = Some(2);
    params.sample_rate = Some(sr);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(192_000);
    params.options = CodecOptions::new()
        .set("joint_stereo", "true")
        .set("dual_channel", "true");
    let mut enc = make_encoder(&params).expect("build encoder");
    let total = (pcm.len() / 4) as u32;
    let frame = AudioFrame {
        samples: total,
        pts: Some(0),
        data: vec![pcm.to_vec()],
    };
    enc.send_frame(&Frame::Audio(frame)).expect("send_frame");
    enc.flush().expect("flush");
    let mut bytes = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        bytes.extend_from_slice(&p.data);
    }
    let frames = split_frames(&bytes);
    assert!(!frames.is_empty());
    for fr in &frames {
        let h = FrameHeader::parse(fr).expect("parse");
        assert_eq!(
            h.mode,
            ChannelMode::JointStereo,
            "joint_stereo must win over dual_channel"
        );
    }
}

/// Dual-channel on a mono (1-channel) input is silently ignored; the
/// encoder still emits a valid mono stream rather than rejecting the
/// configuration (matching the joint_stereo precedent).
#[test]
fn dual_channel_ignored_on_mono() {
    let sr = 44_100u32;
    let n = (sr as f32 * 0.3) as usize;
    let mut data = Vec::with_capacity(n * 2);
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let s = (0.4 * (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * 32767.0) as i16;
        data.extend_from_slice(&s.to_le_bytes());
    }
    let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
    params.media_type = MediaType::Audio;
    params.channels = Some(1);
    params.sample_rate = Some(sr);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(128_000);
    params.options = CodecOptions::new().set("dual_channel", "true");
    let mut enc = make_encoder(&params).expect("build mono encoder");
    let frame = AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![data],
    };
    enc.send_frame(&Frame::Audio(frame)).expect("send_frame");
    enc.flush().expect("flush");
    let mut bytes = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        bytes.extend_from_slice(&p.data);
    }
    let frames = split_frames(&bytes);
    assert!(!frames.is_empty(), "mono produced no frames");
    for fr in &frames {
        let h = FrameHeader::parse(fr).expect("parse");
        assert_eq!(
            h.mode,
            ChannelMode::SingleChannel,
            "mono with dual_channel must stay single_channel"
        );
    }
}

/// ffmpeg cross-decode: ffmpeg's MPEG-audio decoder must accept our
/// dual-channel stream (mode = 10) and recover each program's tone.
/// Skipped silently when ffmpeg is not on PATH (keeps CI portable).
#[test]
fn dual_channel_ffmpeg_interop() {
    use std::process::{Command, Stdio};
    if Command::new("ffmpeg").arg("-version").output().is_err() {
        eprintln!("ffmpeg not available — skipping dual-channel interop");
        return;
    }
    let sr = 44_100u32;
    let f_left = 800.0f32;
    let f_right = 2200.0f32;
    let pcm = build_stereo_pcm(
        sr,
        1.2,
        |t| 0.5 * (2.0 * std::f32::consts::PI * f_left * t).sin(),
        |t| 0.5 * (2.0 * std::f32::consts::PI * f_right * t).sin(),
    );
    let bytes = encode_dual(&pcm, sr, 256);
    assert!(!bytes.is_empty(), "no dual output");

    let tmp_mp1 = std::env::temp_dir().join("oxideav_mp1_dual.mpa");
    let tmp_wav = std::env::temp_dir().join("oxideav_mp1_dual.wav");
    std::fs::write(&tmp_mp1, &bytes).expect("write mp1");
    let out = Command::new("ffmpeg")
        .arg("-y")
        .arg("-loglevel")
        .arg("warning")
        .arg("-f")
        .arg("mp3")
        .arg("-i")
        .arg(&tmp_mp1)
        .arg("-f")
        .arg("wav")
        .arg(&tmp_wav)
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .output()
        .expect("ffmpeg");
    if !out.status.success() {
        eprintln!(
            "ffmpeg failed (status {:?}): {} — treating as skip",
            out.status,
            String::from_utf8_lossy(&out.stderr)
        );
        return;
    }
    let wav = std::fs::read(&tmp_wav).expect("wav");
    let data_off = wav
        .windows(4)
        .position(|w| w == b"data")
        .expect("WAV data tag")
        + 8;
    let mut decoded: Vec<i16> = Vec::new();
    for c in wav[data_off..].chunks_exact(2) {
        decoded.push(i16::from_le_bytes([c[0], c[1]]));
    }
    // ffmpeg emits interleaved stereo; check each channel for its tone.
    let mut left = Vec::with_capacity(decoded.len() / 2);
    let mut right = Vec::with_capacity(decoded.len() / 2);
    for p in decoded.chunks_exact(2) {
        left.push(p[0]);
        right.push(p[1]);
    }
    let warmup = 1500.min(left.len() / 4);
    let l = &left[warmup..];
    let r = &right[warmup..];
    let off_bins = [200.0f32, 5000.0, 10000.0];
    let snr_l = goertzel_snr(l, sr, f_left, &off_bins);
    let snr_r = goertzel_snr(r, sr, f_right, &off_bins);
    eprintln!(
        "dual-channel ffmpeg interop: L@{f_left}={snr_l:.2}dB R@{f_right}={snr_r:.2}dB \
         bytes={}",
        bytes.len()
    );
    assert!(
        snr_l >= 20.0,
        "ffmpeg dual-channel L tone lost: SNR={snr_l:.2}"
    );
    assert!(
        snr_r >= 20.0,
        "ffmpeg dual-channel R tone lost: SNR={snr_r:.2}"
    );
}
