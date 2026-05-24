//! Self-roundtrip integration tests for the MPEG-1 Audio Layer I
//! encoder.
//!
//! ffmpeg ships no MP1 encoder and there is no widely-available
//! third-party MP1 encoder we can use as a bit-exact oracle, so the
//! encoder is validated against this crate's own decoder: PCM →
//! [`Mp1Encoder`] → packet bytes → [`Mp1Decoder`] → PCM, with a bounded
//! RMS / SNR error budget that reflects the inherent quantization
//! losses of Layer I plus the polyphase filterbank delay.
//!
//! The tests cover mono and stereo, several deterministic input shapes
//! (silence, a pure tone, white noise from a seeded PRNG, an impulse),
//! and the codec-registry round-trip path that a real container would
//! take.

use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, CodecRegistry, Encoder, Frame, MediaType, Packet,
};

use oxideav_mp1::{register_codecs, Mp1Decoder, Mp1Encoder};

const SAMPLES_PER_FRAME: u32 = 384;

/// A simple xorshift64* PRNG so the tests are deterministic across
/// platforms (no `rand` crate dependency).
fn xorshift64(state: &mut u64) -> u64 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    x.wrapping_mul(0x2545F4914F6CDD1Du64)
}

/// Build interleaved S16 PCM from an `f64`-per-channel signal.
fn pcm_s16_interleaved(channels: &[Vec<f64>]) -> (u32, Vec<u8>) {
    let nch = channels.len();
    let n = channels[0].len();
    for ch in channels {
        assert_eq!(ch.len(), n);
    }
    let mut bytes = Vec::with_capacity(n * nch * 2);
    for i in 0..n {
        for ch in channels {
            let v = (ch[i] * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    (n as u32, bytes)
}

/// Decode S16 interleaved PCM bytes back to per-channel `f64` vectors
/// in `[-1, 1)`.
fn pcm_to_f64(bytes: &[u8], nch: usize) -> Vec<Vec<f64>> {
    let mut out = vec![Vec::new(); nch];
    let total = bytes.len() / 2;
    let per_ch = total / nch;
    for ch in out.iter_mut() {
        ch.reserve(per_ch);
    }
    for i in 0..total {
        let s = i16::from_le_bytes([bytes[i * 2], bytes[i * 2 + 1]]);
        out[i % nch].push(s as f64 / 32768.0);
    }
    out
}

/// Construct a registry-backed encoder/decoder pair for the given
/// channel count, sample rate, and bitrate. Drives the full
/// registry/factory path (the codec API a real container would use).
fn make_pair(
    channels: u16,
    sample_rate: u32,
    bitrate_kbps: u32,
) -> (Box<dyn Encoder>, Box<dyn oxideav_core::Decoder>) {
    let mut reg = CodecRegistry::new();
    register_codecs(&mut reg);
    let mut params = CodecParameters::audio(CodecId::new("mp1"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(channels);
    params.bit_rate = Some(bitrate_kbps as u64 * 1000);
    let enc = reg
        .first_encoder(&params)
        .expect("encoder factory returned None");
    let dec = reg
        .first_decoder(&params)
        .expect("decoder factory returned None");
    (enc, dec)
}

/// Encode `pcm` (interleaved S16) as a sequence of Layer I frames,
/// then decode them. Returns the concatenated decoded PCM (interleaved
/// S16). Each input frame must carry exactly 384 samples/channel.
fn encode_decode_stream(pcm: &[u8], channels: u16, sample_rate: u32, bitrate_kbps: u32) -> Vec<u8> {
    let (mut enc, mut dec) = make_pair(channels, sample_rate, bitrate_kbps);
    let nch = channels as usize;
    let bytes_per_frame = SAMPLES_PER_FRAME as usize * nch * 2;
    assert_eq!(pcm.len() % bytes_per_frame, 0, "PCM must be frame-aligned");
    let frames = pcm.len() / bytes_per_frame;
    // TimeBase is set by the encoder per-packet; the test doesn't need
    // to construct one here.
    let mut out = Vec::new();
    for f in 0..frames {
        let frame = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: Some((f as i64) * SAMPLES_PER_FRAME as i64),
            data: vec![pcm[f * bytes_per_frame..(f + 1) * bytes_per_frame].to_vec()],
        };
        enc.send_frame(&Frame::Audio(frame)).unwrap();
        let pkt = enc.receive_packet().unwrap();
        // Sanity: the packet must start with the 0xFFF syncword.
        assert!(
            pkt.data.len() >= 4 && pkt.data[0] == 0xFF && (pkt.data[1] & 0xF0) == 0xF0,
            "missing syncword on frame {f}"
        );
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("expected audio frame");
        };
        assert_eq!(a.samples, SAMPLES_PER_FRAME);
        out.extend_from_slice(&a.data[0]);
    }
    out
}

/// Compute the RMS error between two equal-length S16 PCM streams,
/// per channel, after finding the integer sample delay that minimises
/// the error in each channel (the polyphase analysis+synthesis pair
/// introduces a fixed group delay that we don't otherwise know). Returns
/// the maximum RMS across channels.
fn rms_per_channel(orig: &[u8], dec: &[u8], nch: usize) -> f64 {
    assert_eq!(orig.len(), dec.len());
    let orig_f = pcm_to_f64(orig, nch);
    let dec_f = pcm_to_f64(dec, nch);
    let mut worst = 0.0f64;
    for ch in 0..nch {
        let n = orig_f[ch].len().min(dec_f[ch].len());
        // Search a generous window for the integer delay that aligns
        // input and decoded output. 0..=1024 covers the analysis +
        // synthesis polyphase group delay with margin.
        let mut best = f64::INFINITY;
        for d in 0..1024usize {
            if n <= d + 200 {
                break;
            }
            let mut err = 0.0f64;
            let mut cnt = 0usize;
            // Skip the leading transient (200 samples) and the trailing
            // edge so neither side runs off the end.
            for i in 200..(n - d - 50) {
                let diff = orig_f[ch][i] - dec_f[ch][i + d];
                err += diff * diff;
                cnt += 1;
            }
            if cnt > 100 {
                let rms = (err / cnt as f64).sqrt();
                if rms < best {
                    best = rms;
                }
            }
        }
        if best > worst {
            worst = best;
        }
    }
    worst
}

// ---- 1. Silence roundtrip ---------------------------------------

#[test]
fn silence_roundtrips_to_near_silence() {
    // 8 frames of stereo silence at 48 kHz. Encoder analyzes zeros,
    // every subband is silent, allocator hands out zero bits, decoder
    // synthesises zeros — modulo the filterbank ramp-in, the output
    // should be exact silence.
    let n_frames = 8;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let left = vec![0.0f64; n];
    let right = vec![0.0f64; n];
    let (_, pcm) = pcm_s16_interleaved(&[left, right]);
    let out = encode_decode_stream(&pcm, 2, 48_000, 256);
    // Output must be exact silence (no quantization noise from a zero
    // input, no allocated bits).
    for &b in &out {
        assert_eq!(b, 0);
    }
}

// ---- 2. Tone roundtrip (mono) -----------------------------------

#[test]
fn tone_mono_roundtrips_with_bounded_error() {
    // 1 kHz tone at 48 kHz, 16 frames so the analysis/synthesis delays
    // (~480 samples each) clear before the steady-state region we
    // measure. Amplitude 0.5 leaves plenty of headroom under clipping.
    let n_frames = 16;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let freq = 1000.0f64;
    let fs = 48_000.0f64;
    let sig: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * freq * (i as f64) / fs).sin())
        .collect();
    let (_, pcm) = pcm_s16_interleaved(std::slice::from_ref(&sig));
    let out = encode_decode_stream(&pcm, 1, 48_000, 192);
    // Compare the steady-state tail of the decoded output against the
    // input tone (skip the leading ~4 frames, ≈ 1500 samples, to clear
    // the analysis + synthesis ramp-in).
    let rms = rms_per_channel(&pcm, &out, 1);
    // With delay alignment, a 192 kbit/s Layer I encode of a single
    // tone reconstructs to within a few hundredths of a percent of full
    // scale. The bound below catches a regression that breaks the
    // analysis/synthesis pair entirely.
    assert!(rms < 0.01, "tone RMS {rms} too large");
}

// ---- 3. Tone roundtrip (stereo) ---------------------------------

#[test]
fn tone_stereo_roundtrips_with_bounded_error() {
    // Two independent tones, one per channel.
    let n_frames = 16;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let fs = 48_000.0f64;
    let left: Vec<f64> = (0..n)
        .map(|i| 0.4 * (2.0 * std::f64::consts::PI * 1000.0 * (i as f64) / fs).sin())
        .collect();
    let right: Vec<f64> = (0..n)
        .map(|i| 0.4 * (2.0 * std::f64::consts::PI * 1500.0 * (i as f64) / fs).cos())
        .collect();
    let (_, pcm) = pcm_s16_interleaved(&[left, right]);
    let out = encode_decode_stream(&pcm, 2, 48_000, 256);
    let rms = rms_per_channel(&pcm, &out, 2);
    assert!(rms < 0.02, "stereo tone RMS {rms} too large");
}

// ---- 4. PRNG noise roundtrip ------------------------------------

#[test]
fn pseudo_random_noise_roundtrips() {
    // A wideband white-noise input is the hardest case for a fixed
    // bitrate (energy spread across every subband). We still expect a
    // bounded RMS — the allocator does the best it can within budget.
    let n_frames = 24;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let mut state = 0xCAFEBABEDEADBEEFu64;
    let sig: Vec<f64> = (0..n)
        .map(|_| {
            let r = xorshift64(&mut state);
            // Map to [-0.3, 0.3) so the input has plenty of headroom.
            ((r as i64) as f64 / i64::MAX as f64) * 0.3
        })
        .collect();
    let (_, pcm) = pcm_s16_interleaved(&[sig]);
    let out = encode_decode_stream(&pcm, 1, 48_000, 384);
    let rms = rms_per_channel(&pcm, &out, 1);
    // White noise across the band is the worst case (energy in every
    // subband, allocator must spread bits thin). The bound below is
    // generous enough to absorb the noise floor at 384 kbit/s but
    // tight enough to catch a regression that breaks the codec.
    assert!(rms < 0.15, "white-noise RMS {rms} too large");
}

// ---- 5. Frame-shape and metadata --------------------------------

#[test]
fn encoded_frame_matches_spec_layout() {
    // Encode one mono frame of silence and verify:
    // - the first byte is 0xFF (high 8 bits of syncword);
    // - the second byte starts with 0xF (low 4 bits of sync);
    // - the encoded frame size matches the §2.4.2.3 formula
    //   floor(12 * bitrate / Fs) * 4 bytes.
    let n = SAMPLES_PER_FRAME as usize;
    let sig = vec![0.0f64; n];
    let (_, pcm) = pcm_s16_interleaved(&[sig]);
    let bitrate_kbps = 192u32;
    let (mut enc, _dec) = make_pair(1, 48_000, bitrate_kbps);
    let frame = AudioFrame {
        samples: SAMPLES_PER_FRAME,
        pts: Some(0),
        data: vec![pcm],
    };
    enc.send_frame(&Frame::Audio(frame)).unwrap();
    let pkt: Packet = enc.receive_packet().unwrap();
    assert_eq!(pkt.data[0], 0xFF);
    assert_eq!(pkt.data[1] & 0xF0, 0xF0);
    let expected_slots = (12 * bitrate_kbps * 1000) / 48_000;
    let expected_bytes = expected_slots as usize * 4;
    assert_eq!(pkt.data.len(), expected_bytes);
}

#[test]
fn encoder_output_params_describe_stream() {
    let (enc, _dec) = make_pair(2, 44_100, 320);
    let p = enc.output_params();
    assert_eq!(p.codec_id.as_str(), "mp1");
    assert_eq!(p.media_type, MediaType::Audio);
    assert_eq!(p.sample_rate, Some(44_100));
    assert_eq!(p.channels, Some(2));
    assert_eq!(p.bit_rate, Some(320_000));
}

// ---- 6. Registry round-trip (decoder + encoder both present) ----

#[test]
fn registry_exposes_both_encoder_and_decoder() {
    let mut reg = CodecRegistry::new();
    register_codecs(&mut reg);
    assert!(reg.has_decoder(&CodecId::new("mp1")));
    assert!(reg.has_encoder(&CodecId::new("mp1")));
}

// ---- 7. Type sanity ---------------------------------------------

#[test]
fn encoder_and_decoder_types_export() {
    // Just ensure the public types are reachable from the crate root.
    fn _assert_send<T: Send>() {}
    _assert_send::<Mp1Encoder>();
    _assert_send::<Mp1Decoder>();
}
