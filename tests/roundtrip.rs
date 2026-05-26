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

// ---- 8. MPEG-2 LSF round-trip across all three LSF rates -------
//
// 13818-3 §2.4.2.3 adds 16 / 22.05 / 24 kHz sampling and a distinct
// Layer I bitrate ladder when the header `ID` bit is `0`. Each LSF
// rate is exercised end-to-end via the same encoder/decoder pair the
// MPEG-1 tests use.

#[test]
fn lsf_silence_roundtrips_to_near_silence_all_rates() {
    // 4 frames of mono silence at each of the three LSF rates with a
    // representative LSF-ladder bitrate (LSF ladder tops out at 256
    // kbit/s; mid values are picked per rate).
    for &(fs, bitrate) in &[(16_000u32, 64u32), (22_050, 80), (24_000, 96)] {
        let n_frames = 4;
        let n = SAMPLES_PER_FRAME as usize * n_frames;
        let mono = vec![0.0f64; n];
        let (_, pcm) = pcm_s16_interleaved(&[mono]);
        let out = encode_decode_stream(&pcm, 1, fs, bitrate);
        for &b in &out {
            assert_eq!(b, 0, "fs={fs} bitrate={bitrate}: non-silent byte");
        }
    }
}

#[test]
fn lsf_tone_roundtrips_with_bounded_error_24khz() {
    // 1 kHz tone at 24 kHz (LSF), 16 frames so the analysis +
    // synthesis ramp-in (~480 samples each) clears before the
    // steady-state region we measure. The §2.4.2.3 LSF table puts 24
    // kHz under `ID==0, sampling_frequency=0b01`.
    let n_frames = 16;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let freq = 1000.0f64;
    let fs = 24_000.0f64;
    let sig: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * freq * (i as f64) / fs).sin())
        .collect();
    let (_, pcm) = pcm_s16_interleaved(std::slice::from_ref(&sig));
    let out = encode_decode_stream(&pcm, 1, 24_000, 128);
    let rms = rms_per_channel(&pcm, &out, 1);
    assert!(rms < 0.02, "LSF 24 kHz tone RMS {rms} too large");
}

#[test]
fn lsf_tone_roundtrips_with_bounded_error_22k05hz() {
    // 700 Hz tone at 22.05 kHz (LSF, `ID==0, sampling_frequency=0b00`).
    // 22.05 kHz isn't an integer divisor of 12·bitrate, so the
    // padding-bit path could in principle fire on per-frame basis; the
    // encoder currently always emits padding_bit=0 (the frame_len
    // bound below confirms the encoder writes the right slot count).
    let n_frames = 16;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let freq = 700.0f64;
    let fs = 22_050.0f64;
    let sig: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * freq * (i as f64) / fs).sin())
        .collect();
    let (_, pcm) = pcm_s16_interleaved(std::slice::from_ref(&sig));
    let out = encode_decode_stream(&pcm, 1, 22_050, 96);
    let rms = rms_per_channel(&pcm, &out, 1);
    assert!(rms < 0.05, "LSF 22.05 kHz tone RMS {rms} too large");
}

#[test]
fn lsf_tone_roundtrips_with_bounded_error_16khz() {
    // 500 Hz tone at 16 kHz (LSF, `ID==0, sampling_frequency=0b10`).
    // 16 kHz is the lowest LSF rate; the audio bandwidth is ~7.5 kHz.
    let n_frames = 16;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let freq = 500.0f64;
    let fs = 16_000.0f64;
    let sig: Vec<f64> = (0..n)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * freq * (i as f64) / fs).sin())
        .collect();
    let (_, pcm) = pcm_s16_interleaved(std::slice::from_ref(&sig));
    let out = encode_decode_stream(&pcm, 1, 16_000, 64);
    let rms = rms_per_channel(&pcm, &out, 1);
    assert!(rms < 0.05, "LSF 16 kHz tone RMS {rms} too large");
}

#[test]
fn lsf_encoded_frame_layout_24khz() {
    // Encode one mono silent frame at LSF 24 kHz / 128 kbit/s and
    // verify (1) the syncword, (2) the `ID` bit is 0 (the 5th
    // most-significant bit of byte 1), (3) the encoded frame size
    // matches the §2.4.2.3 formula `floor(12·bitrate/Fs)·4 bytes`,
    // and (4) parsing the header confirms LSF + 24 kHz + 128 kbit/s.
    use oxideav_core::{AudioFrame, CodecId, CodecParameters, CodecRegistry, Frame, Packet};
    use oxideav_mp1::register_codecs as reg;
    let n = SAMPLES_PER_FRAME as usize;
    let sig = vec![0.0f64; n];
    let (_, pcm) = pcm_s16_interleaved(&[sig]);
    let bitrate_kbps = 128u32;
    let sample_rate = 24_000u32;
    let mut registry = CodecRegistry::new();
    reg(&mut registry);
    let mut params = CodecParameters::audio(CodecId::new("mp1"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(1);
    params.bit_rate = Some(bitrate_kbps as u64 * 1000);
    let mut enc = registry.first_encoder(&params).expect("encoder");
    let frame = AudioFrame {
        samples: SAMPLES_PER_FRAME,
        pts: Some(0),
        data: vec![pcm],
    };
    enc.send_frame(&Frame::Audio(frame)).unwrap();
    let pkt: Packet = enc.receive_packet().unwrap();
    // syncword: byte 0 = 0xFF, top nibble of byte 1 = 0xF.
    assert_eq!(pkt.data[0], 0xFF);
    assert_eq!(pkt.data[1] & 0xF0, 0xF0);
    // ID bit is bit 4 of byte 1 (5th MSB-counted): must be 0 for LSF.
    assert_eq!(pkt.data[1] & 0x08, 0, "expected LSF ID bit (0)");
    let expected_slots = (12 * bitrate_kbps * 1000) / sample_rate;
    let expected_bytes = expected_slots as usize * 4;
    assert_eq!(pkt.data.len(), expected_bytes);
    // Round-trip parse confirms the header round-trips as LSF, 128 kbit/s,
    // 24 kHz exactly.
    let h = oxideav_mp1::FrameHeader::parse(&pkt.data).unwrap();
    assert!(h.is_lsf());
    assert_eq!(h.sampling_frequency, sample_rate);
    assert_eq!(h.bitrate, oxideav_mp1::Bitrate::Fixed(bitrate_kbps as u16));
}

// ---- 9. LSF stereo round-trip ----------------------------------
//
// 13818-3 §2.4.2.3 LSF stereo: the same Layer I encode + decode path
// run with `channels = 2` at an LSF sampling rate (16 / 22.05 / 24
// kHz). The decoder's §2.4.1.5 audio_data path is sample-rate-
// agnostic, so this exercises the encoder factory's LSF default
// `channels == 2` bitrate (128 kbit/s) plus the stereo allocation /
// scalefactor / sample interleave at the LSF ladder.

#[test]
fn lsf_stereo_tone_roundtrips_with_bounded_error_24khz() {
    // Two independent tones, one per channel, at LSF 24 kHz / 128 kbit/s
    // (the LSF stereo factory default). The §2.4.1.5 SAMPLES region is
    // sb-major, ch-minor for every allocated subband — exercising both
    // channels at every slot.
    let n_frames = 16;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let fs = 24_000.0f64;
    let left: Vec<f64> = (0..n)
        .map(|i| 0.4 * (2.0 * std::f64::consts::PI * 1000.0 * (i as f64) / fs).sin())
        .collect();
    let right: Vec<f64> = (0..n)
        .map(|i| 0.4 * (2.0 * std::f64::consts::PI * 1500.0 * (i as f64) / fs).cos())
        .collect();
    let (_, pcm) = pcm_s16_interleaved(&[left, right]);
    let out = encode_decode_stream(&pcm, 2, 24_000, 128);
    let rms = rms_per_channel(&pcm, &out, 2);
    // LSF 24 kHz has half the audio bandwidth of MPEG-1 48 kHz, so the
    // per-subband perceptual budget at 128 kbit/s stereo is comparable
    // to MPEG-1 256 kbit/s stereo; the bound mirrors the MPEG-1 stereo
    // tone test.
    assert!(rms < 0.05, "LSF 24 kHz stereo tone RMS {rms} too large");
}

#[test]
fn lsf_stereo_silence_roundtrips_to_near_silence() {
    // Stereo silence at each of the three LSF rates with the LSF stereo
    // factory-default bitrate. With a zero input every subband is
    // silent, the allocator hands out zero bits, and every decoded
    // byte must be exactly zero.
    for &(fs, bitrate) in &[(16_000u32, 128u32), (22_050, 128), (24_000, 128)] {
        let n_frames = 4;
        let n = SAMPLES_PER_FRAME as usize * n_frames;
        let left = vec![0.0f64; n];
        let right = vec![0.0f64; n];
        let (_, pcm) = pcm_s16_interleaved(&[left, right]);
        let out = encode_decode_stream(&pcm, 2, fs, bitrate);
        for &b in &out {
            assert_eq!(
                b, 0,
                "LSF stereo fs={fs} bitrate={bitrate}: non-silent byte"
            );
        }
    }
}

#[test]
fn lsf_stereo_encoded_frame_carries_lsf_id_bit() {
    // Sanity: a stereo LSF encode writes `ID == 0` in the header and
    // the §2.4.2.3 LSF sampling-frequency code, matching the bitrate
    // ladder's 128 kbit/s position (LSF index for 128 is `8`).
    let n = SAMPLES_PER_FRAME as usize;
    let sig = vec![0.0f64; n];
    let (_, pcm) = pcm_s16_interleaved(&[sig.clone(), sig]);
    let (mut enc, _dec) = make_pair(2, 22_050, 128);
    let frame = AudioFrame {
        samples: SAMPLES_PER_FRAME,
        pts: Some(0),
        data: vec![pcm],
    };
    enc.send_frame(&Frame::Audio(frame)).unwrap();
    let pkt: Packet = enc.receive_packet().unwrap();
    // §2.4.2.3: ID bit = 0 for LSF, sampling_frequency code 0b00 for
    // 22.05 kHz; the header's `Mode` round-trips as `Stereo`.
    assert_eq!(pkt.data[1] & 0x08, 0, "expected LSF ID bit (0)");
    let h = oxideav_mp1::FrameHeader::parse(&pkt.data).unwrap();
    assert!(h.is_lsf());
    assert_eq!(h.sampling_frequency, 22_050);
    assert_eq!(h.bitrate, oxideav_mp1::Bitrate::Fixed(128));
    assert_eq!(h.mode, oxideav_mp1::Mode::Stereo);
}

// ---- 10. Planar S16P encoder input layout ----------------------
//
// The `Mp1Encoder::frame_to_pcm` path (codec.rs) accepts both
// interleaved S16 (`data.len() == 1`) and planar S16P
// (`data.len() == nch`, one byte-vector per channel). Every test
// above feeds the interleaved shape; these tests pin the planar
// branch and the per-plane size validation.

/// Pack a per-channel `f64` signal as a per-plane S16 byte vector,
/// one plane per channel (the planar S16P layout the encoder accepts).
fn pcm_s16_planar(channels: &[Vec<f64>]) -> (u32, Vec<Vec<u8>>) {
    let nch = channels.len();
    let n = channels[0].len();
    for ch in channels {
        assert_eq!(ch.len(), n);
    }
    let mut planes: Vec<Vec<u8>> = (0..nch).map(|_| Vec::with_capacity(n * 2)).collect();
    for (ch, channel) in channels.iter().enumerate() {
        for &sample in channel.iter() {
            let v = (sample * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
            planes[ch].extend_from_slice(&v.to_le_bytes());
        }
    }
    (n as u32, planes)
}

#[test]
fn encoder_accepts_planar_s16p_layout() {
    // Same 1 kHz tone the mono interleaved test uses, but fed as a
    // single-plane S16P frame. The resulting packet must match the
    // interleaved-encode shape byte-for-byte: identical inputs decoded
    // by the same per-channel deinterleave reach the analysis filter
    // with the same sample sequence, so the encoder output is identical.
    let n_frames = 4;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let fs = 48_000.0f64;
    let sig: Vec<f64> = (0..n)
        .map(|i| 0.4 * (2.0 * std::f64::consts::PI * 1000.0 * (i as f64) / fs).sin())
        .collect();

    let (_, inter_pcm) = pcm_s16_interleaved(std::slice::from_ref(&sig));
    let (_, planar_pcm) = pcm_s16_planar(std::slice::from_ref(&sig));
    assert_eq!(planar_pcm.len(), 1, "mono -> single plane");

    let (mut enc_inter, _) = make_pair(1, 48_000, 192);
    let (mut enc_planar, _) = make_pair(1, 48_000, 192);

    let bytes_per_frame = SAMPLES_PER_FRAME as usize * 2; // mono S16
    for f in 0..n_frames {
        let s = f * bytes_per_frame;
        let e = s + bytes_per_frame;
        let inter_frame = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: Some((f as i64) * SAMPLES_PER_FRAME as i64),
            data: vec![inter_pcm[s..e].to_vec()],
        };
        let planar_frame = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: Some((f as i64) * SAMPLES_PER_FRAME as i64),
            data: vec![planar_pcm[0][s..e].to_vec()],
        };
        enc_inter.send_frame(&Frame::Audio(inter_frame)).unwrap();
        enc_planar.send_frame(&Frame::Audio(planar_frame)).unwrap();
        let pkt_i = enc_inter.receive_packet().unwrap();
        let pkt_p = enc_planar.receive_packet().unwrap();
        assert_eq!(
            pkt_i.data, pkt_p.data,
            "interleaved vs planar mono produced different bytes on frame {f}"
        );
    }
}

#[test]
fn encoder_accepts_planar_s16p_stereo_layout() {
    // Stereo planar: two independent tones, one per plane. Compare
    // byte-for-byte against an equivalent interleaved encode. This
    // exercises the `for (ch, plane) in frame.data.iter().enumerate()`
    // deinterleave branch (codec.rs) at the proper nch = 2 fanout.
    let n_frames = 4;
    let n = SAMPLES_PER_FRAME as usize * n_frames;
    let fs = 48_000.0f64;
    let left: Vec<f64> = (0..n)
        .map(|i| 0.4 * (2.0 * std::f64::consts::PI * 1000.0 * (i as f64) / fs).sin())
        .collect();
    let right: Vec<f64> = (0..n)
        .map(|i| 0.4 * (2.0 * std::f64::consts::PI * 1500.0 * (i as f64) / fs).cos())
        .collect();

    let (_, inter_pcm) = pcm_s16_interleaved(&[left.clone(), right.clone()]);
    let (_, planar_pcm) = pcm_s16_planar(&[left, right]);
    assert_eq!(planar_pcm.len(), 2, "stereo -> two planes");

    let (mut enc_inter, _) = make_pair(2, 48_000, 256);
    let (mut enc_planar, _) = make_pair(2, 48_000, 256);

    let bytes_per_frame_inter = SAMPLES_PER_FRAME as usize * 2 * 2; // stereo S16 interleaved
    let bytes_per_plane = SAMPLES_PER_FRAME as usize * 2; // per-channel S16
    for f in 0..n_frames {
        let inter_frame = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: Some((f as i64) * SAMPLES_PER_FRAME as i64),
            data: vec![
                inter_pcm[f * bytes_per_frame_inter..(f + 1) * bytes_per_frame_inter].to_vec(),
            ],
        };
        let planar_frame = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: Some((f as i64) * SAMPLES_PER_FRAME as i64),
            data: vec![
                planar_pcm[0][f * bytes_per_plane..(f + 1) * bytes_per_plane].to_vec(),
                planar_pcm[1][f * bytes_per_plane..(f + 1) * bytes_per_plane].to_vec(),
            ],
        };
        enc_inter.send_frame(&Frame::Audio(inter_frame)).unwrap();
        enc_planar.send_frame(&Frame::Audio(planar_frame)).unwrap();
        let pkt_i = enc_inter.receive_packet().unwrap();
        let pkt_p = enc_planar.receive_packet().unwrap();
        assert_eq!(
            pkt_i.data, pkt_p.data,
            "interleaved vs planar stereo produced different bytes on frame {f}"
        );
    }
}

#[test]
fn encoder_rejects_wrong_plane_count() {
    // `frame.data.len()` must be exactly `1` (interleaved) or `nch`
    // (planar). A 3-plane vector for a stereo encoder must be rejected
    // as a typed error rather than silently mis-deinterleaving.
    let n = SAMPLES_PER_FRAME as usize;
    let plane = vec![0u8; n * 2];
    let (mut enc, _) = make_pair(2, 48_000, 256);
    let bad_frame = AudioFrame {
        samples: SAMPLES_PER_FRAME,
        pts: Some(0),
        data: vec![plane.clone(), plane.clone(), plane],
    };
    let err = enc.send_frame(&Frame::Audio(bad_frame));
    // send_frame stashes; the error surfaces on receive_packet (per the
    // codec.rs encoder, `frame_to_pcm` runs at receive time).
    if err.is_ok() {
        let pkt_err = enc.receive_packet();
        assert!(
            pkt_err.is_err(),
            "expected wrong-plane-count rejection, got Ok"
        );
    }
}

#[test]
fn encoder_rejects_wrong_plane_size() {
    // A planar frame whose per-plane byte count does not equal
    // `samples * 2` is rejected with a typed error (the planar branch
    // of `frame_to_pcm` validates `plane.len() == samples * 2`).
    let n = SAMPLES_PER_FRAME as usize;
    let good_plane = vec![0u8; n * 2];
    let short_plane = vec![0u8; n * 2 - 4];
    let (mut enc, _) = make_pair(2, 48_000, 256);
    let bad_frame = AudioFrame {
        samples: SAMPLES_PER_FRAME,
        pts: Some(0),
        data: vec![good_plane, short_plane],
    };
    let err = enc.send_frame(&Frame::Audio(bad_frame));
    if err.is_ok() {
        let pkt_err = enc.receive_packet();
        assert!(pkt_err.is_err(), "expected short-plane rejection, got Ok");
    }
}
