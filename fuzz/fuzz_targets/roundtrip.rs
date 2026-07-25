#![no_main]

//! Encode → decode **oracle** target (not just no-panic): every
//! iteration draws an arbitrary PCM signal from the fuzzer, encodes it
//! with this crate's Layer I or Layer II encoder at a fixed
//! high-per-channel-rate config, decodes the produced packets with
//! this crate's decoder, and asserts:
//!
//! 1. every frame encodes and decodes successfully with the exact
//!    §2.4.2.1 sample count (384 Layer I / 1152 Layer II per channel);
//! 2. every packet parses as a header matching the config (layer, ID,
//!    sampling frequency, mode, CRC flag) with the header-implied
//!    §2.4.2.1 byte length;
//! 3. a CRC-emitting Layer I encode verifies as [`CrcStatus::Ok`];
//! 4. all-zero PCM decodes back to all-zero PCM (silence is a fixed
//!    point — established by the crate's roundtrip suite);
//! 5. the decoded PCM matches the input within a per-config RMS
//!    conformance bound (delay-compensated, amplitude ≤ 0.3 full
//!    scale — the shape of the crate's established round-trip
//!    bounds, with calibrated headroom so only a real regression
//!    trips it). The input is synthesized as a fuzzer-controlled
//!    mixture of sinusoids confined to the *transmitted* band: Layer
//!    II's Tables 3-B.2x carry only `sblimit` (27/30) of the 32
//!    subbands, so out-of-band content is discarded by a conformant
//!    codec by design and a raw arbitrary-spectrum RMS oracle would
//!    flag that spec-mandated truncation as a failure;
//! 6. free-format Layer I output (§2.4.2.3 `bitrate_index 0000`) is
//!    probed back via `detect_free_format_frame_length` and must
//!    recover exactly the encoder's §2.4.2.1 slot geometry.
//!
//! Spec basis: ISO/IEC 11172-3 §2.4.2.1 (slot arithmetic), §2.4.1.4
//! (error_check), Annex C (encoder procedures); ISO/IEC 13818-3
//! §2.4.2.3 (LSF ladders / free-format prose §2.4.3.1).

use libfuzzer_sys::fuzz_target;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame};
use oxideav_mp1::{
    detect_free_format_frame_length, encode::EncodeParams, encode::LayerSelect,
    encode::Mp1FrameEncoder, encode::PsyModel, Bitrate, CrcStatus, FrameHeader, Layer, Mode,
};

struct Cfg {
    layer: LayerSelect,
    channels: u16,
    sample_rate: u32,
    kbps: u32,
    /// Delay-compensated RMS bound for a ≤0.3-amplitude in-band
    /// signal (calibrated; see target docs).
    rms_bound: f64,
    /// Center of the analysis+synthesis group-delay search window.
    delay_center: usize,
    /// Top of the transmitted band as a fraction of Fs (0.9 ×
    /// sblimit/64 for Layer II tables; 0.9 × 1/2 for Layer I).
    max_freq_frac: f64,
}

// rms_bound values: calibrated against this generator (see the
// fuzz-round notes in CHANGELOG); never tighter than the crate's
// established integration-test bounds for the same rate class.
const CONFIGS: [Cfg; 7] = [
    Cfg {
        layer: LayerSelect::LayerI,
        channels: 1,
        sample_rate: 48_000,
        kbps: 384,
        rms_bound: 0.05,
        delay_center: 481,
        max_freq_frac: 0.45,
    },
    Cfg {
        layer: LayerSelect::LayerI,
        channels: 2,
        sample_rate: 44_100,
        kbps: 448,
        rms_bound: 0.05,
        delay_center: 481,
        max_freq_frac: 0.45,
    },
    Cfg {
        layer: LayerSelect::LayerI,
        channels: 1,
        sample_rate: 32_000,
        kbps: 224,
        rms_bound: 0.05,
        delay_center: 481,
        max_freq_frac: 0.45,
    },
    Cfg {
        layer: LayerSelect::LayerI,
        channels: 1,
        sample_rate: 24_000,
        kbps: 192,
        rms_bound: 0.05,
        delay_center: 481,
        max_freq_frac: 0.45,
    },
    Cfg {
        layer: LayerSelect::LayerI,
        channels: 2,
        sample_rate: 22_050,
        kbps: 160,
        rms_bound: 0.05,
        delay_center: 481,
        max_freq_frac: 0.45,
    },
    // Layer II: B.2a sblimit = 27 → 0.9 · 27/64; B.2b sblimit = 30 →
    // 0.9 · 30/64.
    Cfg {
        layer: LayerSelect::LayerII,
        channels: 1,
        sample_rate: 48_000,
        kbps: 192,
        rms_bound: 0.15,
        delay_center: 481,
        max_freq_frac: 0.379,
    },
    Cfg {
        layer: LayerSelect::LayerII,
        channels: 2,
        sample_rate: 44_100,
        kbps: 384,
        rms_bound: 0.15,
        delay_center: 481,
        max_freq_frac: 0.421,
    },
];

fn params(cfg: &Cfg) -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new("mp1"));
    p.channels = Some(cfg.channels);
    p.sample_rate = Some(cfg.sample_rate);
    p.bit_rate = Some(cfg.kbps as u64 * 1000);
    p
}

/// Interleaved S16 bytes → per-channel f64 in [-1, 1).
fn to_f64(bytes: &[u8], nch: usize) -> Vec<Vec<f64>> {
    let total = bytes.len() / 2;
    let mut out = vec![Vec::with_capacity(total / nch); nch];
    for i in 0..total {
        let s = i16::from_le_bytes([bytes[2 * i], bytes[2 * i + 1]]);
        out[i % nch].push(f64::from(s) / 32768.0);
    }
    out
}

/// Max-across-channels RMS error after searching a narrow window
/// around the known analysis+synthesis group delay.
fn rms(orig: &[u8], dec: &[u8], nch: usize, delay_center: usize) -> f64 {
    let of = to_f64(orig, nch);
    let df = to_f64(dec, nch);
    let mut worst = 0.0f64;
    for ch in 0..nch {
        let n = of[ch].len().min(df[ch].len());
        let mut best = f64::INFINITY;
        for d in delay_center.saturating_sub(8)..=delay_center + 8 {
            if n <= d + 300 {
                continue;
            }
            let (mut err, mut cnt) = (0.0f64, 0usize);
            for i in 200..(n - d - 50) {
                let diff = of[ch][i] - df[ch][i + d];
                err += diff * diff;
                cnt += 1;
            }
            if cnt > 100 {
                best = best.min((err / cnt as f64).sqrt());
            }
        }
        if best.is_finite() && best > worst {
            worst = best;
        }
    }
    worst
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 3 {
        return;
    }
    let sel = data[0] % 8;
    let with_crc = data[1] & 1 != 0;
    let payload = &data[2..];

    // ---- 6. Free-format Layer I geometry oracle. ----
    if sel == 7 {
        let kbps = 40u16 + u16::from(data[1]) % 200;
        let params = EncodeParams {
            bitrate: Bitrate::Fixed(320), // ignored when free_format_kbps is Some
            free_format_kbps: Some(kbps),
            sampling_frequency: 44_100,
            mode: Mode::SingleChannel,
            emit_crc: false,
            layer: LayerSelect::LayerI,
            psychoacoustic: false,
            psy_model: PsyModel::Model2,
            vbr_target_mnr_db: None,
        };
        let mut enc = Mp1FrameEncoder::new(params);
        let mut pcm = [0.0f64; 384];
        for (i, s) in pcm.iter_mut().enumerate() {
            let b = payload[i % payload.len()];
            *s = (f64::from(b) - 128.0) / 128.0 * 0.3;
        }
        let f0 = enc.encode_frame(&pcm).expect("free-format encode frame 0");
        let f1 = enc.encode_frame(&pcm).expect("free-format encode frame 1");
        assert_eq!(
            f0.len(),
            f1.len(),
            "free-format frame length must be constant"
        );
        let h = FrameHeader::parse(&f0).expect("free-format header must parse");
        assert!(matches!(h.bitrate, Bitrate::Free));
        assert!(matches!(h.layer, Layer::I));
        let n = u32::from(kbps) * 12_000 / 44_100; // §2.4.2.1 N = floor(12·bitrate/Fs)
        assert_eq!(f0.len(), (n as usize) * 4, "free-format slot geometry");
        let mut stream = f0.clone();
        stream.extend_from_slice(&f1);
        let probe = detect_free_format_frame_length(&h, &stream[4..])
            .expect("free-format probe over own output");
        assert_eq!(
            probe.base_slot_count, n,
            "probe must recover the encoder's N"
        );
        assert_eq!(probe.frame_length_bytes as usize, f0.len());
        return;
    }

    // ---- Fixed-ladder configs: full PCM oracle. ----
    let cfg = &CONFIGS[sel as usize];
    let nch = cfg.channels as usize;
    let spf: usize = match cfg.layer {
        LayerSelect::LayerI => 384,
        LayerSelect::LayerII => 1152,
    };
    let n_frames: usize = match cfg.layer {
        LayerSelect::LayerI => 4,
        LayerSelect::LayerII => 2,
    };

    // Build ≤0.3-amplitude interleaved S16 PCM as a fuzzer-controlled
    // mixture of up to 6 in-band sinusoids per channel (frequency,
    // amplitude weight, and phase all drawn from the payload). The
    // spectrum stays inside the transmitted band (see target docs) so
    // the RMS conformance oracle is sound.
    let n_partials = ((payload[0] as usize) % 6) + 1;
    let byte = |i: usize| payload[(1 + i) % payload.len()];
    let mut partials: Vec<Vec<(f64, f64, f64)>> = Vec::with_capacity(nch); // (freq_frac, amp, phase)
    let mut all_zero = true;
    for ch in 0..nch {
        let mut v = Vec::with_capacity(n_partials);
        let mut weight_sum = 0.0f64;
        for k in 0..n_partials {
            let base = (ch * n_partials + k) * 3;
            let fbyte = f64::from(byte(base));
            let abyte = f64::from(byte(base + 1));
            let pbyte = f64::from(byte(base + 2));
            // Frequency fraction of Fs in [0.001, max_freq_frac].
            let f = 0.001 + (fbyte / 255.0) * (cfg.max_freq_frac - 0.001);
            let w = abyte / 255.0;
            weight_sum += w;
            v.push((f, w, pbyte / 255.0 * core::f64::consts::TAU));
        }
        // Normalise so the per-channel peak stays ≤ 0.3 full scale.
        if weight_sum > 0.0 {
            for p in v.iter_mut() {
                p.1 = p.1 / weight_sum * 0.3;
            }
            all_zero = false;
        }
        partials.push(v);
    }
    let total = spf * n_frames * nch;
    let mut pcm = Vec::with_capacity(total * 2);
    for i in 0..spf * n_frames {
        for p in partials.iter() {
            let mut s = 0.0f64;
            for &(f, a, ph) in p {
                s += a * (core::f64::consts::TAU * f * i as f64 + ph).sin();
            }
            let v = (s * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
            pcm.extend_from_slice(&v.to_le_bytes());
        }
    }

    let p = params(cfg);
    let mut enc = match cfg.layer {
        LayerSelect::LayerI if with_crc => {
            oxideav_mp1::encoder::make_encoder_with_crc(&p).expect("encoder factory")
        }
        LayerSelect::LayerI => oxideav_mp1::encoder::make_encoder(&p).expect("encoder factory"),
        LayerSelect::LayerII => {
            oxideav_mp1::encoder::make_encoder_layer2(&p).expect("encoder factory")
        }
    };
    let mut dec = oxideav_mp1::decoder::make_decoder(&p).expect("decoder factory");

    let bytes_per_frame = spf * nch * 2;
    let mut decoded = Vec::with_capacity(pcm.len());
    for f in 0..n_frames {
        let af = AudioFrame {
            samples: spf as u32,
            pts: Some((f * spf) as i64),
            data: vec![pcm[f * bytes_per_frame..(f + 1) * bytes_per_frame].to_vec()],
        };
        enc.send_frame(&Frame::Audio(af)).expect("send_frame");
        let pkt = enc.receive_packet().expect("receive_packet");

        // ---- 2. Structural packet oracle. ----
        let h = FrameHeader::parse(&pkt.data).expect("packet must start with a valid header");
        assert_eq!(
            matches!(h.layer, Layer::II),
            matches!(cfg.layer, LayerSelect::LayerII)
        );
        assert_eq!(h.sampling_frequency, cfg.sample_rate);
        assert_eq!(u16::from(h.channels()), cfg.channels);
        assert_eq!(h.bitrate, Bitrate::Fixed(cfg.kbps as u16));
        assert_eq!(
            pkt.data.len(),
            h.frame_length_bytes().expect("fixed-rate length") as usize,
            "packet length disagrees with §2.4.2.1 header-implied length"
        );

        // ---- 3. Emitted CRC must verify. ----
        if matches!(cfg.layer, LayerSelect::LayerI) {
            assert_eq!(
                h.has_crc(),
                with_crc,
                "protection_bit disagrees with factory"
            );
            if with_crc {
                let st = h.verify_crc(&pkt.data[..4], &pkt.data[4..]);
                assert!(
                    matches!(st, Some(CrcStatus::Ok(_))),
                    "encoder-emitted CRC did not verify: {st:?}"
                );
            }
        }

        // ---- 1. Decode success + exact sample count. ----
        dec.send_packet(&pkt).expect("send_packet");
        let Frame::Audio(a) = dec.receive_frame().expect("receive_frame") else {
            panic!("expected an audio frame");
        };
        assert_eq!(a.samples as usize, spf, "decoded sample count");
        for d in &a.data {
            decoded.extend_from_slice(d);
        }
    }
    assert_eq!(decoded.len(), pcm.len(), "decoded byte length");

    // ---- 4. Silence fixed point. ----
    if all_zero {
        assert!(
            decoded.iter().all(|&b| b == 0),
            "all-zero PCM must decode back to digital silence"
        );
        return;
    }

    // ---- 5. Delay-compensated RMS conformance bound. ----
    let e = rms(&pcm, &decoded, nch, cfg.delay_center);
    assert!(
        e < cfg.rms_bound,
        "roundtrip RMS {e} exceeds conformance bound {} (config {sel})",
        cfg.rms_bound
    );
});
