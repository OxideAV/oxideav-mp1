//! MPEG-1 Audio Layer I encoder — CBR or VBR.
//!
//! # Scope
//! - MPEG-1 Layer I, 32 / 44.1 / 48 kHz, mono, plain stereo, or
//!   joint-stereo. Optional CRC-16 protection via the `crc_check` option.
//! - **CBR mode** (default): one fixed bitrate per encoder instance,
//!   from the standard Layer I ladder (32..=448 kbps). Greedy
//!   energy-per-bit allocation: subbands are iteratively awarded
//!   quantiser upgrades in decreasing order of "signal energy per
//!   extra-bit cost" until no more bits are available. Cost accounting
//!   is exact so the frame always fits.
//! - **VBR mode** (`vbr_quality` / `vbr_target_kbps` options):
//!   per-frame masking-driven allocation — see [`crate::psy`] — with
//!   a target-fill phase that brings the long-term average to the
//!   user-supplied target, and per-frame `bitrate_index` floating
//!   over the standard ladder. The masking model is the lightweight
//!   per-subband variant defined in [`crate::psy`], not a full ISO
//!   §C.1 psymodel-1 implementation.
//! - Scalefactors are extracted per-subband-per-channel from the peak of
//!   the 12 subband samples (one SF per subband per frame; Layer I has no
//!   SCFSI, unlike Layer II).
//!
//! Pipeline (mirror of the decoder):
//!   PCM → polyphase analysis → per-subband scalefactor extraction →
//!   bit allocation → sample quantisation → bit packing.
//!
//! # Joint stereo (`joint_stereo` option)
//! - When enabled on a stereo input the encoder sets `mode = 01`
//!   (joint_stereo) and a `mode_extension` selecting the `bound`
//!   (4 / 8 / 12 / 16 — Table in §2.4.2.3). Subbands `[0..bound)`
//!   stay per-channel; subbands `[bound..32)` carry one shared
//!   allocation and one shared quantised sample stream (§2.4.1.5
//!   `audio_data()` for Layer I). The shared stream is the per-sample
//!   mid signal `M = (L + R) / 2`; both channels still carry their own
//!   6-bit scalefactor, so the decoder reconstructs `L = R = M` above
//!   the bound. Layer I has no intensity scaling (§2.4.2.3), so this
//!   M/S-style sharing is the full extent of Layer I joint stereo.
//!   This saves the upper-subband allocation + sample bits, which the
//!   allocator redistributes to the loudest below-bound bands.
//!
//! # Dual-channel (`dual_channel` option)
//! - For two-channel input, when `dual_channel` is set (and joint
//!   stereo is not) the encoder sets `mode = 10` (dual_channel) per
//!   §2.4.2.3. The wire layout is identical to plain stereo: each
//!   subband carries its own per-channel allocation, scalefactor, and
//!   12 quantised samples — only the 2-bit `mode` field changes
//!   (`00` → `10`). The semantic difference is downstream: the two
//!   channels represent independent programs (e.g. two languages)
//!   rather than a stereo pair. The bound is `SBLIMIT = 32` (no
//!   sharing), so the M/S mid collapse in [`Self::encode_one_frame`]
//!   is a no-op.
//!
//! # What is NOT implemented
//! - No full ISO §C.1 psychoacoustic model (no FFT-based tonality
//!   detector, no Bark spreading function, no full ATH table).
//! - No intensity-stereo scaling (Layer I has none — §2.4.2.3).
//! - No free-format output.
//!
//! CRC-16 protection (§2.4.3.1) is emitted on request via the
//! `crc_check` option; see [`crate::crc`].

use std::collections::VecDeque;

use oxideav_core::Encoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, MediaType, Packet, Result, SampleFormat,
    TimeBase,
};

use crate::analysis::{analyze_frame, AnalysisState, SAMPLES_PER_FRAME};
use crate::bitalloc::{bits_per_sample, scale_table, SBLIMIT};
use crate::crc::Crc16;
use crate::header::ChannelMode;
use crate::psy::{subband_noise_energy, vbr_quality_to_mask_ratio, SubbandMask};
use crate::CODEC_ID_STR;
use oxideav_core::bits::BitWriter;
use oxideav_core::options::{
    parse_options, CodecOptionsStruct, OptionField, OptionKind, OptionValue,
};

/// Number of subband-sample blocks per Layer I frame (12 × 32 = 384 PCM).
const BLOCKS_PER_FRAME: usize = 12;

/// MPEG-1 Layer I bitrate ladder, in kbps. Index 0 = "free format"
/// (encoder doesn't emit), index 15 = forbidden. Per ISO/IEC 11172-3
/// Table 2.4.2.3 (Layer I row).
const LAYER1_BITRATES_KBPS: [u32; 15] = [
    0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448,
];

/// Encoder rate-control mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RateControl {
    /// Constant bit rate — fixed slot from `bitrate_kbps` for every frame.
    Cbr,
    /// Variable bit rate — per-frame slot picked from the standard
    /// ladder driven by a per-subband masking model and bounded by a
    /// rolling target average bitrate.
    Vbr,
}

/// Typed options consumed by the MP1 encoder. Wired into
/// [`CodecParameters::options`] via [`oxideav_core::options`].
///
/// Recognised keys:
///
/// - `vbr_quality` — `u32` 0..=9. When set, switches the encoder to
///   VBR. Lower = stricter masking target = larger files; higher =
///   looser target = smaller files.
/// - `vbr_target_kbps` — `u32` average target bitrate in kbps. When
///   set, also switches the encoder to VBR. The per-frame allocator is
///   capped by a rolling-average controller so the file size honours
///   the target ± expected variance. Defaults to the `bit_rate` field
///   in CBR mode, or 192 kbps when neither is provided.
/// - `cbr_bitrate_kbps` — `u32` CBR slot in kbps. Overrides
///   `bit_rate` in CBR mode; ignored when VBR is active.
/// - `crc_check` — `bool`. When `true`, the encoder clears the header
///   `protection_bit` and inserts a CRC-16 protection word after the
///   header (ISO/IEC 11172-3 §2.4.3.1). Defaults to `false` (no CRC),
///   matching the historical encoder behaviour and most real-world
///   Layer I streams.
/// - `joint_stereo` — `bool`. Only meaningful for 2-channel input.
///   When `true`, the encoder emits `mode = 01` (joint_stereo) and
///   shares the upper-subband allocation + sample stream between the
///   two channels (M/S-style mid sharing above the `bound`).
/// - `js_bound` — `u32` in `{4, 8, 12, 16}`. Selects the joint-stereo
///   `bound`: subbands `[bound..32)` are shared. Maps to the
///   `mode_extension` field (4→0, 8→1, 12→2, 16→3). Only consulted
///   when `joint_stereo` is set. Defaults to 8 (`mode_extension = 1`).
/// - `dual_channel` — `bool`. Only meaningful for 2-channel input and
///   only when `joint_stereo` is *not* set. When `true`, the encoder
///   emits `mode = 10` (dual_channel) per §2.4.2.3. The wire layout is
///   identical to plain stereo (same per-channel allocations,
///   scalefactors, and samples for all 32 subbands); only the 2-bit
///   `mode` field changes from `00` to `10`. The semantic difference
///   is downstream — the channels represent two independent programs
///   rather than a stereo pair.
#[derive(Debug, Clone)]
pub struct Mp1EncoderOptions {
    /// `Some(0..=9)` → switch to VBR with the given quality index.
    pub vbr_quality: Option<u8>,
    /// `Some(kbps)` → switch to VBR targeting that average bitrate.
    pub vbr_target_kbps: Option<u32>,
    /// Override for the CBR slot (kbps). Only consulted in CBR mode.
    pub cbr_bitrate_kbps: Option<u32>,
    /// Emit a CRC-16 protection word (clears the `protection_bit`).
    pub crc_check: bool,
    /// Emit joint-stereo (`mode = 01`) for 2-channel input.
    pub joint_stereo: bool,
    /// Joint-stereo bound (4 / 8 / 12 / 16). Only when `joint_stereo`.
    pub js_bound: u8,
    /// Emit dual-channel (`mode = 10`) for 2-channel input. Ignored
    /// when `joint_stereo` is set (joint_stereo wins).
    pub dual_channel: bool,
}

impl Default for Mp1EncoderOptions {
    fn default() -> Self {
        Self {
            vbr_quality: None,
            vbr_target_kbps: None,
            cbr_bitrate_kbps: None,
            crc_check: false,
            joint_stereo: false,
            js_bound: 8,
            dual_channel: false,
        }
    }
}

impl CodecOptionsStruct for Mp1EncoderOptions {
    const SCHEMA: &'static [OptionField] = &[
        OptionField {
            name: "vbr_quality",
            kind: OptionKind::U32,
            default: OptionValue::U32(u32::MAX),
            help: "VBR quality 0..=9 (0=best, 9=smallest). Switches to VBR mode.",
        },
        OptionField {
            name: "vbr_target_kbps",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "VBR target average bitrate in kbps. Switches to VBR mode.",
        },
        OptionField {
            name: "cbr_bitrate_kbps",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "Override CBR bitrate in kbps. Ignored when VBR is active.",
        },
        OptionField {
            name: "crc_check",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Emit a CRC-16 protection word (clears protection_bit).",
        },
        OptionField {
            name: "joint_stereo",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Emit joint-stereo (mode=01) for 2-channel input.",
        },
        OptionField {
            name: "js_bound",
            kind: OptionKind::U32,
            default: OptionValue::U32(8),
            help: "Joint-stereo bound: 4, 8, 12, or 16 (default 8).",
        },
        OptionField {
            name: "dual_channel",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "Emit dual-channel (mode=10) for 2-channel input. \
                   Ignored if joint_stereo is also set.",
        },
    ];

    fn apply(&mut self, key: &str, v: &OptionValue) -> Result<()> {
        match key {
            "vbr_quality" => {
                let n = v.as_u32()?;
                if n == u32::MAX {
                    return Ok(()); // "unset" sentinel
                }
                if n > 9 {
                    return Err(Error::invalid(format!(
                        "MP1 encoder: vbr_quality must be 0..=9, got {n}"
                    )));
                }
                self.vbr_quality = Some(n as u8);
            }
            "vbr_target_kbps" => {
                let n = v.as_u32()?;
                if n > 0 {
                    self.vbr_target_kbps = Some(n);
                }
            }
            "cbr_bitrate_kbps" => {
                let n = v.as_u32()?;
                if n > 0 {
                    self.cbr_bitrate_kbps = Some(n);
                }
            }
            "crc_check" => {
                self.crc_check = v.as_bool()?;
            }
            "joint_stereo" => {
                self.joint_stereo = v.as_bool()?;
            }
            "js_bound" => {
                let n = v.as_u32()?;
                match n {
                    4 | 8 | 12 | 16 => self.js_bound = n as u8,
                    _ => {
                        return Err(Error::invalid(format!(
                            "MP1 encoder: js_bound must be 4/8/12/16, got {n}"
                        )));
                    }
                }
            }
            "dual_channel" => {
                self.dual_channel = v.as_bool()?;
            }
            _ => unreachable!("guarded by SCHEMA"),
        }
        Ok(())
    }
}

/// Build a Layer I CBR encoder for the requested parameters.
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("MP1 encoder: missing channels"))?;
    if !(1..=2).contains(&channels) {
        return Err(Error::invalid("MP1 encoder: channels must be 1 or 2"));
    }
    let sample_rate = params
        .sample_rate
        .ok_or_else(|| Error::invalid("MP1 encoder: missing sample_rate"))?;
    match sample_rate {
        32_000 | 44_100 | 48_000 => {}
        _ => {
            return Err(Error::unsupported(format!(
                "MP1 encoder: unsupported sample rate {sample_rate} (need 32000/44100/48000)"
            )));
        }
    }

    // Parse encoder options. `vbr_quality` or `vbr_target_kbps` switch
    // the encoder to VBR; otherwise we stay in CBR.
    let opts: Mp1EncoderOptions = parse_options(&params.options)?;
    let rate_control = if opts.vbr_quality.is_some() || opts.vbr_target_kbps.is_some() {
        RateControl::Vbr
    } else {
        RateControl::Cbr
    };

    // CBR slot / VBR anchor: explicit option override > params.bit_rate
    // > 192 kbps. In VBR mode this fixes the *target average* bitrate
    // (and the initial bitrate-index field of frame 0).
    let bitrate_kbps = opts
        .cbr_bitrate_kbps
        .or(opts.vbr_target_kbps)
        .or_else(|| params.bit_rate.map(|b| (b / 1000) as u32))
        .unwrap_or(192);
    let br_index = bitrate_to_index(bitrate_kbps).ok_or_else(|| {
        Error::unsupported(format!(
            "MP1 encoder: unsupported bitrate {bitrate_kbps} kbps"
        ))
    })?;
    let vbr_quality = opts.vbr_quality.unwrap_or(3);
    let vbr_target_kbps = opts.vbr_target_kbps.unwrap_or(bitrate_kbps);
    let crc_check = opts.crc_check;

    // Joint stereo is only meaningful for two channels. Silently fall
    // back to plain stereo / mono when the option is set on a 1-channel
    // stream (the mode field would otherwise be inconsistent with the
    // channel count). `js_bound` maps to the 2-bit `mode_extension`.
    //
    // Dual-channel is also only meaningful for two channels; joint
    // stereo wins if both flags are set (since joint stereo materially
    // changes the bit allocation, while dual-channel only relabels the
    // 2-bit mode field). See §2.4.2.3.
    let joint_stereo = opts.joint_stereo && channels == 2;
    let dual_channel = !joint_stereo && opts.dual_channel && channels == 2;
    let (channel_mode, mode_extension, js_bound) = if joint_stereo {
        let me = match opts.js_bound {
            4 => 0u8,
            8 => 1,
            12 => 2,
            16 => 3,
            _ => 1, // unreachable: validated in apply()
        };
        (ChannelMode::JointStereo, me, opts.js_bound as usize)
    } else if dual_channel {
        // Wire layout identical to plain stereo (bound = SBLIMIT, no
        // sharing); the mode-field branch below emits 0b10. §2.4.2.3.
        (ChannelMode::DualChannel, 0, SBLIMIT)
    } else if channels == 2 {
        (ChannelMode::Stereo, 0, SBLIMIT)
    } else {
        (ChannelMode::SingleChannel, 0, SBLIMIT)
    };

    let sample_format = params.sample_format.unwrap_or(SampleFormat::S16);
    if sample_format != SampleFormat::S16 {
        return Err(Error::unsupported(format!(
            "MP1 encoder: input sample format {sample_format:?} not supported (need S16)"
        )));
    }

    let sr_index = match sample_rate {
        44_100 => 0u8,
        48_000 => 1,
        32_000 => 2,
        _ => unreachable!(),
    };

    let mut output = params.clone();
    output.media_type = MediaType::Audio;
    output.codec_id = CodecId::new(CODEC_ID_STR);
    output.sample_format = Some(sample_format);
    output.channels = Some(channels);
    output.sample_rate = Some(sample_rate);
    output.bit_rate = Some((bitrate_kbps as u64) * 1000);

    Ok(Box::new(Mp1Encoder {
        output_params: output,
        channels,
        sample_rate,
        bitrate_kbps,
        sr_index,
        br_index,
        rate_control,
        vbr_quality,
        vbr_target_kbps,
        crc_check,
        channel_mode,
        mode_extension,
        js_bound,
        time_base: TimeBase::new(1, sample_rate as i64),
        analysis_state: [AnalysisState::new(), AnalysisState::new()],
        pcm_queue: vec![Vec::new(); channels as usize],
        pending_packets: VecDeque::new(),
        frame_index: 0,
        eof: false,
        cumulative_padded_bits: 0,
        vbr_target_bits_acc: 0,
        vbr_used_bits_acc: 0,
    }))
}

struct Mp1Encoder {
    output_params: CodecParameters,
    channels: u16,
    sample_rate: u32,
    /// CBR slot / VBR anchor (target average) in kbps.
    bitrate_kbps: u32,
    sr_index: u8,
    br_index: u32,
    rate_control: RateControl,
    /// VBR mask-strictness knob (0..=9). Only consulted when
    /// `rate_control == Vbr`.
    vbr_quality: u8,
    /// VBR target average bitrate in kbps. Only consulted when
    /// `rate_control == Vbr`.
    vbr_target_kbps: u32,
    /// When true, clear the `protection_bit` and emit a CRC-16 word
    /// after the header (ISO/IEC 11172-3 §2.4.3.1).
    crc_check: bool,
    /// Header `mode` field: Stereo / JointStereo / SingleChannel.
    channel_mode: ChannelMode,
    /// Header `mode_extension` field (joint-stereo bound selector).
    mode_extension: u8,
    /// Joint-stereo bound: subbands `[js_bound..32)` are channel-shared.
    /// Equals `SBLIMIT` (32) for mono and plain stereo (nothing shared).
    js_bound: usize,
    time_base: TimeBase,
    analysis_state: [AnalysisState; 2],
    pcm_queue: Vec<Vec<f32>>,
    pending_packets: VecDeque<Packet>,
    frame_index: u64,
    eof: bool,
    /// Fractional-byte CBR padding accumulator; see [`next_padding`].
    cumulative_padded_bits: u64,
    /// Cumulative VBR target bits over all emitted frames (for the
    /// rolling-average controller).
    vbr_target_bits_acc: u64,
    /// Cumulative VBR bits actually emitted so far.
    vbr_used_bits_acc: u64,
}

impl Mp1Encoder {
    /// Frame size in bytes.
    ///
    /// Layer I uses 4-byte slots: `slots = 12 * bitrate / sample_rate + pad`,
    /// `bytes = slots * 4` (ISO 11172-3 §2.4.3.1).
    fn frame_bytes(&self, padding: bool) -> usize {
        let slots = 12 * self.bitrate_kbps * 1000 / self.sample_rate;
        let padded_slots = slots + if padding { 1 } else { 0 };
        (padded_slots * 4) as usize
    }

    /// Decide whether this frame should set the padding bit. For fractional
    /// slots per frame we accumulate remainders modulo `sample_rate` and pay
    /// off with one padding slot whenever the accumulator overflows. Layer
    /// I's slot is 32 bits wide, so the threshold is `sample_rate * 32`.
    fn next_padding(&mut self) -> bool {
        let num = 12_000u64 * self.bitrate_kbps as u64;
        let sr = self.sample_rate as u64;
        let rem = num - (num / sr) * sr;
        self.cumulative_padded_bits += rem;
        // One slot = 4 bytes = 32 bits. (`cumulative_padded_bits` tracks
        // fractional slot units scaled by sample_rate.)
        let pad = self.cumulative_padded_bits >= sr;
        if pad {
            self.cumulative_padded_bits -= sr;
        }
        pad
    }

    fn ingest(&mut self, frame: &AudioFrame) -> Result<()> {
        // Stream-level validation (channel count, sample rate, S16
        // sample format) is owned by the factory at construction —
        // see `make_encoder`. The slim AudioFrame doesn't carry them.
        let data = frame
            .data
            .first()
            .ok_or_else(|| Error::invalid("MP1 encoder: empty frame"))?;
        let n_ch = self.channels as usize;
        let n_samples = data.len() / (2 * n_ch);
        for i in 0..n_samples {
            for ch in 0..n_ch {
                let off = (i * n_ch + ch) * 2;
                let s = i16::from_le_bytes([data[off], data[off + 1]]) as f32 / 32768.0;
                self.pcm_queue[ch].push(s);
            }
        }
        self.flush_ready_frames(false)
    }

    fn flush_ready_frames(&mut self, drain: bool) -> Result<()> {
        let n_ch = self.channels as usize;
        loop {
            let avail = self.pcm_queue[0].len();
            if avail < SAMPLES_PER_FRAME {
                if drain && avail > 0 {
                    for ch in 0..n_ch {
                        self.pcm_queue[ch].resize(SAMPLES_PER_FRAME, 0.0);
                    }
                } else {
                    return Ok(());
                }
            }
            let pkt = self.encode_one_frame()?;
            self.pending_packets.push_back(pkt);
            if drain && self.pcm_queue[0].iter().all(|&v| v == 0.0) {
                return Ok(());
            }
        }
    }

    fn encode_one_frame(&mut self) -> Result<Packet> {
        let n_ch = self.channels as usize;

        // Drain 384 samples/channel from the queue.
        let mut pcm_in: Vec<[f32; SAMPLES_PER_FRAME]> = vec![[0.0f32; SAMPLES_PER_FRAME]; n_ch];
        for ch in 0..n_ch {
            for i in 0..SAMPLES_PER_FRAME {
                pcm_in[ch][i] = self.pcm_queue[ch][i];
            }
            self.pcm_queue[ch].drain(..SAMPLES_PER_FRAME);
        }

        // --- 1. Analysis: 32 × 12 subband buffer per channel ---
        let mut sub: Vec<[[f32; 12]; 32]> = (0..n_ch).map(|_| [[0.0f32; 12]; 32]).collect();
        for ch in 0..n_ch {
            analyze_frame(&mut self.analysis_state[ch], &pcm_in[ch], &mut sub[ch]);
        }

        // --- 1a. Joint-stereo mid-down above the bound ---
        // For subbands [bound..32) Layer I joint stereo carries a single
        // shared sample stream (§2.4.1.5 audio_data). We collapse both
        // channels to the per-sample mid `M = (L + R) / 2` so the shared
        // quantised level reconstructs `L = R = M` at the decoder (Layer
        // I has no intensity scaling — §2.4.2.3). Below the bound the
        // channels stay independent. `js_bound == SBLIMIT` for mono /
        // plain stereo, so this loop is a no-op there.
        let bound = self.js_bound.min(SBLIMIT);
        if n_ch == 2 && bound < SBLIMIT {
            for sb in bound..SBLIMIT {
                for i in 0..12 {
                    let m = 0.5 * (sub[0][sb][i] + sub[1][sb][i]);
                    sub[0][sb][i] = m;
                    sub[1][sb][i] = m;
                }
            }
        }

        // --- 2. Scalefactor pick: one 6-bit SF index per subband per
        //     channel. Peak over all 12 samples in the subband. ---
        let mut scf_idx = vec![[0u8; 32]; n_ch];
        for ch in 0..n_ch {
            for sb in 0..SBLIMIT {
                let mut peak = 0.0f32;
                for i in 0..12 {
                    let v = sub[ch][sb][i].abs();
                    if v > peak {
                        peak = v;
                    }
                }
                scf_idx[ch][sb] = pick_scalefactor(peak);
            }
        }

        // --- 3. Subband energies for the allocator ---
        let mut energy = vec![[0.0f32; 32]; n_ch];
        for ch in 0..n_ch {
            for sb in 0..SBLIMIT {
                let mut e = 0.0f32;
                for i in 0..12 {
                    let v = sub[ch][sb][i];
                    e += v * v;
                }
                energy[ch][sb] = e / 12.0;
            }
        }

        // --- 4. Bit allocation + frame slot selection ---
        // Dispatch on rate-control mode. CBR fixes the slot up-front
        // and runs the energy-per-bit greedy allocator against the
        // resulting budget. VBR runs the masking-driven allocator
        // first, then picks the smallest standard slot whose frame can
        // hold the result (subject to a rolling-average controller
        // that nudges the per-frame max bits toward `vbr_target_kbps`).
        let header_bits = 32u32;
        let crc_bits = if self.crc_check { 16u32 } else { 0 };
        // Bit-allocation field: 4 bits per (subband, channel) below the
        // bound, plus 4 bits per shared subband above it (§2.4.1.5).
        let bitalloc_bits = 4u32 * (bound as u32 * n_ch as u32 + (SBLIMIT - bound) as u32);
        let fixed_overhead_bits = header_bits + crc_bits + bitalloc_bits;

        let (alloc, frame_bytes, padding, frame_br_index) = match self.rate_control {
            RateControl::Cbr => {
                self.allocate_cbr(n_ch, bound, &energy, &scf_idx, fixed_overhead_bits)?
            }
            RateControl::Vbr => {
                self.allocate_vbr(n_ch, bound, &sub, &energy, &scf_idx, fixed_overhead_bits)?
            }
        };

        // --- 5. Write frame ---
        let mut w = BitWriter::with_capacity(frame_bytes);

        // Header (32 bits).
        // syncword 0xFFF, ID=1 (MPEG-1), layer=11 (Layer I),
        // protection_bit (1 = no CRC, 0 = CRC present), bitrate_index,
        // sampling_frequency_index, padding bit, private bit=0, mode
        // (00=stereo, 11=mono), mode_extension=0, copyright=0,
        // original=0, emphasis=0.
        w.write_u32(0xFFF, 12);
        w.write_u32(1, 1); // ID = MPEG-1
        w.write_u32(0b11, 2); // Layer I
                              // protection_bit: 0 when a CRC word follows the header.
        w.write_u32(if self.crc_check { 0 } else { 1 }, 1);
        w.write_u32(frame_br_index, 4);
        w.write_u32(self.sr_index as u32, 2);
        w.write_u32(if padding { 1 } else { 0 }, 1);
        w.write_u32(0, 1); // private
                           // mode (§2.4.2.3): 00=stereo, 01=joint_stereo,
                           // 10=dual_channel, 11=single_channel.
        let mode_bits = match self.channel_mode {
            ChannelMode::SingleChannel => 0b11u32,
            ChannelMode::JointStereo => 0b01,
            ChannelMode::DualChannel => 0b10,
            ChannelMode::Stereo => 0b00,
        };
        w.write_u32(mode_bits, 2);
        // mode_extension carries the joint-stereo bound selector; 0 in
        // every other mode (§2.4.2.3).
        w.write_u32(self.mode_extension as u32, 2);
        w.write_u32(0, 1); // copyright
        w.write_u32(0, 1); // original
        w.write_u32(0, 2); // emphasis

        // The CRC-16 protection word (ISO/IEC 11172-3 §2.4.3.1) sits
        // immediately after the header when `protection_bit == 0`. The
        // header is exactly 32 bits and byte-aligned, so its last two
        // bytes (the protected header tail) are now bytes 2..4 of the
        // writer buffer. Seed the CRC with them, emit a 16-bit zero
        // placeholder (keeping the writer byte-aligned), and patch the
        // real value in once the bit-allocation field is folded in.
        let mut crc = Crc16::new();
        let crc_placeholder_at = if self.crc_check {
            let hdr = w.bytes();
            crc.push_byte(hdr[2]);
            crc.push_byte(hdr[3]);
            let at = hdr.len();
            w.write_u32(0, 16); // CRC placeholder, patched after allocation
            Some(at)
        } else {
            None
        };

        // --- 5a. Bit allocation (§2.4.1.5 audio_data). Channel-paired
        //     for subbands [0..bound), then one shared element for each
        //     subband in [bound..32). For joint stereo the allocator has
        //     already mirrored alloc[0][sb] into alloc[1][sb] above the
        //     bound, so we emit alloc[0][sb] there. Each 4-bit element is
        //     folded into the CRC in wire order.
        for sb in 0..bound {
            for ch in 0..n_ch {
                let a = alloc[ch][sb] as u32;
                if self.crc_check {
                    crc.push_bits(a, 4);
                }
                w.write_u32(a, 4);
            }
        }
        for sb in bound..SBLIMIT {
            let a = alloc[0][sb] as u32;
            if self.crc_check {
                crc.push_bits(a, 4);
            }
            w.write_u32(a, 4);
        }

        // --- 5b. Scalefactors (6 bits per subband-per-channel with
        //     allocation != 0). Layer I has no SCFSI. Order: by subband,
        //     then by channel — and §2.4.1.5 keeps scalefactors PER
        //     channel even for shared (above-bound) subbands. Above the
        //     bound both channels carry the same mid signal, so their
        //     scalefactors are equal, but both are still written.
        for sb in 0..SBLIMIT {
            for ch in 0..n_ch {
                let a = if sb < bound {
                    alloc[ch][sb]
                } else {
                    alloc[0][sb]
                };
                if a != 0 {
                    w.write_u32(scf_idx[ch][sb] as u32, 6);
                }
            }
        }

        // --- 5c. Sample payload. 12 blocks. For each block: one sample
        //     per (subband, channel) below the bound, then one shared
        //     sample per subband above it (§2.4.1.5). The shared sample
        //     is the mid signal stored in channel 0; the decoder applies
        //     each channel's own scalefactor to it.
        let sc = scale_table();
        for block in 0..BLOCKS_PER_FRAME {
            for sb in 0..bound {
                for ch in 0..n_ch {
                    let a = alloc[ch][sb];
                    if a == 0 {
                        continue;
                    }
                    let nb = bits_per_sample(a).unwrap();
                    let sf_mag = sc[scf_idx[ch][sb] as usize];
                    let raw = quantise_sample(sub[ch][sb][block], sf_mag, nb);
                    w.write_u32(raw, nb as u32);
                }
            }
            for sb in bound..SBLIMIT {
                let a = alloc[0][sb];
                if a == 0 {
                    continue;
                }
                let nb = bits_per_sample(a).unwrap();
                // Quantise the shared mid sample against channel 0's
                // scalefactor; the decoder rescales per channel.
                let sf_mag = sc[scf_idx[0][sb] as usize];
                let raw = quantise_sample(sub[0][sb][block], sf_mag, nb);
                w.write_u32(raw, nb as u32);
            }
        }

        // --- 5d. Pad to frame length. Fill with zero ancillary data.
        w.align_to_byte();
        let mut bytes = w.into_bytes();

        // Patch the CRC placeholder now that the protected field is
        // complete. `crc.value()` covers the header tail + every
        // allocation bit folded in above.
        if let Some(at) = crc_placeholder_at {
            let word = crc.value().to_be_bytes();
            bytes[at] = word[0];
            bytes[at + 1] = word[1];
        }

        if bytes.len() > frame_bytes {
            bytes.truncate(frame_bytes);
        }
        if bytes.len() < frame_bytes {
            bytes.resize(frame_bytes, 0);
        }

        // Update VBR running totals so the next frame's controller can
        // see how much credit/debit we've built up against the target.
        if matches!(self.rate_control, RateControl::Vbr) {
            self.vbr_used_bits_acc += (bytes.len() as u64) * 8;
            // target_bits_per_frame = bitrate * 384 / sample_rate; multiply
            // numerator by 1000 to keep integer units in kbps.
            let tgt = (self.vbr_target_kbps as u64) * 1000 * SAMPLES_PER_FRAME as u64
                / self.sample_rate as u64;
            self.vbr_target_bits_acc += tgt;
        }

        let pts = (self.frame_index as i64) * SAMPLES_PER_FRAME as i64;
        let mut pkt = Packet::new(0, self.time_base, bytes);
        pkt.pts = Some(pts);
        pkt.dts = Some(pts);
        pkt.duration = Some(SAMPLES_PER_FRAME as i64);
        pkt.flags.keyframe = true;
        self.frame_index += 1;
        Ok(pkt)
    }

    /// CBR allocator: fixes the slot from `bitrate_kbps`, runs the
    /// energy-per-bit greedy allocator against the resulting budget,
    /// and returns the allocation matrix plus frame-size metadata.
    fn allocate_cbr(
        &mut self,
        n_ch: usize,
        bound: usize,
        energy: &[[f32; 32]],
        scf_idx: &[[u8; 32]],
        fixed_overhead_bits: u32,
    ) -> Result<(Vec<[u8; 32]>, usize, bool, u32)> {
        let padding = self.next_padding();
        let frame_bytes = self.frame_bytes(padding);
        let frame_bits = frame_bytes * 8;

        let sample_and_scf_budget: i64 = frame_bits as i64 - fixed_overhead_bits as i64;
        if sample_and_scf_budget < 0 {
            return Err(Error::other("MP1 encoder: frame too small for header"));
        }

        let alloc = greedy_energy_allocator(n_ch, bound, energy, scf_idx, sample_and_scf_budget);
        Ok((alloc, frame_bytes, padding, self.br_index))
    }

    /// VBR allocator: drives subband upgrades from a per-frame masking
    /// model, then spends any leftover headroom (up to a rolling
    /// per-frame budget tied to `vbr_target_kbps`) on the loudest
    /// remaining bands. Picks the smallest standard Layer I bitrate
    /// slot whose frame can hold the result.
    ///
    /// Two-phase design:
    ///
    /// 1. **Masking phase.** While any (ch, sb) pair has quantisation
    ///    noise above its mask threshold, grant the upgrade with the
    ///    best NMR-drop / bit ratio. This phase responds to per-frame
    ///    masking: a transient with energy in many bands gets more
    ///    bits than a steady tone in one band.
    /// 2. **Target-fill phase.** If the masking phase finishes with
    ///    bits to spare under the rolling target budget, keep
    ///    granting upgrades using the same energy-per-bit heuristic
    ///    as CBR until the target is exhausted. This is what brings
    ///    the *average* bitrate to the requested level.
    ///
    /// The rolling controller (`vbr_target_bits_acc - vbr_used_bits_acc`)
    /// nudges the per-frame ceiling up or down so cumulative output
    /// converges on `vbr_target_kbps` even when individual frames over-
    /// or undershoot.
    fn allocate_vbr(
        &mut self,
        n_ch: usize,
        bound: usize,
        sub: &[[[f32; 12]; 32]],
        energy: &[[f32; 32]],
        scf_idx: &[[u8; 32]],
        fixed_overhead_bits: u32,
    ) -> Result<(Vec<[u8; 32]>, usize, bool, u32)> {
        // Per-channel masking estimate.
        let mask_ratio = vbr_quality_to_mask_ratio(self.vbr_quality);
        let mask: Vec<SubbandMask> = (0..n_ch)
            .map(|ch| SubbandMask::analyze(&sub[ch], self.sample_rate, mask_ratio))
            .collect();

        // Rolling-average controller: bias the per-frame max-bits
        // budget by the running surplus (target_bits - used_bits),
        // clipped to ±50% of the per-frame target.
        let frame_target_bits = (self.vbr_target_kbps as i64) * 1000 * SAMPLES_PER_FRAME as i64
            / self.sample_rate as i64;
        let surplus: i64 = self.vbr_target_bits_acc as i64 - self.vbr_used_bits_acc as i64;
        let cap_correction = surplus.clamp(-frame_target_bits / 2, frame_target_bits / 2);
        let max_frame_bits = (frame_target_bits + cap_correction).max(64);
        // Hard ceiling so a single transient frame never blows past
        // 448 kbps (the top of the Layer I ladder).
        let hard_ceiling_bits =
            (448i64 * 1000 * SAMPLES_PER_FRAME as i64 / self.sample_rate as i64).max(64);
        let max_frame_bits = max_frame_bits.min(hard_ceiling_bits);

        let sc = scale_table();
        let mut alloc: Vec<[u8; 32]> = vec![[0u8; 32]; n_ch];
        let mut bits_used: i64 = fixed_overhead_bits as i64;

        // ---- Phase 1: masking-driven upgrades. ----
        loop {
            let mut best: Option<(usize, usize, u8, u32)> = None;
            let mut best_score = f32::NEG_INFINITY;

            for ch in 0..n_ch {
                for sb in 0..SBLIMIT {
                    // Above the bound the two channels share one stream;
                    // only channel 0 is a candidate (its upgrade is
                    // mirrored to channel 1 below). See `upgrade_cost`.
                    if sb >= bound && ch != 0 {
                        continue;
                    }
                    let cur = alloc[ch][sb];
                    if cur >= 14 {
                        continue;
                    }
                    if energy[ch][sb] <= 0.0 {
                        continue;
                    }
                    // Already masked at current allocation? Skip.
                    let sf_mag = sc[scf_idx[ch][sb] as usize];
                    let cur_noise = subband_noise_energy(cur, sf_mag, energy[ch][sb]);
                    if cur_noise <= mask[ch].threshold[sb] {
                        continue;
                    }
                    let next = cur + 1;
                    let cost = upgrade_cost_bits_bound(cur, next, sb, bound, n_ch);
                    if (bits_used + cost as i64) > max_frame_bits {
                        continue;
                    }
                    // NMR-drop / bit: log(cur_noise / next_noise) / cost.
                    let next_noise = subband_noise_energy(next, sf_mag, energy[ch][sb]);
                    let nmr_drop = (cur_noise / next_noise.max(1.0e-30)).max(1.0);
                    let score = nmr_drop.ln() / (cost as f32).max(1.0);
                    if score > best_score {
                        best_score = score;
                        best = Some((ch, sb, next, cost));
                    }
                }
            }

            match best {
                Some((ch, sb, next, cost)) => {
                    alloc[ch][sb] = next;
                    if sb >= bound {
                        alloc[1.min(n_ch - 1)][sb] = next;
                    }
                    bits_used += cost as i64;
                }
                None => break,
            }
        }

        // ---- Phase 2: target-fill. Spend any leftover budget up to a
        // quality-scaled fraction of the rolling cap. We want the
        // *average* output rate to honour the user's target even when
        // the mask is easily satisfied (e.g. a pure tone), but lower
        // quality knobs should leave more headroom unused (smaller
        // files at the cost of lower per-band SNR). The fraction
        // interpolates linearly across q=0..9: q=0 → 100% of cap,
        // q=9 → 50%. Mask-satisfied frames at low quality therefore
        // stop short of the rolling cap. ----
        let peak_energy = energy
            .iter()
            .flat_map(|row| row.iter().copied())
            .fold(0.0f32, f32::max);
        // 1e-20 of peak ≈ -200 dB power. Effectively "any non-zero
        // band". Pure-zero bands stay excluded so silence frames stop
        // early.
        let noise_floor = peak_energy * 1e-20 + 1.0e-30;
        let q_frac = 1.0_f32 - 0.5 * (self.vbr_quality.min(9) as f32 / 9.0);
        let phase2_cap = (max_frame_bits as f32 * q_frac) as i64;
        let phase2_cap = phase2_cap.max(bits_used).min(max_frame_bits);
        loop {
            let mut best: Option<(usize, usize, u8, u32)> = None;
            let mut best_score = f32::NEG_INFINITY;

            for ch in 0..n_ch {
                for sb in 0..SBLIMIT {
                    // Shared subbands: channel 0 is the sole candidate.
                    if sb >= bound && ch != 0 {
                        continue;
                    }
                    let cur = alloc[ch][sb];
                    if cur >= 14 {
                        continue;
                    }
                    if energy[ch][sb] <= noise_floor {
                        continue;
                    }
                    let next = cur + 1;
                    let cost = upgrade_cost_bits_bound(cur, next, sb, bound, n_ch);
                    if (bits_used + cost as i64) > phase2_cap {
                        continue;
                    }
                    // Slightly biased score: pure-energy ratio is fine
                    // when the budget is the bottleneck, but for very
                    // sparse spectra (a tone on top of zero bands) we
                    // also want to *spread* bits to neighbours so the
                    // output reconstructs cleanly. Add a small floor
                    // so dim bands aren't infinitely deferred.
                    let score = (energy[ch][sb] + peak_energy * 1.0e-6) / (cost as f32).max(1.0);
                    if score > best_score {
                        best_score = score;
                        best = Some((ch, sb, next, cost));
                    }
                }
            }

            match best {
                Some((ch, sb, next, cost)) => {
                    alloc[ch][sb] = next;
                    if sb >= bound {
                        alloc[1.min(n_ch - 1)][sb] = next;
                    }
                    bits_used += cost as i64;
                }
                None => break,
            }
        }

        // Pick the smallest standard slot whose unpadded frame can hold
        // `bits_used` (rounded up to whole bytes). Layer I frame sizes
        // are quantised to 4-byte slots, so the smallest fit is unique
        // among the ladder entries.
        let (br_idx_pick, slot_kbps) = pick_vbr_layer1_slot(self.sample_rate, bits_used as usize);
        // Frame bytes for that slot, no padding (VBR doesn't use the
        // CBR padding accumulator — slot changes already absorb the
        // fractional-byte slack).
        let slots = 12 * slot_kbps * 1000 / self.sample_rate;
        let frame_bytes = (slots * 4) as usize;
        Ok((alloc, frame_bytes, false, br_idx_pick as u32))
    }
}

impl Encoder for Mp1Encoder {
    fn codec_id(&self) -> &CodecId {
        &self.output_params.codec_id
    }
    fn output_params(&self) -> &CodecParameters {
        &self.output_params
    }
    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        match frame {
            Frame::Audio(a) => self.ingest(a),
            _ => Err(Error::invalid("MP1 encoder: audio frames only")),
        }
    }
    fn receive_packet(&mut self) -> Result<Packet> {
        self.pending_packets.pop_front().ok_or(Error::NeedMore)
    }
    fn flush(&mut self) -> Result<()> {
        if !self.eof {
            self.eof = true;
            self.flush_ready_frames(true)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Helper functions.
// ---------------------------------------------------------------------------

/// Reverse-map a bitrate in kbps to its 4-bit header-field index (1..=14
/// for MPEG-1 Layer I). Returns `None` for unsupported values.
fn bitrate_to_index(kbps: u32) -> Option<u32> {
    LAYER1_BITRATES_KBPS
        .iter()
        .skip(1) // skip "free format"
        .position(|&v| v == kbps)
        .map(|idx| (idx + 1) as u32)
}

/// Pick the smallest standard Layer I bitrate slot whose unpadded
/// frame can accommodate `needed_bits` (header + bit-allocation +
/// scalefactors + samples). Returns `(bitrate_index, kbps)`.
///
/// Layer I frame size: `frame_bytes = (12 * bitrate / sample_rate) * 4`.
/// We iterate from the smallest slot to the largest and return the
/// first one whose frame is big enough. If none of the standard slots
/// fits — which can happen when even 448 kbps falls short for a noisy
/// frame at a high target — we return the maximum slot and the caller
/// truncates.
fn pick_vbr_layer1_slot(sample_rate: u32, needed_bits: usize) -> (u8, u32) {
    let needed_bytes = needed_bits.div_ceil(8);
    for (i, &kbps) in LAYER1_BITRATES_KBPS.iter().enumerate() {
        if i == 0 {
            continue; // skip "free format"
        }
        let slots = 12 * kbps * 1000 / sample_rate;
        let frame_bytes = (slots * 4) as usize;
        if frame_bytes >= needed_bytes {
            return (i as u8, kbps);
        }
    }
    let last_idx = (LAYER1_BITRATES_KBPS.len() - 1) as u8;
    (last_idx, LAYER1_BITRATES_KBPS[last_idx as usize])
}

/// Energy-per-bit greedy allocator shared by CBR and (the fallback path
/// of) VBR. Walks subband × channel pairs and grants the upgrade with
/// the highest energy-per-bit ratio until no upgrade fits in
/// `budget_bits`. Skips bands more than 80 dB below the loudest band
/// (cheap proxy for "no signal here").
///
/// `bound` is the joint-stereo bound: for subbands `[bound..32)` the two
/// channels share one allocation + sample stream (Layer I §2.4.1.5), so
/// only channel 0 is a candidate there and its grant is mirrored into
/// channel 1; the cost (one sample stream, two scalefactors) is counted
/// via [`upgrade_cost_bits_bound`]. `bound == SBLIMIT` for mono / plain
/// stereo, which reduces this to the original per-channel walk.
fn greedy_energy_allocator(
    n_ch: usize,
    bound: usize,
    energy: &[[f32; 32]],
    _scf_idx: &[[u8; 32]],
    budget_bits: i64,
) -> Vec<[u8; 32]> {
    let mut alloc: Vec<[u8; 32]> = vec![[0u8; 32]; n_ch];
    let mut remaining: i64 = budget_bits;

    let peak_energy = energy
        .iter()
        .flat_map(|row| row.iter().copied())
        .fold(0.0f32, f32::max);
    let noise_floor = peak_energy * 1e-8; // ~-80 dB power

    loop {
        let mut best: Option<(usize, usize, u8, i64)> = None;
        let mut best_score = f32::NEG_INFINITY;

        for ch in 0..n_ch {
            for sb in 0..SBLIMIT {
                // Shared subbands: channel 0 is the sole candidate.
                if sb >= bound && ch != 0 {
                    continue;
                }
                let cur = alloc[ch][sb];
                if cur >= 14 {
                    continue;
                }
                if energy[ch][sb] <= noise_floor {
                    continue;
                }
                let next = cur + 1;
                let cost = upgrade_cost_bits_bound(cur, next, sb, bound, n_ch);
                let score = energy[ch][sb] / (cost as f32).max(1.0);
                if score > best_score && cost as i64 <= remaining {
                    best_score = score;
                    best = Some((ch, sb, next, cost as i64));
                }
            }
        }

        match best {
            Some((ch, sb, next, cost)) => {
                alloc[ch][sb] = next;
                if sb >= bound {
                    alloc[1.min(n_ch - 1)][sb] = next;
                }
                remaining -= cost;
            }
            None => break,
        }
    }
    alloc
}

/// Given a signal peak magnitude, pick the smallest scalefactor index
/// whose magnitude `>= peak` (we want the tightest step size that still
/// covers the peak without clipping). Layer I SCALE[i] = 2 * 2^(-i/3),
/// so larger `i` means smaller magnitude.
///
/// Falls back to index 62 (smallest SF) for tiny peaks, and to index 0
/// (largest SF) for peaks that exceed the table range.
fn pick_scalefactor(peak: f32) -> u8 {
    if !peak.is_finite() || peak <= 0.0 {
        return 62;
    }
    let sc = scale_table();
    // We want the largest i such that SCALE[i] >= peak. SCALE is
    // monotonically decreasing, so scan from 0 and keep the last i that
    // satisfies the condition.
    let mut best = 0u8;
    for i in 0..63u8 {
        if sc[i as usize] >= peak {
            best = i;
        } else {
            break;
        }
    }
    best
}

/// Extra bits required when upgrading subband allocation from `cur` to
/// `next`. Accounts for 12 sample slots and a 6-bit scalefactor that is
/// only paid once when the allocation transitions 0 → non-zero.
fn upgrade_cost_bits(cur: u8, next: u8) -> u32 {
    debug_assert!(next > cur);
    let cur_nb = if cur == 0 { 0 } else { cur as u32 + 1 };
    let next_nb = next as u32 + 1;
    let sample_delta = 12 * (next_nb - cur_nb);
    if cur == 0 {
        // Pay scalefactor bits once.
        sample_delta + 6
    } else {
        sample_delta
    }
}

/// Bound-aware variant of [`upgrade_cost_bits`]. For a subband below the
/// joint-stereo `bound` (or in mono / plain stereo where `bound ==
/// SBLIMIT`) the cost is identical to [`upgrade_cost_bits`]: one sample
/// stream and one scalefactor. For a subband at or above the bound the
/// two channels share a single quantised sample stream but still carry
/// *one scalefactor each* (§2.4.1.5), so the 0 → non-zero transition
/// pays `n_ch` scalefactors (12 bits for stereo) on top of the 12 shared
/// sample slots; subsequent upgrades pay only the 12 shared sample-slot
/// deltas.
fn upgrade_cost_bits_bound(cur: u8, next: u8, sb: usize, bound: usize, n_ch: usize) -> u32 {
    debug_assert!(next > cur);
    if sb < bound {
        return upgrade_cost_bits(cur, next);
    }
    let cur_nb = if cur == 0 { 0 } else { cur as u32 + 1 };
    let next_nb = next as u32 + 1;
    let sample_delta = 12 * (next_nb - cur_nb); // one shared stream
    if cur == 0 {
        sample_delta + 6 * n_ch as u32 // one scalefactor per channel
    } else {
        sample_delta
    }
}

/// Quantise a normalised subband sample `s` (in [-1, 1]) for a subband
/// with scalefactor magnitude `sf_mag` and Layer I allocation `nb` bits.
///
/// Decoder computes: `out = (raw - 2^(nb-1) + 1) * 2/(2^nb - 1) * sf_mag`.
/// Inverting: `raw = round(s * (2^nb - 1) / (2 * sf_mag)) + 2^(nb-1) - 1`,
/// clamped to `0..=2^nb - 1`.
fn quantise_sample(s: f32, sf_mag: f32, nb: u8) -> u32 {
    let denom = (1u32 << nb) - 1; // 2^nb - 1
    let offset = (1i32 << (nb - 1)) - 1; // 2^(nb-1) - 1
    let max_code = denom;
    let sf = sf_mag.max(1e-30);
    let level = (s * (denom as f32) / (2.0 * sf)).round() as i32;
    // Decoder accepts level ∈ [-(offset), offset + 1] → raw ∈ [0, 2^nb-1].
    let raw = level + offset;
    raw.clamp(0, max_code as i32) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bitrate_index_lookup() {
        assert_eq!(bitrate_to_index(32), Some(1));
        assert_eq!(bitrate_to_index(192), Some(6));
        assert_eq!(bitrate_to_index(448), Some(14));
        assert_eq!(bitrate_to_index(999), None);
    }

    #[test]
    fn bound_aware_upgrade_cost() {
        // Below the bound (or mono / plain stereo), the cost matches the
        // per-channel formula: 12 sample slots + one 6-bit SF on the
        // 0 → non-zero transition.
        assert_eq!(
            upgrade_cost_bits_bound(0, 1, 0, 32, 2),
            upgrade_cost_bits(0, 1)
        );
        assert_eq!(
            upgrade_cost_bits_bound(3, 4, 0, 8, 2),
            upgrade_cost_bits(3, 4)
        );
        // Above the bound, the 0 → non-zero transition pays one
        // scalefactor PER channel (12 bits stereo) on top of the single
        // shared 12-sample stream (alloc 0→1 ⇒ nb 0→2 ⇒ 24 sample bits).
        assert_eq!(upgrade_cost_bits_bound(0, 1, 16, 8, 2), 12 * 2 + 6 * 2);
        // A subsequent upgrade above the bound pays only the shared
        // sample-slot delta (nb 2→3 ⇒ 12 bits), no extra scalefactors.
        assert_eq!(upgrade_cost_bits_bound(1, 2, 16, 8, 2), 12);
        // Mono above-bound is a degenerate case (n_ch = 1): one SF.
        assert_eq!(upgrade_cost_bits_bound(0, 1, 16, 8, 1), 12 * 2 + 6);
    }

    #[test]
    fn scalefactor_pick_reasonable() {
        // Peak 1.0 should land at an index whose magnitude >= 1.0.
        let i = pick_scalefactor(1.0);
        let sc = scale_table();
        let mag = sc[i as usize];
        assert!(mag >= 1.0, "mag={mag} i={i}");
        // Tiny peak → large index.
        let i = pick_scalefactor(1e-8);
        assert!(i >= 50, "got {i}");
    }

    #[test]
    fn quantise_roundtrip_basic() {
        // For nb=15 and SF index 3 (mag = 1.0), the decoder's dequant
        // factor is 2/(2^15 - 1). Quantising a sample near 0.5 should
        // give a raw code whose dequantised value is close.
        let sf_mag = 1.0f32;
        let s = 0.5f32;
        let raw = quantise_sample(s, sf_mag, 15);
        let denom = (1u32 << 15) - 1;
        let level = raw as i32 - (1i32 << 14) + 1;
        let recovered = level as f32 * 2.0 / denom as f32 * sf_mag;
        assert!((recovered - s).abs() < 1e-3, "recovered {recovered} vs {s}");
    }

    #[test]
    fn encoder_roundtrip_silence() {
        use crate::decoder::make_decoder;
        use oxideav_core::Frame as CoreFrame;

        let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        params.channels = Some(1);
        params.sample_rate = Some(44_100);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(128_000);
        let mut enc = make_encoder(&params).unwrap();

        // Feed 3 frames of silence.
        let n = SAMPLES_PER_FRAME * 3;
        let mut data = Vec::new();
        for _ in 0..n {
            data.extend_from_slice(&0i16.to_le_bytes());
        }
        let frame = AudioFrame {
            samples: n as u32,
            pts: Some(0),
            data: vec![data],
        };
        enc.send_frame(&CoreFrame::Audio(frame)).unwrap();
        let mut packets: Vec<Packet> = Vec::new();
        while let Ok(p) = enc.receive_packet() {
            packets.push(p);
        }
        enc.flush().unwrap();
        while let Ok(p) = enc.receive_packet() {
            packets.push(p);
        }
        assert!(!packets.is_empty(), "no packets produced");

        // Decode back.
        let dparams = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        let mut dec = make_decoder(&dparams).unwrap();
        let mut decoded = 0u32;
        for p in &packets {
            dec.send_packet(p).unwrap();
            if let Ok(CoreFrame::Audio(a)) = dec.receive_frame() {
                decoded += a.samples;
            }
        }
        assert!(
            decoded >= SAMPLES_PER_FRAME as u32,
            "decoded too few samples: {decoded}"
        );
    }

    /// Build an encoder with the `crc_check` option set and feed it a
    /// short tone; returns the emitted packets.
    #[cfg(test)]
    fn encode_tone_with_crc(channels: u16) -> Vec<Packet> {
        use oxideav_core::options::CodecOptions;
        use oxideav_core::Frame as CoreFrame;

        let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        params.channels = Some(channels);
        params.sample_rate = Some(44_100);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(192_000);
        params.options = CodecOptions::new().set("crc_check", "true");
        let mut enc = make_encoder(&params).unwrap();

        let n = SAMPLES_PER_FRAME * 4;
        let mut data = Vec::new();
        for i in 0..n {
            let t = i as f32 / 44_100.0;
            let s = (2.0 * std::f32::consts::PI * 1000.0 * t).sin();
            let q = (s * 20_000.0) as i16;
            for _ in 0..channels {
                data.extend_from_slice(&q.to_le_bytes());
            }
        }
        let frame = AudioFrame {
            samples: n as u32,
            pts: Some(0),
            data: vec![data],
        };
        enc.send_frame(&CoreFrame::Audio(frame)).unwrap();
        enc.flush().unwrap();
        let mut packets = Vec::new();
        while let Ok(p) = enc.receive_packet() {
            packets.push(p);
        }
        packets
    }

    /// With `crc_check` enabled the encoder must clear the protection
    /// bit and insert a 16-bit CRC word, and our decoder must accept
    /// the resulting frames (CRC verifies).
    #[test]
    fn crc_emit_and_decode_roundtrips() {
        use crate::decoder::make_decoder;
        use crate::header::FrameHeader;
        use oxideav_core::Frame as CoreFrame;

        for &channels in &[1u16, 2] {
            let packets = encode_tone_with_crc(channels);
            assert!(!packets.is_empty(), "no packets for {channels}ch");

            for p in &packets {
                let h = FrameHeader::parse(&p.data).unwrap();
                // protection_bit == 0 ⇒ `protection == true` in our header.
                assert!(h.protection, "CRC frame must signal protection");
                // CRC word is bytes 4..6 and must be non-trivial for a
                // tone (the all-ones init evolves under real allocation).
                assert!(p.data.len() >= 6, "frame too short for CRC word");
            }

            // Decode every frame: a CRC mismatch would surface as an
            // error from receive_frame.
            let dparams = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
            let mut dec = make_decoder(&dparams).unwrap();
            let mut decoded = 0u32;
            for p in &packets {
                dec.send_packet(p).unwrap();
                match dec.receive_frame() {
                    Ok(CoreFrame::Audio(a)) => decoded += a.samples,
                    Ok(_) => panic!("unexpected non-audio frame"),
                    Err(e) => panic!("CRC frame rejected on decode: {e}"),
                }
            }
            assert!(decoded > 0, "no samples decoded for {channels}ch");
        }
    }

    /// Corrupting the stored CRC word must make the decoder reject the
    /// frame on CRC mismatch — this isolates the CRC check from the
    /// forbidden-allocation-code (15) guard, which runs earlier.
    #[test]
    fn crc_detects_corruption() {
        use crate::decoder::make_decoder;

        let packets = encode_tone_with_crc(2);
        let mut pkt = packets[0].clone();
        // Bytes 4..6 hold the stored CRC word. Flipping a bit there
        // leaves the protected field intact but the stored != computed,
        // so the decoder must report a CRC mismatch.
        let original = pkt.data[5];
        pkt.data[5] ^= 0x01;
        assert_ne!(pkt.data[5], original);

        let dparams = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        let mut dec = make_decoder(&dparams).unwrap();
        dec.send_packet(&pkt).unwrap();
        let res = dec.receive_frame();
        assert!(
            res.is_err(),
            "decoder accepted a CRC-corrupted frame: {res:?}"
        );

        // And a protected-field corruption (a bit-allocation byte) is
        // likewise caught — either by the CRC or the forbidden-code
        // guard; both are decode-time rejections.
        let mut pkt2 = packets[0].clone();
        pkt2.data[6] ^= 0x10;
        let mut dec2 = make_decoder(&dparams).unwrap();
        dec2.send_packet(&pkt2).unwrap();
        assert!(
            dec2.receive_frame().is_err(),
            "decoder accepted a frame with a corrupted allocation byte"
        );
    }

    /// Default (no `crc_check`) keeps protection_bit = 1 and no CRC word
    /// — backward-compatible with the historical encoder output.
    #[test]
    fn no_crc_by_default() {
        use crate::header::FrameHeader;

        let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        params.channels = Some(2);
        params.sample_rate = Some(44_100);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(192_000);
        let mut enc = make_encoder(&params).unwrap();

        let n = SAMPLES_PER_FRAME;
        let data = vec![0u8; n * 2 * 2];
        let frame = AudioFrame {
            samples: n as u32,
            pts: Some(0),
            data: vec![data],
        };
        enc.send_frame(&Frame::Audio(frame)).unwrap();
        enc.flush().unwrap();
        let p = enc.receive_packet().unwrap();
        let h = FrameHeader::parse(&p.data).unwrap();
        assert!(!h.protection, "default output must have protection_bit = 1");
    }
}
