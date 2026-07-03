//! `oxideav_core::Decoder` wiring for MPEG-1 Audio Layer I.
//!
//! [`Mp1Decoder`] turns one compressed Layer I packet into an
//! interleaved S16 [`AudioFrame`]. It ties together the three decode
//! stages built in the sibling modules:
//!
//! 1. [`FrameHeader::parse`](crate::header::FrameHeader::parse) — the
//!    §2.4.1.3 32-bit header, plus the optional CRC word (§2.4.1.4),
//!    verified via
//!    [`FrameHeader::verify_crc`](crate::header::FrameHeader::verify_crc)
//!    against the §2.4.3.1 polynomial; a mismatch mutes the frame
//!    (§2.4.3.1 concealment).
//! 2. [`decode_audio_data`] — the §2.4.1.5 / §2.4.3.2 bit-allocation,
//!    scalefactor and sample requantization, producing 32 × 12
//!    requantized subband samples per channel.
//! 3. [`SynthesisFilter`] — the §2.4.3.2 polyphase synthesis
//!    filterbank, run once per sample-slot per channel after the
//!    Table 3-B.1 rescale, giving 384 PCM samples per channel per
//!    frame.
//!
//! One packet is expected to hold exactly one Layer I frame (the
//! standard framing produced by every MP1 muxer). The decoder finds
//! the syncword, decodes the single frame, and emits its 384-sample
//! PCM frame; trailing bytes after the frame are ignored.

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecRegistry, CodecTag,
    Decoder, Encoder, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::decode::{decode_audio_data, SubbandSamples, SAMPLES_PER_SUBBAND, SUBBANDS};
use crate::decode_layer2::{
    decode_layer2_audio_data, verify_layer2_crc, Layer2Subbands, LAYER2_SAMPLES_PER_FRAME,
    LAYER2_SAMPLES_PER_SUBBAND,
};
use crate::encode::{
    EncodeParams, Layer2HeaderParams, LayerSelect, Mp1FrameEncoder, Mp1Layer2FrameEncoder,
};
use crate::header::{find_sync, Bitrate, CrcStatus, FrameHeader, Layer, Mode};
use crate::synthesis::{to_s16, SynthesisFilter};

/// The canonical codec id for MPEG-1 Audio Layer I.
const CODEC_ID: &str = "mp1";

/// The §2.4.3.1 error-concealment strategy applied when a
/// CRC-protected frame fails its `error_check()`.
///
/// §2.4.3.1 recommends, *verbatim*, "application of a concealment
/// technique, such as **muting of the actual frame** or **repetition
/// of the previous frame**". Both are implemented here and selectable
/// at decoder-construction time (or at runtime via
/// [`Mp1Decoder::set_concealment`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConcealmentMode {
    /// Mute the offending frame: the decoder emits silence for the
    /// frame and rings the synthesis filterbank history out with
    /// zeros (the previous frame's filter tail still decays naturally,
    /// so there is no hard discontinuity click). This is the default
    /// and matches the prior crate behaviour.
    #[default]
    Mute,
    /// Repeat the previous frame: the decoder re-uses the last
    /// successfully-decoded frame's requantized subband samples and
    /// runs them through the synthesis filterbank again, producing a
    /// fresh 384-sample PCM frame. If no frame has decoded
    /// successfully yet (the corrupt frame is the very first one),
    /// there is nothing to repeat, so this falls back to
    /// [`ConcealmentMode::Mute`] for that frame only.
    RepeatPrevious,
}

/// Build an [`Mp1Decoder`] for the given parameters. The channel count
/// is taken from the frame headers at decode time, so `params` only
/// needs to carry the codec id.
pub(crate) fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(Mp1Decoder::new(params.codec_id.clone())))
}

/// A packet-to-frame MPEG-1 Audio Layer I decoder.
///
/// Holds one synthesis filterbank per channel (max two) so the
/// overlap-add `V` history (§2.4.3.2, Figure 3-A.2) carries across
/// frames. [`reset`](Decoder::reset) zeroes that history after a seek.
#[derive(Debug)]
pub struct Mp1Decoder {
    codec_id: CodecId,
    /// One synthesis bank per channel; lazily extended to the channel
    /// count of the first decoded frame.
    filters: Vec<SynthesisFilter>,
    pending: Option<Packet>,
    eof: bool,
    /// The §2.4.3.1 concealment strategy applied on CRC mismatch.
    concealment: ConcealmentMode,
    /// The requantized subband samples of the last *successfully*
    /// decoded frame, kept for [`ConcealmentMode::RepeatPrevious`].
    /// `None` until a frame has decoded without a CRC failure.
    last_subbands: Option<SubbandSamples>,
    /// The Layer II requantized subband samples of the last
    /// *successfully* decoded Layer II frame, kept for
    /// [`ConcealmentMode::RepeatPrevious`]. Layer II carries 36
    /// sample-slots per subband (1152 samples / channel) so it cannot
    /// share storage with the 12-slot Layer I [`SubbandSamples`].
    /// `None` until a Layer II frame has decoded without a CRC failure.
    last_layer2_subbands: Option<Layer2Subbands>,
}

impl Mp1Decoder {
    /// Build a Layer I decoder for the given codec id, with the default
    /// [`ConcealmentMode::Mute`] CRC concealment.
    ///
    /// The channel count is taken from the frame headers at decode
    /// time, so the codec id is the only thing the decoder needs up
    /// front. Use [`with_concealment`](Self::with_concealment) to pick
    /// [`ConcealmentMode::RepeatPrevious`] instead.
    pub fn new(codec_id: CodecId) -> Mp1Decoder {
        Mp1Decoder {
            codec_id,
            filters: Vec::new(),
            pending: None,
            eof: false,
            concealment: ConcealmentMode::default(),
            last_subbands: None,
            last_layer2_subbands: None,
        }
    }

    /// Set the §2.4.3.1 concealment strategy applied when a
    /// CRC-protected frame fails verification (builder form).
    ///
    /// Defaults to [`ConcealmentMode::Mute`]. Use
    /// [`ConcealmentMode::RepeatPrevious`] to repeat the previous
    /// frame's subband samples instead of muting.
    pub fn with_concealment(mut self, mode: ConcealmentMode) -> Mp1Decoder {
        self.concealment = mode;
        self
    }

    /// Set the §2.4.3.1 concealment strategy at runtime.
    ///
    /// Takes effect for every frame decoded after the call.
    pub fn set_concealment(&mut self, mode: ConcealmentMode) {
        self.concealment = mode;
    }

    /// The §2.4.3.1 concealment strategy currently in effect.
    pub fn concealment(&self) -> ConcealmentMode {
        self.concealment
    }

    /// Decode one Layer I or Layer II frame found in `data` into
    /// interleaved S16 PCM. Returns
    /// `(pcm_bytes, samples_per_channel, channels)`.
    fn decode_frame(&mut self, data: &[u8]) -> Result<(Vec<u8>, u32, usize)> {
        // Locate the frame sync. find_sync validates the whole 32-bit
        // header so a leading ID3 tag / junk is skipped.
        let off = find_sync(data)
            .ok_or_else(|| Error::invalid("oxideav-mp1: no Layer I/II frame sync in packet"))?;
        let frame = &data[off..];
        let header = FrameHeader::parse(frame)
            .map_err(|e| Error::invalid(format!("oxideav-mp1: header parse: {e}")))?;

        let nch = header.channels() as usize;
        // Audio data begins after the 4 header bytes and the optional
        // 16-bit CRC word (§2.4.1.4).
        let mut audio_start = 4;
        if header.has_crc() {
            audio_start += 2;
        }
        if frame.len() < audio_start {
            return Err(Error::invalid(
                "oxideav-mp1: packet too short for header + CRC",
            ));
        }
        let audio = &frame[audio_start..];

        // Layer II frames go through their own audio_data() decode and
        // synthesis loop (36 sample-slots per channel instead of 12).
        // §2.4.3.1 + Annex B Table 3-B.5: the Layer II protected fields
        // are header bits 16…31 + the §2.4.1.6 allocation field + the
        // §2.4.1.6 scfsi field. `verify_layer2_crc` walks the allocation
        // field to determine how many scfsi bits are present, then
        // CRCs the full protected region. On a mismatch the same
        // §2.4.3.1 concealment strategy as Layer I applies (mute or
        // repeat-previous).
        if matches!(header.layer, Layer::II) {
            if header.has_crc() {
                match verify_layer2_crc(&header, &frame[..4], &frame[4..]) {
                    Some(CrcStatus::Mismatch { .. }) => {
                        return Ok(self.conceal_layer2_frame(nch));
                    }
                    None => {
                        return Err(Error::invalid(
                            "oxideav-mp1: Layer II packet too short for CRC-protected field",
                        ));
                    }
                    _ => {}
                }
            }
            return self.decode_layer2_frame(&header, audio);
        }

        // §2.4.3.1 error_check(): when the frame is CRC-protected,
        // verify the stored word over header bits 16…31 + the
        // bit-allocation field (Table 3-B.5). On a mismatch §2.4.3.1
        // recommends concealment ("muting of the actual frame or
        // repetition of the previous frame"). Which one is applied is
        // selected by `self.concealment`.
        if header.has_crc() {
            match header.verify_crc(&frame[..4], &frame[4..]) {
                Some(CrcStatus::Mismatch { .. }) => {
                    return Ok(self.conceal_frame(nch));
                }
                // Ok / Absent: proceed. None means the slice was too
                // short for the protected field — treat as truncated.
                None => {
                    return Err(Error::invalid(
                        "oxideav-mp1: packet too short for CRC-protected field",
                    ));
                }
                _ => {}
            }
        }

        let subbands = decode_audio_data(&header, audio)
            .map_err(|e| Error::invalid(format!("oxideav-mp1: audio_data: {e}")))?;

        let out = self.synthesize_subbands(&subbands, nch);
        // Remember this good frame's subbands for a future
        // RepeatPrevious concealment (§2.4.3.1).
        self.last_subbands = Some(subbands);
        Ok(out)
    }

    /// Decode one Layer II frame's audio_data + run the §2.4.3.2
    /// polyphase synthesis filterbank over its 36 sample-slots,
    /// producing 1152 PCM samples per channel.
    fn decode_layer2_frame(
        &mut self,
        header: &FrameHeader,
        audio: &[u8],
    ) -> Result<(Vec<u8>, u32, usize)> {
        let nch = header.channels() as usize;
        let subbands = decode_layer2_audio_data(header, audio)
            .map_err(|e| Error::invalid(format!("oxideav-mp1: layer II audio_data: {e}")))?;
        let out = self.synthesize_layer2_subbands(&subbands, nch);
        // Remember this good frame's Layer II subbands for a future
        // RepeatPrevious concealment (§2.4.3.1).
        self.last_layer2_subbands = Some(subbands);
        Ok(out)
    }

    /// Apply the selected §2.4.3.1 concealment for a Layer II CRC
    /// failure.
    ///
    /// * [`ConcealmentMode::Mute`] — push 36 slots of zero subband
    ///   samples through each channel's bank: the frame is silent and
    ///   the previous frame's filter tail rings out continuously.
    /// * [`ConcealmentMode::RepeatPrevious`] — re-synthesize the last
    ///   successfully-decoded Layer II frame's requantized subband
    ///   samples. If no good Layer II frame has decoded yet, this
    ///   falls back to muting for that frame.
    ///
    /// Returns `(pcm_bytes, samples_per_channel, channels)` like
    /// [`decode_frame`](Self::decode_frame). A concealed frame never
    /// becomes the "previous frame" for a subsequent repeat — same
    /// last-good-only semantics as the Layer I concealment path.
    fn conceal_layer2_frame(&mut self, nch: usize) -> (Vec<u8>, u32, usize) {
        match self.concealment {
            ConcealmentMode::RepeatPrevious => {
                if let Some(prev) = self.last_layer2_subbands.clone() {
                    let prev_nch = prev.channels.clamp(1, 2);
                    return self.synthesize_layer2_subbands(&prev, prev_nch);
                }
                self.conceal_muted_layer2_frame(nch)
            }
            ConcealmentMode::Mute => self.conceal_muted_layer2_frame(nch),
        }
    }

    /// Build a muted Layer II concealment frame (§2.4.3.1 "muting of the
    /// actual frame"): an empty [`Layer2Subbands`] pushed through each
    /// channel's synthesis bank so the overlap-add `V` history advances
    /// continuously rather than being cut, giving the standard's
    /// "mute" behaviour without a discontinuity click.
    fn conceal_muted_layer2_frame(&mut self, nch: usize) -> (Vec<u8>, u32, usize) {
        // `Layer2Subbands::silent` carries `sblimit = 0`, forcing every
        // (sb, slot) sample to zero through `Layer2Subbands::slot` —
        // the §2.4.3.1 muting shape for Layer II.
        let zeros = Layer2Subbands::silent(nch);
        self.synthesize_layer2_subbands(&zeros, nch)
    }

    /// Run the §2.4.3.2 polyphase synthesis filterbank over a Layer II
    /// frame's 36 sub-band sample-slots, packing the result into
    /// interleaved S16 PCM (1152 samples per channel).
    fn synthesize_layer2_subbands(
        &mut self,
        subbands: &Layer2Subbands,
        nch: usize,
    ) -> (Vec<u8>, u32, usize) {
        while self.filters.len() < nch {
            self.filters.push(SynthesisFilter::new());
        }
        let total_samples = LAYER2_SAMPLES_PER_SUBBAND * 32; // 1152 per channel
        let mut pcm = vec![0i16; total_samples * nch];
        for slot in 0..LAYER2_SAMPLES_PER_SUBBAND {
            for ch in 0..nch {
                let input = subbands.slot(ch, slot);
                let out = self.filters[ch].synthesize(&input);
                for (j, &v) in out.iter().enumerate() {
                    let frame_sample = slot * 32 + j;
                    pcm[frame_sample * nch + ch] = to_s16(v);
                }
            }
        }
        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        (bytes, total_samples as u32, nch)
    }

    /// Run the §2.4.3.2 polyphase synthesis filterbank over one frame's
    /// requantized subband samples and pack the result into interleaved
    /// S16 PCM.
    ///
    /// For each of the 12 slots and each channel, the 32 rescaled
    /// subband samples are pushed through that channel's bank (the
    /// overlap-add `V` history persists across calls). Returns
    /// `(pcm_bytes, samples_per_channel, channels)`.
    fn synthesize_subbands(
        &mut self,
        subbands: &SubbandSamples,
        nch: usize,
    ) -> (Vec<u8>, u32, usize) {
        // Ensure we have one synthesis bank per channel (state persists
        // across frames for the overlap-add history).
        while self.filters.len() < nch {
            self.filters.push(SynthesisFilter::new());
        }

        let total_samples = SAMPLES_PER_SUBBAND * 32; // 384 per channel
        let mut pcm = vec![0i16; total_samples * nch];
        for slot in 0..SAMPLES_PER_SUBBAND {
            for ch in 0..nch {
                let input = subbands.slot(ch, slot);
                let out = self.filters[ch].synthesize(&input);
                // 32 PCM samples for this (slot, ch). Interleave:
                // overall sample index = slot*32 + j, channel ch.
                for (j, &v) in out.iter().enumerate() {
                    let frame_sample = slot * 32 + j;
                    pcm[frame_sample * nch + ch] = to_s16(v);
                }
            }
        }

        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        (bytes, total_samples as u32, nch)
    }

    /// Apply the selected §2.4.3.1 concealment for a CRC failure.
    ///
    /// * [`ConcealmentMode::Mute`] — push zero subband samples through
    ///   each channel's bank: the frame is silent and the previous
    ///   frame's filter tail rings out continuously.
    /// * [`ConcealmentMode::RepeatPrevious`] — re-synthesize the last
    ///   successfully-decoded frame's requantized subband samples. If
    ///   no good frame has decoded yet (the corrupt frame is the very
    ///   first), there is nothing to repeat and this falls back to
    ///   muting for that frame.
    ///
    /// Returns `(pcm_bytes, samples_per_channel, channels)` like
    /// [`decode_frame`](Self::decode_frame). A concealed frame never
    /// becomes the "previous frame" for a subsequent repeat: only a
    /// genuinely-decoded frame is stored, so a run of corrupt frames
    /// repeats the *last good* frame each time rather than chaining
    /// repeats of repeats.
    fn conceal_frame(&mut self, nch: usize) -> (Vec<u8>, u32, usize) {
        match self.concealment {
            ConcealmentMode::RepeatPrevious => {
                if let Some(prev) = self.last_subbands.clone() {
                    // Repeat the previous frame's subband samples,
                    // honouring the channel count it was decoded with
                    // (a mid-stream channel change while corrupt is
                    // exotic; reuse the stored channel count).
                    let prev_nch = prev.channels.clamp(1, 2);
                    return self.synthesize_subbands(&prev, prev_nch);
                }
                // No previous frame: fall back to muting.
                self.conceal_muted_frame(nch)
            }
            ConcealmentMode::Mute => self.conceal_muted_frame(nch),
        }
    }

    /// Build a muted concealment frame (§2.4.3.1: "muting of the actual
    /// frame").
    ///
    /// Zero subband samples are pushed through each channel's synthesis
    /// bank so the overlap-add `V` history advances continuously (the
    /// previous frame's filter tail rings out rather than being cut),
    /// giving the standard's "mute" behaviour without a discontinuity
    /// click.
    fn conceal_muted_frame(&mut self, nch: usize) -> (Vec<u8>, u32, usize) {
        let zeros = SubbandSamples::silent(nch);
        self.synthesize_subbands(&zeros, nch)
    }
}

impl Decoder for Mp1Decoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.pending.is_some() {
            return Err(Error::other(
                "oxideav-mp1: call receive_frame before sending another packet",
            ));
        }
        self.pending = Some(packet.clone());
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let Some(pkt) = self.pending.take() else {
            return if self.eof {
                Err(Error::Eof)
            } else {
                Err(Error::NeedMore)
            };
        };
        if pkt.data.is_empty() {
            return Ok(Frame::Audio(AudioFrame {
                samples: 0,
                pts: pkt.pts,
                data: vec![Vec::new()],
            }));
        }
        let (bytes, samples, _nch) = self.decode_frame(&pkt.data)?;
        Ok(Frame::Audio(AudioFrame {
            samples,
            pts: pkt.pts,
            data: vec![bytes],
        }))
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.pending = None;
        self.eof = false;
        // Drop the repeat-concealment history: after a seek there is no
        // meaningful "previous frame" to repeat. The concealment *mode*
        // is a configured policy and is intentionally preserved.
        self.last_subbands = None;
        self.last_layer2_subbands = None;
        for f in &mut self.filters {
            f.reset();
        }
        Ok(())
    }
}

/// Build an [`Mp1Encoder`] from the supplied parameters. `sample_rate`
/// is required (one of the six Layer I rates: MPEG-1 32 / 44.1 / 48
/// kHz from 11172-3 §2.4.2.3, or MPEG-2 LSF 16 / 22.05 / 24 kHz from
/// 13818-3 §2.4.2.3), `channels` is required (1 or 2), and `bit_rate`
/// is optional — it defaults to a sensible per-channel value if
/// absent. The chosen mode is mono for `channels == 1` and stereo for
/// `channels == 2`. The §2.4.1.4 CRC is **not** emitted; the encoded
/// frame's `protection_bit` is `1`. Use
/// [`make_encoder_with_crc`] to enable optional CRC emission, or
/// [`make_encoder_layer2`] to produce Layer II frames instead.
pub(crate) fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    make_encoder_inner(params, /*emit_crc=*/ false, LayerSelect::LayerI)
}

/// Build an [`Mp1Encoder`] from `params`, with the optional §2.4.1.4
/// CRC `error_check()` enabled.
///
/// Identical to [`make_encoder`] except the produced encoder writes
/// `protection_bit == 0` and a 16-bit CRC word computed over the Annex
/// B Table 3-B.5 Layer I protected fields (§2.4.3.1 polynomial
/// `G(X) = X^16 + X^15 + X^2 + 1`, init `0xFFFF`). The CRC's 16 bits are
/// taken out of the per-frame audio-data budget, so the slot count
/// reported by the header stays at the §2.4.2.1
/// `N = floor(12 · bitrate / Fs) + padding` value.
pub(crate) fn make_encoder_with_crc(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    make_encoder_inner(params, /*emit_crc=*/ true, LayerSelect::LayerI)
}

/// Build an [`Mp1Encoder`] that emits **Layer II** (§2.4.1.6) frames.
///
/// Identical parameter contract to [`make_encoder`] (`sample_rate` and
/// `channels` required, `bit_rate` optional), but the produced encoder
/// dispatches to [`Mp1Layer2FrameEncoder`] and consumes **1152** PCM
/// samples per channel per [`Encoder::send_frame`] call (the §2.4.2.1
/// Layer II frame granularity) rather than the Layer I 384.
///
/// The default per-rate bitrate is picked from the §2.4.2.3 Layer II
/// ladder (mono-channel midpoint for `channels == 1`, stereo midpoint
/// for `channels == 2`); the LSF rates (13818-3 §2.4.2.3) draw from
/// the LSF Layer II ladder. The §2.4.1.4 CRC is **not** emitted by
/// default.
pub(crate) fn make_encoder_layer2(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    make_encoder_inner(params, /*emit_crc=*/ false, LayerSelect::LayerII)
}

fn make_encoder_inner(
    params: &CodecParameters,
    emit_crc: bool,
    layer: LayerSelect,
) -> Result<Box<dyn Encoder>> {
    let sample_rate = params
        .sample_rate
        .ok_or_else(|| Error::invalid("oxideav-mp1: sample_rate required"))?;
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("oxideav-mp1: channels required"))?;
    let mode = match channels {
        1 => Mode::SingleChannel,
        2 => Mode::Stereo,
        n => {
            return Err(Error::invalid(format!(
                "oxideav-mp1: unsupported channel count {n}",
            )))
        }
    };
    // Whether the requested sample rate is an MPEG-2 LSF rate
    // (13818-3 §2.4.2.3): 16 / 22.05 / 24 kHz. The LSF Layer I
    // ladder tops out at 256 kbit/s vs MPEG-1's 448, and the default
    // bitrate is scaled down to match the per-rate audio bandwidth
    // (LSF Fs ≈ 1/2 MPEG-1 Fs, so a per-bit-budget midpoint of half
    // the MPEG-1 default keeps allocation density comparable).
    let is_lsf = matches!(sample_rate, 16_000 | 22_050 | 24_000);
    // Pick a default bitrate when the caller didn't specify one. The
    // ladders differ by layer and by ID bit (§2.4.2.3 has four
    // distinct ladders: MPEG-1 Layer I, MPEG-1 Layer II, LSF Layer I,
    // LSF Layer II). The mono / stereo midpoints below come straight
    // from the respective ladder so the default lands on a valid
    // index without requiring caller intervention.
    let bitrate_kbps = match (params.bit_rate, layer) {
        (Some(bps), _) => (bps / 1000) as u16,
        (None, LayerSelect::LayerI) if is_lsf && channels == 1 => 96,
        (None, LayerSelect::LayerI) if is_lsf => 128,
        (None, LayerSelect::LayerI) if channels == 1 => 192,
        (None, LayerSelect::LayerI) => 256,
        // Layer II defaults: 13818-3 §2.4.2.3 LSF Layer II ladder is
        // {8,16,24,32,40,48,56,64,80,96,112,128,144,160}; pick 64
        // mono / 96 stereo (LSF mono fits comfortably in 64 kbit/s
        // and the stereo midpoint 96 keeps allocation density in the
        // same neighbourhood as the LSF Layer I default).
        (None, LayerSelect::LayerII) if is_lsf && channels == 1 => 64,
        (None, LayerSelect::LayerII) if is_lsf => 96,
        // 11172-3 §2.4.2.3 MPEG-1 Layer II ladder is
        // {32,48,56,64,80,96,112,128,160,192,224,256,320,384}; pick
        // 128 mono / 192 stereo as comfortable defaults.
        (None, LayerSelect::LayerII) if channels == 1 => 128,
        (None, LayerSelect::LayerII) => 192,
    };
    let bitrate = Bitrate::Fixed(bitrate_kbps);
    let enc_params = EncodeParams::new(bitrate, sample_rate, mode)
        .with_emit_crc(emit_crc)
        .with_layer(layer);
    let mut out_params = CodecParameters::audio(CodecId::new(CODEC_ID));
    out_params.sample_rate = Some(sample_rate);
    out_params.channels = Some(channels);
    out_params.sample_format = Some(SampleFormat::S16);
    out_params.bit_rate = Some(bitrate_kbps as u64 * 1000);
    out_params.tag = Some(CodecTag::wave_format(0x0050));
    Ok(Box::new(Mp1Encoder::new(
        CodecId::new(CODEC_ID),
        enc_params,
        out_params,
    )))
}

/// A frame-to-packet MPEG-1 Audio Layer I / Layer II encoder.
///
/// Wraps either [`Mp1FrameEncoder`] (Layer I, 384 PCM samples/channel
/// per frame) or [`Mp1Layer2FrameEncoder`] (Layer II, 1152
/// samples/channel) — each of which owns the per-channel analysis
/// history — and adapts the chosen inner encoder to the
/// [`oxideav_core::Encoder`] trait: accept an [`AudioFrame`] of
/// interleaved S16 (or the same-shape S16P planar layout), produce one
/// compressed Layer I/II packet per call.
///
/// The layer is selected at construction time via
/// [`EncodeParams::layer`] (Layer I is the default for byte-for-byte
/// compatibility with the encoder's pre-switch behaviour). Each input
/// frame must carry exactly **384** (Layer I, §2.4.2.1) or **1152**
/// (Layer II, §2.4.2.1) samples per channel; partial frames on flush
/// return [`Error::Eof`].
#[derive(Debug)]
pub struct Mp1Encoder {
    codec_id: CodecId,
    inner: Mp1InnerEncoder,
    /// The output stream parameters muxers will read back via
    /// [`Encoder::output_params`].
    output: CodecParameters,
    pending_frame: Option<AudioFrame>,
    pending_pkt: Option<Packet>,
    eof: bool,
}

/// Layer-dispatched inner encoder: each variant owns its own per-channel
/// analysis history (see [`Mp1FrameEncoder`] / [`Mp1Layer2FrameEncoder`]).
#[derive(Debug)]
enum Mp1InnerEncoder {
    /// Layer I (§2.4.1.5): 384 samples/channel per frame.
    LayerI(Mp1FrameEncoder),
    /// Layer II (§2.4.1.6): 1152 samples/channel per frame.
    LayerII(Mp1Layer2FrameEncoder),
}

impl Mp1InnerEncoder {
    fn channels(&self) -> usize {
        match self {
            Mp1InnerEncoder::LayerI(e) => e.channels(),
            Mp1InnerEncoder::LayerII(e) => e.channels(),
        }
    }

    /// Samples per channel that one [`Mp1Encoder::send_frame`] must
    /// carry: 384 for Layer I, 1152 for Layer II (§2.4.2.1).
    fn samples_per_frame(&self) -> usize {
        match self {
            Mp1InnerEncoder::LayerI(_) => SAMPLES_PER_SUBBAND * SUBBANDS,
            Mp1InnerEncoder::LayerII(_) => LAYER2_SAMPLES_PER_FRAME,
        }
    }

    /// Encode one frame's worth of interleaved PCM, surfacing the inner
    /// encoder's typed error wrapped in [`Error::other`] (matching the
    /// pre-switch Layer I path's surfacing of [`crate::encode::EncodeError`]).
    fn encode_frame(&mut self, pcm: &[f64]) -> Result<Vec<u8>> {
        match self {
            Mp1InnerEncoder::LayerI(e) => e
                .encode_frame(pcm)
                .map_err(|err| Error::other(format!("oxideav-mp1: encode: {err}"))),
            Mp1InnerEncoder::LayerII(e) => e
                .encode_frame(pcm)
                .map_err(|err| Error::other(format!("oxideav-mp1: encode: {err}"))),
        }
    }
}

impl Mp1Encoder {
    fn new(codec_id: CodecId, params: EncodeParams, output: CodecParameters) -> Mp1Encoder {
        // §2.4.1.5 / §2.4.1.6 dispatch: the layer field of `params`
        // chooses which inner encoder owns the per-channel analysis
        // history. The Layer II branch translates the shared
        // [`EncodeParams`] (bitrate / sampling frequency / channel mode
        // / optional CRC) into the [`Layer2HeaderParams`] shape that
        // [`Mp1Layer2FrameEncoder`] consumes; all other header fields
        // default to their [`Layer2HeaderParams::new`] values
        // (no mode_extension, no padding, no private/copyright,
        // original = 1, Emphasis::None).
        let inner = match params.layer {
            LayerSelect::LayerI => Mp1InnerEncoder::LayerI(Mp1FrameEncoder::new(params)),
            LayerSelect::LayerII => {
                let bitrate_kbps = match params.bitrate {
                    Bitrate::Fixed(k) => k,
                    // Free / forbidden bitrates surface their typed
                    // rejection from `encode_layer2_frame` on first
                    // encode, matching the Layer I path's lazy
                    // validation contract.
                    _ => 0,
                };
                let mut hp =
                    Layer2HeaderParams::new(params.sampling_frequency, bitrate_kbps, params.mode);
                hp.has_crc = params.emit_crc;
                // Forward the psychoacoustic + VBR settings to the Layer
                // II frame encoder so the registry-facing trait-object
                // path matches the direct `Mp1Layer2FrameEncoder` API.
                let mut l2 = Mp1Layer2FrameEncoder::new(hp);
                if params.psychoacoustic {
                    l2 = l2
                        .with_psy_model(params.psy_model)
                        .with_psychoacoustic(true);
                    if let Some(target) = params.vbr_target_mnr_db {
                        l2 = l2.with_vbr(target);
                    }
                }
                Mp1InnerEncoder::LayerII(l2)
            }
        };
        Mp1Encoder {
            codec_id,
            inner,
            output,
            pending_frame: None,
            pending_pkt: None,
            eof: false,
        }
    }

    /// Decode the AudioFrame's raw bytes into a per-channel-interleaved
    /// `f64` PCM vector in `[-1, 1)`, supporting both interleaved S16
    /// (`data.len() == 1`) and planar S16P (`data.len() == nch`).
    fn frame_to_pcm(&self, frame: &AudioFrame) -> Result<Vec<f64>> {
        let nch = self.inner.channels();
        let samples = frame.samples as usize;
        let total = samples * nch;
        let mut pcm = vec![0.0f64; total];
        if frame.data.len() == 1 {
            // Interleaved S16.
            let bytes = &frame.data[0];
            if bytes.len() != total * 2 {
                return Err(Error::invalid(format!(
                    "oxideav-mp1: frame data len {} != {} expected",
                    bytes.len(),
                    total * 2
                )));
            }
            for (i, p) in pcm.iter_mut().enumerate() {
                let lo = bytes[i * 2];
                let hi = bytes[i * 2 + 1];
                let v = i16::from_le_bytes([lo, hi]);
                *p = v as f64 / 32768.0;
            }
        } else if frame.data.len() == nch {
            // Planar S16P: one plane per channel.
            for (ch, plane) in frame.data.iter().enumerate() {
                if plane.len() != samples * 2 {
                    return Err(Error::invalid(format!(
                        "oxideav-mp1: plane {ch} len {} != {}",
                        plane.len(),
                        samples * 2
                    )));
                }
                for s in 0..samples {
                    let v = i16::from_le_bytes([plane[s * 2], plane[s * 2 + 1]]);
                    pcm[s * nch + ch] = v as f64 / 32768.0;
                }
            }
        } else {
            return Err(Error::invalid(format!(
                "oxideav-mp1: expected 1 or {nch} planes, got {}",
                frame.data.len()
            )));
        }
        Ok(pcm)
    }
}

impl Encoder for Mp1Encoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.output
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        if self.pending_pkt.is_some() || self.pending_frame.is_some() {
            return Err(Error::other(
                "oxideav-mp1: call receive_packet before sending another frame",
            ));
        }
        let Frame::Audio(a) = frame else {
            return Err(Error::invalid("oxideav-mp1: encoder requires audio frame"));
        };
        // §2.4.2.1 per-frame sample count: 384 for Layer I, 1152 for
        // Layer II. The inner encoder reports the value that matches
        // whichever layer was selected at construction time.
        let expected = self.inner.samples_per_frame() as u32;
        if a.samples != expected {
            return Err(Error::invalid(format!(
                "oxideav-mp1: frame must carry {expected} samples/channel, got {}",
                a.samples
            )));
        }
        self.pending_frame = Some(a.clone());
        Ok(())
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        if let Some(p) = self.pending_pkt.take() {
            return Ok(p);
        }
        let Some(frame) = self.pending_frame.take() else {
            return if self.eof {
                Err(Error::Eof)
            } else {
                Err(Error::NeedMore)
            };
        };
        let pcm = self.frame_to_pcm(&frame)?;
        let bytes = self.inner.encode_frame(&pcm)?;
        let sr = self.output.sample_rate.unwrap_or(48_000);
        let mut pkt = Packet::new(0, TimeBase::new(1, sr as i64), bytes);
        if let Some(pts) = frame.pts {
            pkt.pts = Some(pts);
        }
        Ok(pkt)
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        self.pending_frame = None;
        Ok(())
    }
}

/// Install the MPEG-1 Audio Layer I codec into `reg` (decoder and
/// encoder).
///
/// Claims the WAVE format tag `0x0050` (MPEG-1 Audio, Layer I/II) and
/// the Matroska codec id `A_MPEG/L1` so containers can route Layer I
/// streams here. The decoder emits interleaved S16 PCM; the encoder
/// accepts the same shape and produces one Layer I packet per
/// 384-sample frame.
pub fn register_codecs(reg: &mut CodecRegistry) {
    let info = CodecInfo::new(CodecId::new(CODEC_ID))
        .capabilities(
            CodecCapabilities::audio("mp1")
                .with_decode()
                .with_encode()
                .with_lossy(true),
        )
        .decoder(make_decoder)
        .encoder(make_encoder)
        .tags([
            CodecTag::wave_format(0x0050),
            CodecTag::matroska("A_MPEG/L1"),
        ]);
    reg.register(info);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::Mode;
    use oxideav_core::TimeBase;

    /// Append `n` bits (`val`, MSB-first) to a growing bit buffer.
    struct BitWriter {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }
    impl BitWriter {
        fn new() -> Self {
            BitWriter {
                bytes: Vec::new(),
                acc: 0,
                nbits: 0,
            }
        }
        fn put(&mut self, val: u32, n: u8) {
            for i in (0..n).rev() {
                let b = (val >> i) & 1;
                self.acc = (self.acc << 1) | b;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }
        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.acc <<= 8 - self.nbits;
                self.bytes.push(self.acc as u8);
            }
            self.bytes
        }
    }

    /// Build a complete, self-consistent mono Layer I frame: a header
    /// (no CRC) followed by audio data that allocates subband 0 only
    /// (nb=4), with a chosen scalefactor and 12 sample codes.
    fn build_mono_frame(scf_index: u32, sample_codes: &[u32; 12]) -> Vec<u8> {
        // Header: MPEG-1, Layer I (0b11), protection=1 (no CRC),
        // bitrate index 0b1000 (256 kbit/s), 48 kHz (0b01), no padding,
        // single_channel (0b11), no ext, original. Zero-valued fields
        // (padding/private/mode_ext/copyright/emphasis) contribute
        // nothing and are omitted from the OR.
        let word: u32 = (0xFFF << 20)
            | (1 << 19)        // ID = MPEG
            | (0b11 << 17)     // layer I
            | (1 << 16)        // protection = 1 (no CRC)
            | (0b1000 << 12)   // bitrate index
            | (0b01 << 10)     // 48 kHz
            | (0b11 << 6)      // single_channel
            | (1 << 2); // original
        let mut frame = word.to_be_bytes().to_vec();

        let mut bw = BitWriter::new();
        // allocation: sb0 = 0b0011 (nb = 4); sb1..31 = 0.
        bw.put(0b0011, 4);
        for _ in 1..32 {
            bw.put(0b0000, 4);
        }
        // scalefactor for sb0 (6 bits).
        bw.put(scf_index, 6);
        // 12 samples for sb0, 4 bits each.
        for &c in sample_codes {
            bw.put(c, 4);
        }
        frame.extend_from_slice(&bw.finish());
        frame
    }

    #[test]
    fn register_installs_decoder() {
        let mut reg = CodecRegistry::new();
        register_codecs(&mut reg);
        assert!(reg.has_decoder(&CodecId::new("mp1")));
        // tags resolve.
        let t = CodecTag::wave_format(0x0050);
        assert_eq!(
            reg.resolve_tag_ref(&oxideav_core::ProbeContext::new(&t))
                .map(|c| c.as_str()),
            Some("mp1"),
        );
    }

    #[test]
    fn full_frame_to_pcm_smoke() {
        // Decode a hand-built mono frame end to end and check the PCM
        // frame shape + that silence in produces silence out on the
        // first frame's leading samples (the synthesis filter's V
        // history is zero, so the impulse response ramps in).
        let codes = [8u32, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8]; // mid-code
        let frame = build_mono_frame(20, &codes);

        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
        dec.send_packet(&pkt).unwrap();
        let f = dec.receive_frame().unwrap();
        let Frame::Audio(a) = f else {
            panic!("expected audio frame");
        };
        // 384 samples per channel, mono -> 384 samples, S16 -> 768 bytes.
        assert_eq!(a.samples, 384);
        assert_eq!(a.data.len(), 1);
        assert_eq!(a.data[0].len(), 384 * 2);
    }

    #[test]
    fn decode_produces_nonzero_for_nonzero_input() {
        // Use the extreme sample code so the requantized value is large;
        // after a few frames the synthesis output must be clearly
        // non-zero (the filter has ramped up).
        let codes = [15u32; 12]; // most-positive nb=4 code
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();

        let mut saw_nonzero = false;
        for _ in 0..4 {
            let frame = build_mono_frame(0, &codes); // scf index 0 -> factor 2.0
            let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            if a.data[0].iter().any(|&b| b != 0) {
                saw_nonzero = true;
            }
        }
        assert!(
            saw_nonzero,
            "decoder produced only silence for a loud input"
        );
    }

    #[test]
    fn reset_clears_filter_history() {
        let codes = [15u32; 12];
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        // Drive a few loud frames.
        for _ in 0..3 {
            let frame = build_mono_frame(0, &codes);
            let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
            dec.send_packet(&pkt).unwrap();
            let _ = dec.receive_frame().unwrap();
        }
        dec.reset().unwrap();
        // After reset, a truly-silent frame (all subbands unallocated)
        // must decode to exact silence — proving no overlap-add history
        // lingered. Build it as header + 16 zero bytes (32 allocation
        // nibbles of 0, hence no scalefactors and no samples).
        let word: u32 = (0xFFF << 20)
            | (1 << 19)
            | (0b11 << 17)
            | (1 << 16)
            | (0b1000 << 12)
            | (0b01 << 10)
            | (0b11 << 6)
            | (1 << 2);
        let mut frame = word.to_be_bytes().to_vec();
        frame.extend_from_slice(&[0u8; 16]);
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert!(
            a.data[0].iter().all(|&b| b == 0),
            "silence after reset still carried filter history"
        );
    }

    #[test]
    fn rejects_packet_without_sync() {
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), vec![0u8; 64]);
        dec.send_packet(&pkt).unwrap();
        assert!(dec.receive_frame().is_err());
    }

    #[test]
    fn empty_packet_yields_empty_frame() {
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), Vec::new());
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert_eq!(a.samples, 0);
    }

    /// Build a CRC-protected (protection_bit == 0) mono Layer I frame:
    /// header, then the two CRC bytes, then audio data that allocates
    /// subband 0 (nb=4) with the given scalefactor and 12 sample codes.
    /// When `corrupt_crc` is set the stored CRC word is deliberately
    /// wrong so the decoder's §2.4.3.1 concealment path fires.
    fn build_protected_mono_frame(
        scf_index: u32,
        sample_codes: &[u32; 12],
        corrupt_crc: bool,
    ) -> Vec<u8> {
        // protection = 0 -> CRC present (bit 16 cleared vs build_mono).
        let word: u32 = (0xFFF << 20)
            | (1 << 19)        // ID = MPEG
            | (0b11 << 17)     // layer I
            // protection bit (16) = 0 -> CRC present
            | (0b1000 << 12)   // bitrate index (256 kbit/s)
            | (0b01 << 10)     // 48 kHz
            | (0b11 << 6)      // single_channel
            | (1 << 2); // original
        let header = word.to_be_bytes();

        // Build the audio-data bytes (the bit-allocation field is the
        // CRC-protected part).
        let mut bw = BitWriter::new();
        bw.put(0b0011, 4); // sb0 nb=4
        for _ in 1..32 {
            bw.put(0b0000, 4);
        }
        bw.put(scf_index, 6);
        for &c in sample_codes {
            bw.put(c, 4);
        }
        let audio = bw.finish();

        // Compute the §2.4.3.1 CRC over header bits 16..31 + the
        // allocation field via the public encoder-side helper.
        let h = FrameHeader::parse(&header).unwrap();
        let mut crc = h.compute_crc(&header, &audio).unwrap();
        if corrupt_crc {
            crc ^= 0xFFFF;
        }

        let mut frame = header.to_vec();
        frame.extend_from_slice(&crc.to_be_bytes());
        frame.extend_from_slice(&audio);
        frame
    }

    #[test]
    fn crc_protected_frame_decodes_audio_on_match() {
        // A correctly-CRC'd protected frame decodes its audio (loud
        // input -> non-silent output once the filter ramps).
        let codes = [15u32; 12];
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        let mut saw_nonzero = false;
        for _ in 0..4 {
            let frame = build_protected_mono_frame(0, &codes, false);
            // Confirm we really built a protected frame.
            let h = FrameHeader::parse(&frame).unwrap();
            assert!(h.has_crc());
            let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            assert_eq!(a.samples, 384);
            if a.data[0].iter().any(|&b| b != 0) {
                saw_nonzero = true;
            }
        }
        assert!(saw_nonzero, "valid CRC frame produced only silence");
    }

    #[test]
    fn crc_mismatch_is_concealed_as_mute() {
        // A frame whose stored CRC is wrong must be muted (§2.4.3.1
        // concealment): the decoder emits a correctly-shaped frame, and
        // a sequence of corrupt frames with no prior history must be
        // exact silence (the filter rings out zeros).
        let codes = [15u32; 12];
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        for _ in 0..6 {
            let frame = build_protected_mono_frame(0, &codes, true);
            let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            assert_eq!(a.samples, 384, "concealment must keep frame shape");
            assert!(
                a.data[0].iter().all(|&b| b == 0),
                "muted concealment frame must be exact silence with no prior history"
            );
        }
    }

    #[test]
    fn crc_mismatch_after_audio_rings_out_history() {
        // Decode a few loud valid frames, then a corrupt one: the muted
        // frame still advances the overlap-add history (the previous
        // tail rings out) rather than being zeroed hard — so the muted
        // frame is allowed to be non-silent in its leading samples but
        // the decoder must not error and must keep the frame shape.
        let codes = [15u32; 12];
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        for _ in 0..3 {
            let f = build_protected_mono_frame(0, &codes, false);
            let pkt = Packet::new(0, TimeBase::new(1, 48_000), f);
            dec.send_packet(&pkt).unwrap();
            let _ = dec.receive_frame().unwrap();
        }
        let f = build_protected_mono_frame(0, &codes, true);
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), f);
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert_eq!(a.samples, 384);
        assert_eq!(a.data[0].len(), 384 * 2);
    }

    /// Collect a decoder's PCM bytes for one packet (decode + extract).
    fn decode_pcm(dec: &mut Box<dyn Decoder>, frame: Vec<u8>) -> Vec<u8> {
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio");
        };
        a.data[0].clone()
    }

    #[test]
    fn concealment_defaults_to_mute() {
        let dec = Mp1Decoder::new(CodecId::new("mp1"));
        assert_eq!(dec.concealment(), ConcealmentMode::Mute);
        let dec = dec.with_concealment(ConcealmentMode::RepeatPrevious);
        assert_eq!(dec.concealment(), ConcealmentMode::RepeatPrevious);
    }

    /// With [`ConcealmentMode::RepeatPrevious`], a CRC-failing frame
    /// must reproduce exactly the PCM the *previous good frame's*
    /// subband samples produce when run through the (now-advanced)
    /// filterbank — i.e. it equals decoding a fresh, valid copy of that
    /// same frame on a decoder in the identical filter state.
    #[test]
    fn repeat_previous_repeats_last_good_subbands() {
        let codes = [12u32; 12];
        // Reference decoder: decode a good frame, then decode a *second*
        // identical good frame. The second frame's PCM is exactly what
        // "repeating the previous frame" should produce, because the
        // subband samples are identical and the filter state coming in
        // is identical.
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut reference = make_decoder(&params).unwrap();
        let _ = decode_pcm(&mut reference, build_protected_mono_frame(7, &codes, false));
        let want = decode_pcm(&mut reference, build_protected_mono_frame(7, &codes, false));

        // Subject decoder: same good frame, then a CRC-corrupt frame
        // with RepeatPrevious. The concealed frame must equal `want`.
        let mut subject = Box::new(
            Mp1Decoder::new(CodecId::new("mp1")).with_concealment(ConcealmentMode::RepeatPrevious),
        ) as Box<dyn Decoder>;
        let _ = decode_pcm(&mut subject, build_protected_mono_frame(7, &codes, false));
        let got = decode_pcm(&mut subject, build_protected_mono_frame(7, &codes, true));
        assert_eq!(
            got, want,
            "RepeatPrevious must reproduce the last good frame"
        );
        // And it must NOT be silence (the previous frame was loud).
        assert!(got.iter().any(|&b| b != 0), "repeat produced only silence");
    }

    /// A run of corrupt frames under RepeatPrevious repeats the *last
    /// good* frame each time (a concealed frame is never stored as the
    /// new "previous"), so the second concealed frame differs from the
    /// first only through the advancing filterbank tail — never by
    /// chaining a repeat of a repeat. We verify both concealed frames
    /// match the corresponding reference frames decoded from the same
    /// good subbands.
    #[test]
    fn repeat_previous_does_not_chain_repeats() {
        let codes = [14u32; 12];
        let params = CodecParameters::audio(CodecId::new("mp1"));
        // Reference: one good frame, then two more identical good frames.
        let mut reference = make_decoder(&params).unwrap();
        let _ = decode_pcm(&mut reference, build_protected_mono_frame(3, &codes, false));
        let want1 = decode_pcm(&mut reference, build_protected_mono_frame(3, &codes, false));
        let want2 = decode_pcm(&mut reference, build_protected_mono_frame(3, &codes, false));

        let mut subject = Box::new(
            Mp1Decoder::new(CodecId::new("mp1")).with_concealment(ConcealmentMode::RepeatPrevious),
        ) as Box<dyn Decoder>;
        let _ = decode_pcm(&mut subject, build_protected_mono_frame(3, &codes, false));
        let got1 = decode_pcm(&mut subject, build_protected_mono_frame(3, &codes, true));
        let got2 = decode_pcm(&mut subject, build_protected_mono_frame(3, &codes, true));
        assert_eq!(got1, want1);
        assert_eq!(got2, want2);
    }

    /// RepeatPrevious with no prior good frame (the first frame is
    /// corrupt) falls back to muting: exact silence with no history.
    #[test]
    fn repeat_previous_first_frame_falls_back_to_mute() {
        let codes = [15u32; 12];
        let mut dec = Box::new(
            Mp1Decoder::new(CodecId::new("mp1")).with_concealment(ConcealmentMode::RepeatPrevious),
        ) as Box<dyn Decoder>;
        let pcm = decode_pcm(&mut dec, build_protected_mono_frame(0, &codes, true));
        assert!(
            pcm.iter().all(|&b| b == 0),
            "first-frame RepeatPrevious must mute (no previous frame)"
        );
    }

    /// `set_concealment` switches the strategy at runtime. Start in
    /// Mute (the default), decode a good frame, then switch to
    /// RepeatPrevious and confirm the next corrupt frame is non-silent
    /// (it repeated the previous good loud frame, not muted).
    #[test]
    fn set_concealment_switches_at_runtime() {
        let codes = [13u32; 12];

        // Subject decoder: starts in the Mute default, decodes a good
        // loud frame, then switches to RepeatPrevious before the corrupt
        // frame arrives. The corrupt frame must repeat the loud frame
        // (non-silent).
        let mut owned = Mp1Decoder::new(CodecId::new("mp1"));
        assert_eq!(owned.concealment(), ConcealmentMode::Mute);
        {
            let pkt = Packet::new(
                0,
                TimeBase::new(1, 48_000),
                build_protected_mono_frame(2, &codes, false),
            );
            owned.send_packet(&pkt).unwrap();
            let _ = owned.receive_frame().unwrap();
        }
        owned.set_concealment(ConcealmentMode::RepeatPrevious);
        assert_eq!(owned.concealment(), ConcealmentMode::RepeatPrevious);
        let mut subject = Box::new(owned) as Box<dyn Decoder>;
        let pcm = decode_pcm(&mut subject, build_protected_mono_frame(2, &codes, true));
        assert!(
            pcm.iter().any(|&b| b != 0),
            "after set_concealment(RepeatPrevious) the corrupt frame should repeat the loud previous frame"
        );

        // Contrast decoder: same good frame, but kept on the Mute
        // default. Its concealed frame must differ from the repeated one
        // (it rings the tail out rather than re-driving the loud band).
        let mut mute = make_decoder(&CodecParameters::audio(CodecId::new("mp1"))).unwrap();
        let _ = decode_pcm(&mut mute, build_protected_mono_frame(2, &codes, false));
        let mute_pcm = decode_pcm(&mut mute, build_protected_mono_frame(2, &codes, true));
        assert_ne!(
            pcm, mute_pcm,
            "RepeatPrevious and Mute must produce different concealment PCM"
        );
    }

    /// A concealed frame must keep the standard 384-sample shape under
    /// RepeatPrevious as well.
    #[test]
    fn repeat_previous_keeps_frame_shape() {
        let codes = [10u32; 12];
        let mut dec = Box::new(
            Mp1Decoder::new(CodecId::new("mp1")).with_concealment(ConcealmentMode::RepeatPrevious),
        ) as Box<dyn Decoder>;
        let _ = decode_pcm(&mut dec, build_protected_mono_frame(5, &codes, false));
        let pkt = Packet::new(
            0,
            TimeBase::new(1, 48_000),
            build_protected_mono_frame(5, &codes, true),
        );
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert_eq!(a.samples, 384);
        assert_eq!(a.data[0].len(), 384 * 2);
    }

    /// After `reset`, RepeatPrevious has no "previous frame" to repeat,
    /// so the first corrupt frame mutes to exact silence.
    #[test]
    fn reset_clears_repeat_history() {
        let codes = [15u32; 12];
        let mut owned = Mp1Decoder::new(CodecId::new("mp1"));
        owned.set_concealment(ConcealmentMode::RepeatPrevious);
        // A good frame establishes a "previous frame".
        {
            let pkt = Packet::new(
                0,
                TimeBase::new(1, 48_000),
                build_protected_mono_frame(0, &codes, false),
            );
            owned.send_packet(&pkt).unwrap();
            let _ = owned.receive_frame().unwrap();
        }
        owned.reset().unwrap();
        // Reset must keep the *mode* but drop the previous-frame state.
        assert_eq!(owned.concealment(), ConcealmentMode::RepeatPrevious);
        let mut boxed = Box::new(owned) as Box<dyn Decoder>;
        let pcm = decode_pcm(&mut boxed, build_protected_mono_frame(0, &codes, true));
        assert!(
            pcm.iter().all(|&b| b == 0),
            "after reset, RepeatPrevious must mute (history dropped)"
        );
    }

    // ---- Layer II CRC concealment routing ------------------------

    /// Build a protected mono Layer II frame at 64 kbit/s / 44.1 kHz
    /// (selects Table 3-B.2a, sblimit = 27, no padding), with the
    /// given allocation-of-sb0 (`alloc_sb0`, nbal=4 column, valid
    /// values 0..=14) and the standard "all subsequent subbands
    /// unallocated" pattern. The CRC word is computed correctly when
    /// `corrupt_crc` is false; otherwise it is XOR-flipped so the
    /// decoder's §2.4.3.1 concealment path fires.
    fn build_protected_l2_mono_frame(alloc_sb0: u32, corrupt_crc: bool) -> Vec<u8> {
        use crate::decode_layer2::compute_layer2_crc;
        use crate::tables_layer2::layer2_bit_allocation_table;
        // Layer II (0b10), MPEG-1, protection=0 -> CRC present.
        // bitrate_index 0b0100 -> 64 kbit/s; sampling 0b00 -> 44.1 kHz;
        // single_channel. frame_length = floor(144 * 64000 / 44100)
        // = 208 bytes.
        let word: u32 = (0xFFF << 20)
            | (1 << 19)        // ID = MPEG-1
            | (0b10 << 17)     // layer II
            // protection bit (16) = 0 -> CRC present
            | (0b0100 << 12)   // bitrate index -> 64 kbit/s
            // sampling 0b00 (44.1 kHz) — implicit zero; clippy identity_op.
            | (0b11 << 6)      // single_channel
            | (1 << 2); // original
        let header = word.to_be_bytes();
        let h = FrameHeader::parse(&header).unwrap();
        let table = layer2_bit_allocation_table(&h);

        // Allocation field: sb0 takes the requested alloc, all higher
        // subbands stay at 0 (sums to 88 bits = 11 bytes for B.2a mono).
        let mut bw = BitWriter::new();
        bw.put(alloc_sb0, 4);
        for sb in 1..table.sblimit() {
            bw.put(0, table.nbal(sb));
        }
        // scfsi for the one allocated subband (2 bits, selector 0b00).
        if alloc_sb0 != 0 {
            bw.put(0b00, 2);
            // Three scalefactor indices (6 bits each) — all zeros are
            // valid (table index 0, multiplier 2.0).
            for _ in 0..3 {
                bw.put(0, 6);
            }
            // 12 syntax-granules: one codeword each. For sb0 alloc 1
            // (B.2a row 0..2 column 0): nlevels=3, grouped, 5-bit
            // codeword. Use all-zero codes (valid).
            for _ in 0..12 {
                bw.put(0, 5);
            }
        }
        let body = bw.finish();

        // Compute CRC over header bits 16..31 + allocation + scfsi.
        let mut crc = compute_layer2_crc(&h, &header, &body).expect("layer2 crc");
        if corrupt_crc {
            crc ^= 0xFFFF;
        }

        // Frame: header + CRC word (MSB-first) + body, padded to the
        // header's reported frame_length.
        let mut frame = header.to_vec();
        frame.extend_from_slice(&crc.to_be_bytes());
        frame.extend_from_slice(&body);
        let target_len = h.frame_length_bytes().unwrap() as usize;
        if frame.len() < target_len {
            frame.resize(target_len, 0);
        }
        frame
    }

    #[test]
    fn layer2_crc_match_decodes_to_1152_samples() {
        // A valid Layer II protected frame must produce a 1152-sample
        // PCM frame — same shape as an unprotected Layer II frame.
        let frame = build_protected_l2_mono_frame(1, /*corrupt=*/ false);
        // Confirm the header reports a CRC-protected frame.
        let h = FrameHeader::parse(&frame).unwrap();
        assert_eq!(h.layer, Layer::II);
        assert!(h.has_crc());

        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), frame);
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio frame");
        };
        // §2.4.2.1: 1152 samples per channel per Layer II frame.
        assert_eq!(a.samples, 1152);
        assert_eq!(a.data[0].len(), 1152 * 2);
    }

    #[test]
    fn layer2_crc_mismatch_is_concealed_as_mute() {
        // A protected Layer II frame whose CRC has been flipped must
        // be concealed (default ConcealmentMode::Mute): the decoder
        // produces a correctly-shaped 1152-sample PCM frame whose
        // contents are exact silence on the very first frame (the
        // filterbank's `V` history is zero, so the muting concealment
        // emits silence + a tail that's also still silence).
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        for _ in 0..3 {
            let frame = build_protected_l2_mono_frame(1, /*corrupt=*/ true);
            let pkt = Packet::new(0, TimeBase::new(1, 44_100), frame);
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            assert_eq!(
                a.samples, 1152,
                "Layer II concealment must keep frame shape"
            );
            assert!(
                a.data[0].iter().all(|&b| b == 0),
                "muted Layer II concealment with no prior history must be exact silence"
            );
        }
    }

    #[test]
    fn layer2_crc_mismatch_repeats_last_good_subbands() {
        // With ConcealmentMode::RepeatPrevious, a CRC-failing Layer II
        // frame must reproduce exactly the PCM the previous good
        // Layer II frame's subband samples would produce on a decoder
        // in the identical (now-advanced) filterbank state — i.e. it
        // matches a fresh decoder consuming a second copy of the good
        // frame.
        let reference_params = CodecParameters::audio(CodecId::new("mp1"));
        let mut reference = make_decoder(&reference_params).unwrap();
        let good_frame = build_protected_l2_mono_frame(2, /*corrupt=*/ false);
        let pkt1 = Packet::new(0, TimeBase::new(1, 44_100), good_frame.clone());
        reference.send_packet(&pkt1).unwrap();
        let _ = reference.receive_frame().unwrap();
        let pkt2 = Packet::new(0, TimeBase::new(1, 44_100), good_frame.clone());
        reference.send_packet(&pkt2).unwrap();
        let Frame::Audio(want) = reference.receive_frame().unwrap() else {
            panic!("audio");
        };

        let mut subject = Box::new(
            Mp1Decoder::new(CodecId::new("mp1")).with_concealment(ConcealmentMode::RepeatPrevious),
        ) as Box<dyn Decoder>;
        let pkt_a = Packet::new(0, TimeBase::new(1, 44_100), good_frame.clone());
        subject.send_packet(&pkt_a).unwrap();
        let _ = subject.receive_frame().unwrap();
        let corrupt = build_protected_l2_mono_frame(2, /*corrupt=*/ true);
        let pkt_b = Packet::new(0, TimeBase::new(1, 44_100), corrupt);
        subject.send_packet(&pkt_b).unwrap();
        let Frame::Audio(got) = subject.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert_eq!(
            got.samples, 1152,
            "Layer II RepeatPrevious must keep the 1152-sample shape"
        );
        assert_eq!(
            got.data[0], want.data[0],
            "Layer II RepeatPrevious must reproduce the last good frame's PCM"
        );
    }

    #[test]
    fn layer2_crc_mismatch_first_frame_repeat_falls_back_to_mute() {
        // RepeatPrevious with no prior good Layer II frame: the first
        // CRC-failing frame falls back to muting (exact silence).
        let mut dec = Box::new(
            Mp1Decoder::new(CodecId::new("mp1")).with_concealment(ConcealmentMode::RepeatPrevious),
        ) as Box<dyn Decoder>;
        let frame = build_protected_l2_mono_frame(1, /*corrupt=*/ true);
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), frame);
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert_eq!(a.samples, 1152);
        assert!(
            a.data[0].iter().all(|&b| b == 0),
            "first-frame Layer II RepeatPrevious must mute (no previous frame to repeat)"
        );
    }

    #[test]
    fn layer2_reset_clears_repeat_history() {
        // After reset, RepeatPrevious has no Layer II "previous frame"
        // to repeat, so the next corrupt frame mutes to exact silence.
        let mut owned = Mp1Decoder::new(CodecId::new("mp1"));
        owned.set_concealment(ConcealmentMode::RepeatPrevious);
        {
            let f = build_protected_l2_mono_frame(1, false);
            let pkt = Packet::new(0, TimeBase::new(1, 44_100), f);
            owned.send_packet(&pkt).unwrap();
            let _ = owned.receive_frame().unwrap();
        }
        owned.reset().unwrap();
        assert_eq!(owned.concealment(), ConcealmentMode::RepeatPrevious);
        let mut boxed = Box::new(owned) as Box<dyn Decoder>;
        let f = build_protected_l2_mono_frame(1, true);
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), f);
        boxed.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = boxed.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert!(
            a.data[0].iter().all(|&b| b == 0),
            "after reset, Layer II RepeatPrevious must mute (history dropped)"
        );
    }

    #[test]
    fn stereo_frame_interleaves_two_channels() {
        // Build a plain-stereo frame, sb0 allocated nb=4 on both
        // channels with different sample codes, confirm 384*2 samples
        // and interleaving.
        // mode = stereo (0b00) contributes nothing to the OR, so it's
        // left implicit here.
        let word: u32 = (0xFFF << 20)
            | (1 << 19)
            | (0b11 << 17)
            | (1 << 16)
            | (0b1000 << 12)
            | (0b01 << 10)
            | (1 << 2);
        let mut frame = word.to_be_bytes().to_vec();
        let mut bw = BitWriter::new();
        // alloc: sb0 ch0=0b0011, sb0 ch1=0b0011, then sb1..31 both 0.
        bw.put(0b0011, 4);
        bw.put(0b0011, 4);
        for _ in 1..32 {
            bw.put(0, 4);
            bw.put(0, 4);
        }
        // scalefactors sb0 ch0, sb0 ch1.
        bw.put(10, 6);
        bw.put(10, 6);
        // samples: per slot, ch0 then ch1 (4 bits each).
        for _ in 0..12 {
            bw.put(15, 4);
            bw.put(0, 4);
        }
        frame.extend_from_slice(&bw.finish());

        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).unwrap();
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("audio");
        };
        assert_eq!(a.samples, 384);
        // stereo interleaved -> 384 frames * 2 ch * 2 bytes.
        assert_eq!(a.data[0].len(), 384 * 2 * 2);
        // Mode check: confirm we built a stereo header.
        let h = FrameHeader::parse(&pkt.data).unwrap();
        assert_eq!(h.mode, Mode::Stereo);
    }

    // ---- §2.4.1.4 optional CRC emission (encoder-level) ----------

    /// Build a 1 kHz mono S16 frame (384 samples / channel) at the
    /// given sample rate, in the AudioFrame shape the encoder consumes.
    fn build_audio_frame_mono(sample_rate: u32) -> AudioFrame {
        let n = 384usize;
        let mut bytes = Vec::with_capacity(n * 2);
        for k in 0..n {
            let t = k as f64 / sample_rate as f64;
            let v = (2.0 * std::f64::consts::PI * 1_000.0 * t).sin();
            let s = (v * 16_000.0) as i16;
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        AudioFrame {
            samples: n as u32,
            pts: None,
            data: vec![bytes],
        }
    }

    #[test]
    fn make_encoder_default_emits_protection_bit_1() {
        // The default registry factory must continue to emit
        // `protection_bit == 1` (no CRC) so existing consumers are
        // byte-compatible.
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(48_000);
        params.channels = Some(1);
        let mut enc = make_encoder(&params).unwrap();
        let frame = build_audio_frame_mono(48_000);
        enc.send_frame(&Frame::Audio(frame)).unwrap();
        let pkt = enc.receive_packet().unwrap();
        let h = FrameHeader::parse(&pkt.data).unwrap();
        assert!(!h.has_crc(), "default factory must produce no CRC");
    }

    #[test]
    fn make_encoder_with_crc_emits_verifying_crc() {
        // The opt-in factory writes a CRC that verifies clean against
        // the decoder-side check.
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(48_000);
        params.channels = Some(1);
        let mut enc = make_encoder_with_crc(&params).unwrap();
        let frame = build_audio_frame_mono(48_000);
        enc.send_frame(&Frame::Audio(frame)).unwrap();
        let pkt = enc.receive_packet().unwrap();
        let h = FrameHeader::parse(&pkt.data).unwrap();
        assert!(h.has_crc(), "with-CRC factory must clear protection_bit");
        let status = h
            .verify_crc(&pkt.data[..4], &pkt.data[4..])
            .expect("CRC region present");
        assert!(
            status.is_good(),
            "encoder-emitted CRC failed verification: {status:?}"
        );
    }

    #[test]
    fn make_encoder_with_crc_round_trips_through_decoder() {
        // A full encode-with-CRC -> decode loop reaches PCM cleanly:
        // the §2.4.3.1 CRC matches, the decoder takes the audio data
        // path (not concealment), and the frame shape is preserved.
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(48_000);
        params.channels = Some(1);
        let mut enc = make_encoder_with_crc(&params).unwrap();
        let mut dec = make_decoder(&params).unwrap();
        // Drive several frames so the synthesis filter ramps and we
        // see audio (not just leading-edge silence) on the decode side.
        let mut saw_nonzero = false;
        for _ in 0..6 {
            let frame = build_audio_frame_mono(48_000);
            enc.send_frame(&Frame::Audio(frame)).unwrap();
            let pkt = enc.receive_packet().unwrap();
            // Confirm every produced packet is CRC-protected.
            let h = FrameHeader::parse(&pkt.data).unwrap();
            assert!(h.has_crc());
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            assert_eq!(a.samples, 384);
            if a.data[0].iter().any(|&b| b != 0) {
                saw_nonzero = true;
            }
        }
        assert!(
            saw_nonzero,
            "CRC-protected encode -> decode produced only silence"
        );
    }

    #[test]
    fn make_encoder_with_crc_validates_params() {
        // The CRC factory shares its validation with the default
        // factory: missing sample_rate / channels must be rejected the
        // same way.
        let bare = CodecParameters::audio(CodecId::new("mp1"));
        assert!(make_encoder_with_crc(&bare).is_err());
        let mut sr_only = CodecParameters::audio(CodecId::new("mp1"));
        sr_only.sample_rate = Some(48_000);
        assert!(make_encoder_with_crc(&sr_only).is_err());
    }

    // ---- §2.4.1.5 / §2.4.1.6 LayerSelect dispatch (encoder-level) ----

    /// Build an `n`-sample-per-channel mono S16 sine in the
    /// AudioFrame shape the encoder consumes. Generalisation of
    /// `build_audio_frame_mono` so the same helper feeds both the
    /// 384-sample Layer I path and the 1152-sample Layer II path.
    fn build_audio_frame_mono_n(sample_rate: u32, n: usize) -> AudioFrame {
        let mut bytes = Vec::with_capacity(n * 2);
        for k in 0..n {
            let t = k as f64 / sample_rate as f64;
            let v = (2.0 * std::f64::consts::PI * 1_000.0 * t).sin();
            let s = (v * 16_000.0) as i16;
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        AudioFrame {
            samples: n as u32,
            pts: None,
            data: vec![bytes],
        }
    }

    /// Build an `n`-sample-per-channel stereo S16 frame whose left
    /// channel is a 1 kHz sine and right channel is a 1.5 kHz sine,
    /// interleaved.
    fn build_audio_frame_stereo_n(sample_rate: u32, n: usize) -> AudioFrame {
        let mut bytes = Vec::with_capacity(n * 2 * 2);
        for k in 0..n {
            let t = k as f64 / sample_rate as f64;
            let l = ((2.0 * std::f64::consts::PI * 1_000.0 * t).sin() * 16_000.0) as i16;
            let r = ((2.0 * std::f64::consts::PI * 1_500.0 * t).sin() * 16_000.0) as i16;
            bytes.extend_from_slice(&l.to_le_bytes());
            bytes.extend_from_slice(&r.to_le_bytes());
        }
        AudioFrame {
            samples: n as u32,
            pts: None,
            data: vec![bytes],
        }
    }

    #[test]
    fn make_encoder_default_dispatches_to_layer_i() {
        // Regression: the default `make_encoder` factory keeps producing
        // Layer I frames (header layer bits `0b11`) for byte-for-byte
        // compatibility with the encoder's pre-LayerSelect behaviour,
        // and continues to accept exactly 384 samples/channel.
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(48_000);
        params.channels = Some(1);
        let mut enc = make_encoder(&params).unwrap();
        // The Layer I granularity (§2.4.2.1).
        let frame = build_audio_frame_mono_n(48_000, 384);
        enc.send_frame(&Frame::Audio(frame)).unwrap();
        let pkt = enc.receive_packet().unwrap();
        let h = FrameHeader::parse(&pkt.data).unwrap();
        assert_eq!(h.layer, Layer::I, "default factory must emit Layer I");
        // A wrong-granularity send is rejected on the Layer I path (the
        // 1152 Layer II frame size is not a Layer I frame).
        let wrong = build_audio_frame_mono_n(48_000, 1152);
        let err = enc.send_frame(&Frame::Audio(wrong)).unwrap_err();
        let s = format!("{err}");
        assert!(
            s.contains("384"),
            "Layer I encoder must reject non-384 frame sizes, got: {s}"
        );
    }

    #[test]
    fn make_encoder_layer2_emits_layer_ii_header() {
        // The Layer II factory produces a frame whose header's `layer`
        // bits are `0b10` (Layer II) and whose §2.4.2.1 frame length
        // matches `floor(144 · bitrate / Fs) + padding`. Mono 48 kHz
        // at the 128 kbit/s default lands at exactly 384 bytes
        // (`floor(144 · 128_000 / 48_000)` = 384, padding = 0).
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(48_000);
        params.channels = Some(1);
        let mut enc = make_encoder_layer2(&params).expect("make_encoder_layer2");
        // The Layer II granularity (§2.4.2.1).
        let frame = build_audio_frame_mono_n(48_000, 1152);
        enc.send_frame(&Frame::Audio(frame)).unwrap();
        let pkt = enc.receive_packet().unwrap();
        let h = FrameHeader::parse(&pkt.data).unwrap();
        assert_eq!(h.layer, Layer::II, "Layer II factory must emit Layer II");
        // The §2.4.2.1 frame length: 144 · 128_000 / 48_000 = 384.
        assert_eq!(pkt.data.len(), 384);
        assert_eq!(h.mode, Mode::SingleChannel);
        assert_eq!(h.sampling_frequency, 48_000);
        // The Layer II factory must reject a Layer-I-sized frame on
        // the granularity check (1152, not 384, is the §2.4.2.1
        // Layer II frame size).
        let wrong = build_audio_frame_mono_n(48_000, 384);
        let err = enc.send_frame(&Frame::Audio(wrong)).unwrap_err();
        let s = format!("{err}");
        assert!(
            s.contains("1152"),
            "Layer II encoder must reject non-1152 frame sizes, got: {s}"
        );
    }

    #[test]
    fn make_encoder_layer2_round_trips_through_decoder() {
        // A Layer II encode -> decode loop reaches PCM cleanly: the
        // frame parses as Layer II, the decoder produces 1152
        // samples/channel per frame (§2.4.2.1), and several frames in
        // the synthesis filterbank ramps to non-silence (so we're
        // actually carrying signal end-to-end, not just zero-padding
        // through the encoder).
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(44_100);
        params.channels = Some(2);
        let mut enc = make_encoder_layer2(&params).expect("make_encoder_layer2");
        let mut dec = make_decoder(&params).unwrap();
        let mut saw_nonzero = false;
        for _ in 0..6 {
            let frame = build_audio_frame_stereo_n(44_100, 1152);
            enc.send_frame(&Frame::Audio(frame)).unwrap();
            let pkt = enc.receive_packet().unwrap();
            let h = FrameHeader::parse(&pkt.data).unwrap();
            assert_eq!(h.layer, Layer::II);
            assert_eq!(h.mode, Mode::Stereo);
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            // §2.4.2.1: 1152 samples per channel per Layer II frame.
            assert_eq!(a.samples, 1152);
            // Stereo interleaved: 1152 * 2 channels * 2 bytes/sample.
            assert_eq!(a.data[0].len(), 1152 * 2 * 2);
            if a.data[0].iter().any(|&b| b != 0) {
                saw_nonzero = true;
            }
        }
        assert!(
            saw_nonzero,
            "Layer II encode -> decode produced only silence"
        );
    }

    #[test]
    fn top_level_layer2_psychoacoustic_vbr_round_trips() {
        // The registry-facing `Mp1Encoder` Layer II path forwards the
        // `EncodeParams` psychoacoustic + VBR settings to the inner
        // `Mp1Layer2FrameEncoder`. Build one directly with Layer II +
        // psychoacoustic + VBR and confirm a multi-frame encode -> decode
        // loop produces valid, decodable Layer II frames (each header's
        // bitrate is a real ladder rung; the decoder reads 1152
        // samples/channel; signal reaches PCM end-to-end).
        let enc_params = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::Stereo)
            .with_layer(LayerSelect::LayerII)
            .with_psychoacoustic(true)
            .with_vbr(0.0);
        let mut out = CodecParameters::audio(CodecId::new(CODEC_ID));
        out.sample_rate = Some(48_000);
        out.channels = Some(2);
        out.sample_format = Some(SampleFormat::S16);
        let mut enc = Mp1Encoder::new(CodecId::new(CODEC_ID), enc_params, out);

        let mut dec_params = CodecParameters::audio(CodecId::new("mp1"));
        dec_params.sample_rate = Some(48_000);
        dec_params.channels = Some(2);
        let mut dec = make_decoder(&dec_params).unwrap();

        let mut saw_nonzero = false;
        for _ in 0..6 {
            let frame = build_audio_frame_stereo_n(48_000, 1152);
            enc.send_frame(&Frame::Audio(frame)).unwrap();
            let pkt = enc.receive_packet().unwrap();
            let h = FrameHeader::parse(&pkt.data).unwrap();
            assert_eq!(h.layer, Layer::II);
            assert_eq!(h.mode, Mode::Stereo);
            // The per-frame bitrate must be a real §2.4.2.3 ladder rung,
            // and the emitted frame length must match its header.
            let len = h.frame_length_bytes().expect("fixed per-frame bitrate") as usize;
            assert_eq!(pkt.data.len(), len, "frame length matches header bitrate");
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            assert_eq!(a.samples, 1152);
            if a.data[0].iter().any(|&b| b != 0) {
                saw_nonzero = true;
            }
        }
        assert!(
            saw_nonzero,
            "psychoacoustic VBR Layer II produced only silence"
        );
    }

    #[test]
    fn top_level_layer2_model1_round_trips() {
        // `EncodeParams::with_psy_model(Model1)` reaches the inner
        // `Mp1Layer2FrameEncoder` through the registry-facing wrapper:
        // frames encode under the clause D.1 model and decode to
        // non-silent PCM.
        let enc_params = EncodeParams::new(Bitrate::Fixed(192), 44_100, Mode::Stereo)
            .with_layer(LayerSelect::LayerII)
            .with_psychoacoustic(true)
            .with_psy_model(crate::encode::PsyModel::Model1);
        let mut out = CodecParameters::audio(CodecId::new(CODEC_ID));
        out.sample_rate = Some(44_100);
        out.channels = Some(2);
        out.sample_format = Some(SampleFormat::S16);
        let mut enc = Mp1Encoder::new(CodecId::new(CODEC_ID), enc_params, out);

        let mut dec_params = CodecParameters::audio(CodecId::new("mp1"));
        dec_params.sample_rate = Some(44_100);
        dec_params.channels = Some(2);
        let mut dec = make_decoder(&dec_params).unwrap();

        let mut saw_nonzero = false;
        for _ in 0..4 {
            let frame = build_audio_frame_stereo_n(44_100, 1152);
            enc.send_frame(&Frame::Audio(frame)).unwrap();
            let pkt = enc.receive_packet().unwrap();
            let h = FrameHeader::parse(&pkt.data).unwrap();
            assert_eq!(h.layer, Layer::II);
            dec.send_packet(&pkt).unwrap();
            let Frame::Audio(a) = dec.receive_frame().unwrap() else {
                panic!("audio");
            };
            assert_eq!(a.samples, 1152);
            if a.data[0].iter().any(|&b| b != 0) {
                saw_nonzero = true;
            }
        }
        assert!(saw_nonzero, "Model 1 Layer II produced only silence");
    }

    #[test]
    fn make_encoder_layer2_validates_params() {
        // The Layer II factory shares its parameter validation with the
        // Layer I factory: missing sample_rate / channels must be
        // rejected the same way.
        let bare = CodecParameters::audio(CodecId::new("mp1"));
        assert!(make_encoder_layer2(&bare).is_err());
        let mut sr_only = CodecParameters::audio(CodecId::new("mp1"));
        sr_only.sample_rate = Some(48_000);
        assert!(make_encoder_layer2(&sr_only).is_err());
    }
}
