//! # oxideav-mp1
//!
//! Pure-Rust **MPEG-1 / MPEG-2 LSF Audio Layer I** codec, clean-room
//! rebuilt against ISO/IEC 11172-3 (1993) — with every numeric table
//! read only from the standard's Annex B — and ISO/IEC 13818-3 (1997)
//! §2.4.2.3, whose Lower-Sampling-Frequencies extension adds the
//! 16 / 22.05 / 24 kHz sampling rates and a distinct Layer I bitrate
//! ladder when the `ID` header bit is `0`.
//!
//! The crate implements the full Layer I decode and encode paths
//! across both editions:
//!
//! * The §2.4.1.3 / §2.4.2.3 frame **header**, frame sync and
//!   frame-length computation (§2.4.2.1 / §2.4.3.1), plus the optional
//!   16-bit CRC `error_check()` field (§2.4.1.4), now **verified**
//!   against the §2.4.3.1 generator polynomial
//!   `G(X) = X^16 + X^15 + X^2 + 1` (init `0xFFFF`) over the Table
//!   3-B.5 protected fields (header bits 16…31 + the bit-allocation
//!   field) — see [`header`]. Both the MPEG-1 sampling-frequency /
//!   bitrate tables and the 13818-3 §2.4.2.3 LSF tables are wired in;
//!   the `ID` bit selects which pair applies.
//! * The §2.4.1.5 / §2.4.3.2 **audio-data decode**: 4-bit bit
//!   allocation, 6-bit scalefactor indices, and per-sample
//!   requantization, for all four modes (mono, stereo, dual-channel,
//!   joint-stereo upper-band sharing) — see [`decode`].
//! * The §2.4.3.2 **rescale** by the Annex B Table 3-B.1 scalefactor
//!   multipliers and the polyphase **synthesis filterbank** (the
//!   3-Annex A Figure 3-A.2 flow chart: 32→64 matrixing, the 512-tap
//!   Table 3-B.3 window `D[]`, and the overlap-add) — see [`tables`]
//!   and [`synthesis`].
//! * An [`oxideav_core::Decoder`] that turns one Layer I packet into a
//!   384-sample-per-channel interleaved S16
//!   [`AudioFrame`](oxideav_core::AudioFrame), and a matching
//!   [`oxideav_core::Encoder`] — see [`codec`]. [`register`] installs
//!   them into the runtime registry; the [`decoder::make_decoder`] and
//!   [`encoder::make_encoder`] direct factories build the same boxed
//!   trait objects without the registry, mirroring the rest of the
//!   workspace's dual-API convention.
//!
//! On a CRC mismatch the §2.4.3.1 concealment is **selectable**: the
//! decoder either *mutes* the offending frame
//! ([`ConcealmentMode::Mute`], the default — emit a silent frame and
//! ring out the filterbank history) or *repeats the previous frame*
//! ([`ConcealmentMode::RepeatPrevious`] — re-synthesize the last good
//! frame's subband samples). The mode is chosen at construction via
//! [`Mp1Decoder::with_concealment`] /
//! [`decoder::make_decoder_with_concealment`], or switched at runtime
//! with [`Mp1Decoder::set_concealment`].
//!
//! The Layer I **encoder** now supports optional §2.4.1.4 CRC emission:
//! the default factory ([`encoder::make_encoder`]) still writes
//! `protection_bit == 1` (no CRC), and a new opt-in factory
//! ([`encoder::make_encoder_with_crc`]) writes `protection_bit == 0`
//! plus a 16-bit CRC word over the Annex B Table 3-B.5 protected
//! fields (the same §2.4.3.1 polynomial the decoder verifies against).
//! Both factories produce byte-identical frame lengths — the CRC's 16
//! bits are taken out of the per-frame audio-data budget so the
//! §2.4.2.1 slot count is unchanged.
//!
//! The top-level [`Mp1Encoder`] trait object also exposes a Layer-I /
//! Layer-II dispatch switch via [`encode::EncodeParams::layer`] (a new
//! [`encode::LayerSelect`] field, default Layer I): callers that want
//! Layer II output reach for the direct factory
//! [`encoder::make_encoder_layer2`] (or set `layer = LayerSelect::LayerII`
//! before constructing [`Mp1Encoder`] manually), which drives the
//! §C.1.3 [`Mp1Layer2FrameEncoder`] and consumes 1152 PCM samples per
//! channel per frame (the §2.4.2.1 Layer II granularity) instead of
//! the Layer I 384.
//!
//! **Both** encode layers can drive bit allocation from the Annex D
//! **Psychoacoustic Model 2** ([`model2`]). For Layer I, set
//! [`encode::EncodeParams::psychoacoustic`] (or call
//! [`encode::EncodeParams::with_psychoacoustic`]) and the
//! [`encode::Mp1FrameEncoder`] runs the full clause-D.2.4 per-frame
//! procedure — 1024-point Hann-windowed FFT, two-block unpredictability,
//! per-partition energy/tonality, spreading-function convolution, and
//! the per-FFT-line audibility threshold — to produce per-subband
//! signal-to-mask ratios that feed [`encode::allocate_bits_psy`]. For
//! Layer II, [`encode::Mp1Layer2FrameEncoder::with_psychoacoustic`]
//! drives the §C.1.5.2.7 allocator from the same per-subband SMR via
//! [`encode::encode_layer2_frame_psy`] /
//! [`encode::allocate_bits_layer2_psy`]. The model is honoured at
//! 32 / 44.1 / 48 kHz (the rates with Annex D Model 2 tables); the
//! MPEG-2 LSF half-rates fall back to the signal-energy proxy
//! ([`encode::allocate_bits`] / [`encode::allocate_bits_layer2`]). The
//! Annex D numeric tables — the Layer I & II threshold-in-quiet families
//! (D.1a–c / D.1d–f), D.2a–f, D.3a–c, D.4a–c and D.5 — are fully
//! transcribed in [`psy`] from the staged 11172-3 text extractions /
//! renders.
//!
//! **Both Annex D example models** are assembled: the clause D.2
//! Model 2 above, and the clause D.1 **Psychoacoustic Model 1**
//! ([`model1`]) — the nine-step tonal/non-tonal masker procedure with
//! its per-layer FFT length (512-point Layer I / 1024-point Layer II).
//! [`encode::EncodeParams::with_psy_model`] /
//! [`encode::Mp1Layer2FrameEncoder::with_psy_model`] select the model
//! ([`encode::PsyModel`]); Model 2 remains the default.

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

pub mod codec;
pub mod decode;
pub mod decode_layer2;
pub mod decoder;
pub mod encode;
pub mod encoder;
pub mod header;
// internal — exposed for tests/fuzz; not part of the stable API
#[doc(hidden)]
pub mod model1;
// internal — exposed for tests/fuzz; not part of the stable API
#[doc(hidden)]
pub mod model2;
// internal — exposed for tests/fuzz; not part of the stable API
#[doc(hidden)]
pub mod psy;
pub mod synthesis;
pub mod tables;
// internal — exposed for tests/fuzz; not part of the stable API
#[doc(hidden)]
pub mod tables_layer2;

pub use codec::{register_codecs, ConcealmentMode, Mp1Decoder, Mp1Encoder};
pub use decode::{
    allocation_bits, decode_audio_data, requantize, BitReader, DecodeError, Subband,
    SubbandSamples, SAMPLES_PER_SUBBAND, SUBBANDS,
};
pub use decode_layer2::{
    compute_layer2_crc, decode_layer2_audio_data, verify_layer2_crc, Layer2Subband, Layer2Subbands,
    LAYER2_SAMPLES_PER_FRAME, LAYER2_SAMPLES_PER_SUBBAND,
};
pub use decoder::make_decoder_with_concealment;
pub use encode::{
    allocate_bits, allocate_bits_layer2, allocate_bits_layer2_psy, allocate_bits_psy,
    bitrate_index_layer2, encode_layer2_frame, encode_layer2_frame_psy,
    encode_layer2_frame_psy_with_ancillary, encode_layer2_frame_with_ancillary,
    layer2_bitrate_ladder, layer2_frame_payload_bits, layer2_min_mnr_db, layer2_stereo_bound,
    layer2_subband_peak_per_part, pack_layer2_header, quantize, quantize_layer2_sample,
    select_layer2_intensity_bound, select_layer2_scalefactors, select_layer2_vbr_bitrate,
    select_scalefactor, sum_nbal_per_channel, write_layer2_allocation_field, write_layer2_header,
    write_layer2_samples_field, write_layer2_scalefactor_field, Allocation, AnalysisFilter,
    BitWriter, EncodeError, EncodeParams, Layer2Allocation, Layer2AllocationFieldError,
    Layer2EncodeError, Layer2HeaderError, Layer2HeaderParams, Layer2SamplesFieldError,
    Layer2SamplesFieldInput, Layer2ScalefactorFieldError, Layer2ScalefactorFieldInput,
    Layer2ScalefactorIndices, Layer2SubbandPeaks, LayerSelect, Mp1FrameEncoder,
    Mp1Layer2FrameEncoder, PsyModel,
};
pub use encoder::{make_encoder_layer2, make_encoder_with_crc};
pub use header::{
    detect_free_format_frame_length, find_sync, Bitrate, CrcStatus, Emphasis, FrameHeader,
    FreeFormatFrameLength, FreeFormatProbeError, HeaderError, Id, Layer, Mode, ModeExtension,
};
pub use synthesis::{to_s16, SynthesisFilter};
pub use tables::{ANALYSIS_WINDOW, QUANT_A, QUANT_B, SCALEFACTORS, SNR_DB, SYNTHESIS_WINDOW};

/// Install the MPEG-1 Audio Layer I decoder into `ctx`'s codec
/// registry. See [`codec::register_codecs`].
pub fn register(ctx: &mut RuntimeContext) {
    codec::register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("mp1", register);
