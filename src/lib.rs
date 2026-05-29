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
//! One spec gap remains, not blocking decode to PCM: Annex D's
//! psychoacoustic models are not implemented (the encoder's allocator
//! is signal-energy-driven); the staged 11172-3 PDF carries Annex D
//! §D.1 prose but the essential numeric tables (D.1a/b/c absolute
//! thresholds, D.2a/b/c critical band boundaries) are only present in
//! an OCR-corrupted text layer, so the perceptual model is a DOCS-GAP
//! awaiting clean Annex D renders.

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

pub mod codec;
pub mod decode;
pub mod decode_layer2;
pub mod decoder;
pub mod encode;
pub mod encoder;
pub mod header;
pub mod synthesis;
pub mod tables;
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
    allocate_bits, allocate_bits_layer2, bitrate_index_layer2, layer2_frame_payload_bits,
    layer2_stereo_bound, pack_layer2_header, quantize, select_scalefactor, sum_nbal_per_channel,
    write_layer2_allocation_field, write_layer2_header, write_layer2_samples_field,
    write_layer2_scalefactor_field, Allocation, AnalysisFilter, BitWriter, EncodeError,
    EncodeParams, Layer2Allocation, Layer2AllocationFieldError, Layer2HeaderError,
    Layer2HeaderParams, Layer2SamplesFieldError, Layer2SamplesFieldInput,
    Layer2ScalefactorFieldError, Layer2ScalefactorFieldInput, Mp1FrameEncoder,
};
pub use encoder::make_encoder_with_crc;
pub use header::{
    find_sync, Bitrate, CrcStatus, Emphasis, FrameHeader, HeaderError, Id, Layer, Mode,
    ModeExtension,
};
pub use synthesis::{to_s16, SynthesisFilter};
pub use tables::{ANALYSIS_WINDOW, QUANT_A, QUANT_B, SCALEFACTORS, SNR_DB, SYNTHESIS_WINDOW};

/// Install the MPEG-1 Audio Layer I decoder into `ctx`'s codec
/// registry. See [`codec::register_codecs`].
pub fn register(ctx: &mut RuntimeContext) {
    codec::register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("mp1", register);
