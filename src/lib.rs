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
//!   16-bit CRC `error_check()` field (§2.4.1.4) — see [`header`].
//!   Both the MPEG-1 sampling-frequency / bitrate tables and the
//!   13818-3 §2.4.2.3 LSF tables are wired in; the `ID` bit selects
//!   which pair applies.
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
//!   [`AudioFrame`](oxideav_core::AudioFrame) — see [`codec`].
//!   [`register`] installs it into the runtime registry.
//!
//! Spec gaps that remain (none block decode to PCM): the CRC-16
//! generator polynomial and Table 3-B.5 bit-coverage are not present
//! in the staged PDF, so the CRC word is read but not verified
//! ([`header::CrcStatus::PresentUnverified`]); and a Layer I encoder
//! is not yet implemented.

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

pub mod codec;
pub mod decode;
pub mod encode;
pub mod header;
pub mod synthesis;
pub mod tables;

pub use codec::{register_codecs, Mp1Decoder, Mp1Encoder};
pub use decode::{
    allocation_bits, decode_audio_data, requantize, BitReader, DecodeError, Subband,
    SubbandSamples, SAMPLES_PER_SUBBAND, SUBBANDS,
};
pub use encode::{
    allocate_bits, quantize, select_scalefactor, Allocation, AnalysisFilter, BitWriter,
    EncodeError, EncodeParams, Mp1FrameEncoder,
};
pub use header::{
    find_sync, Bitrate, CrcStatus, Emphasis, FrameHeader, HeaderError, Id, Mode, ModeExtension,
};
pub use synthesis::{to_s16, SynthesisFilter};
pub use tables::{ANALYSIS_WINDOW, QUANT_A, QUANT_B, SCALEFACTORS, SNR_DB, SYNTHESIS_WINDOW};

/// Install the MPEG-1 Audio Layer I decoder into `ctx`'s codec
/// registry. See [`codec::register_codecs`].
pub fn register(ctx: &mut RuntimeContext) {
    codec::register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("mp1", register);
