//! # oxideav-mp1
//!
//! Pure-Rust **MPEG-1 Audio Layer I** decoder, clean-room rebuilt
//! against ISO/IEC 11172-3 (1993) with every numeric table read only
//! from the standard's Annex B.
//!
//! The crate implements the full Layer I decode path:
//!
//! * The §2.4.1.3 / §2.4.2.3 frame **header**, frame sync and
//!   frame-length computation (§2.4.2.1 / §2.4.3.1), plus the optional
//!   16-bit CRC `error_check()` field (§2.4.1.4) — see [`header`].
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
pub mod header;
pub mod synthesis;
pub mod tables;

pub use codec::{register_codecs, Mp1Decoder};
pub use decode::{
    allocation_bits, decode_audio_data, requantize, BitReader, DecodeError, Subband,
    SubbandSamples, SAMPLES_PER_SUBBAND, SUBBANDS,
};
pub use header::{
    find_sync, Bitrate, CrcStatus, Emphasis, FrameHeader, HeaderError, Id, Mode, ModeExtension,
};
pub use synthesis::{to_s16, SynthesisFilter};
pub use tables::{SCALEFACTORS, SYNTHESIS_WINDOW};

/// Crate-local error type. Header parsing has its own
/// [`HeaderError`] and the audio-data decode has [`DecodeError`];
/// this variant covers the codec paths (notably encode) the rebuild
/// has not reached yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The requested path (Layer I **encode**) is not wired up yet.
    /// Decode to PCM is fully implemented (see [`codec`]).
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "oxideav-mp1: Layer I encode not implemented yet")
    }
}

impl std::error::Error for Error {}

/// Install the MPEG-1 Audio Layer I decoder into `ctx`'s codec
/// registry. See [`codec::register_codecs`].
pub fn register(ctx: &mut RuntimeContext) {
    codec::register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("mp1", register);
