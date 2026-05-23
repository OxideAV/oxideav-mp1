//! # oxideav-mp1
//!
//! **Status:** clean-room rebuild in progress (reset 2026-05-24).
//!
//! The prior implementation was retired under the workspace clean-room
//! policy: the provenance of its synthesis-window data table could not
//! be defended as clean-room (the module doc-comment recorded the
//! values as transcribed from an external library's source file rather
//! than read solely from the ISO/IEC specification). The crate is being
//! re-implemented from scratch against ISO/IEC 11172-3 (1993), the
//! MPEG-1 audio standard, with every numeric table read only from the
//! standard.
//!
//! This foundation implements the MPEG-1 Audio **Layer I** frame
//! header (§2.4.1.3 / §2.4.2.3), frame sync and frame-length
//! computation (§2.4.2.1 / §2.4.3.1), and the structural wiring for the
//! optional 16-bit CRC `error_check()` (§2.4.1.4) — see [`header`] —
//! and the Layer I **audio-data decode** up to requantized subband
//! samples (§2.4.1.5 / §2.4.2.5 / §2.4.3.2) — see [`decode`].
//!
//! [`decode::decode_audio_data`] reads the per-subband 4-bit bit
//! allocation, the 6-bit scalefactor indices, and the per-sample
//! requantization, producing the 32 × 12 requantized subband samples
//! per channel for one frame (mono, stereo, dual-channel, and
//! joint-stereo upper-band sharing). The final rescale by the
//! Annex B Table 3-B.1 scalefactor multiplier and the polyphase
//! synthesis filterbank are **not** implemented yet (see the README
//! "Spec gaps" note for why Table 3-B.1 is unavailable), so the crate
//! registers no [`Decoder`](oxideav_core::Decoder) into the runtime
//! context. [`register`] is a no-op until a decoder is wired.

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

pub mod decode;
pub mod header;

pub use decode::{
    allocation_bits, decode_audio_data, requantize, BitReader, DecodeError, Subband,
    SubbandSamples, SAMPLES_PER_SUBBAND, SUBBANDS,
};
pub use header::{
    find_sync, Bitrate, CrcStatus, Emphasis, FrameHeader, HeaderError, Id, Mode, ModeExtension,
};

/// Crate-local error type. Header parsing has its own
/// [`HeaderError`]; this variant covers the parts of the codec
/// (audio-data decode, encode) that the clean-room rebuild has not
/// reached yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The requested decode/encode path is not wired up yet; only the
    /// Layer I frame-header foundation (see [`header`]) is implemented.
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "oxideav-mp1: audio-data decode not implemented yet")
    }
}

impl std::error::Error for Error {}

/// No-op codec registration — the orphan-rebuild scaffold registers
/// nothing into the runtime context.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("mp1", register);
