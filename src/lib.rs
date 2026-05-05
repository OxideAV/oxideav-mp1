//! MPEG-1 Audio Layer I (MP1) codec.
//!
//! Decoder: packet-in / `AudioFrame`-out, covering:
//!
//! - MPEG-1 sample rates: 32 000, 44 100, and 48 000 Hz.
//! - All Layer I channel modes: single-channel, stereo, dual-channel,
//!   joint-stereo (bound-based sample sharing; Layer I has no intensity
//!   stereo scaling — §2.4.2.3).
//! - Bit-allocation codes 0..14 (15 is forbidden and rejected).
//! - 6-bit scalefactor indices into the `SCALE[64]` table
//!   (ISO/IEC 11172-3 Table 3-B.1).
//! - 32-band polyphase synthesis filter per Annex B / Annex D.
//!
//! Encoder: CBR or VBR Layer I output, mirror of the decode path.
//!
//! - **CBR**: greedy energy-per-bit allocation against a fixed slot
//!   from the standard ladder.
//! - **VBR**: per-frame masking-driven allocation (`crate::psy`)
//!   followed by an energy-per-bit target-fill phase capped by a
//!   rolling-average controller. Per-frame `bitrate_index` floats
//!   over the standard slots; the long-term average converges on the
//!   user-supplied `vbr_target_kbps`.
//!
//! Mono or plain stereo only, no joint stereo, no CRC. See
//! [`encoder`] for the `vbr_quality` / `vbr_target_kbps` options.
//!
//! Not in scope (either direction): CRC verification, free-format frames
//! (bitrate index 0), intensity-stereo scaling.
//!
//! See [`decoder::make_decoder`] / [`encoder::make_encoder`] for entry
//! points and [`crate::bitalloc`] for the requantization math.

#![allow(
    clippy::needless_range_loop,
    clippy::excessive_precision,
    clippy::unreadable_literal,
    clippy::too_many_arguments,
    clippy::doc_overindented_list_items
)]

pub mod analysis;
pub mod bitalloc;
pub mod decoder;
pub mod encoder;
pub mod header;
pub mod psy;
pub mod synthesis;
pub mod window;

use oxideav_core::{CodecCapabilities, CodecId, CodecParameters, Result};
use oxideav_core::{CodecInfo, CodecRegistry, Decoder, Encoder};

pub const CODEC_ID_STR: &str = "mp1";

pub fn register_codecs(reg: &mut CodecRegistry) {
    let caps = CodecCapabilities::audio("mp1_sw")
        .with_lossy(true)
        .with_intra_only(true)
        .with_max_channels(2)
        .with_max_sample_rate(48_000);
    reg.register(
        CodecInfo::new(CodecId::new(CODEC_ID_STR))
            .capabilities(caps)
            .decoder(make_decoder)
            .encoder(make_encoder),
    );
}

/// Unified registration entry point — installs MP1 into the codec
/// sub-registry of the supplied [`oxideav_core::RuntimeContext`].
pub fn register(ctx: &mut oxideav_core::RuntimeContext) {
    register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("mp1", register);

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    decoder::make_decoder(params)
}

fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    encoder::make_encoder(params)
}

#[cfg(test)]
mod register_tests {
    use super::*;

    #[test]
    fn register_via_runtime_context_installs_codec_factory() {
        let mut ctx = oxideav_core::RuntimeContext::new();
        register(&mut ctx);
        let id = CodecId::new(CODEC_ID_STR);
        assert!(
            ctx.codecs.has_decoder(&id),
            "decoder factory not installed via RuntimeContext"
        );
        assert!(
            ctx.codecs.has_encoder(&id),
            "encoder factory not installed via RuntimeContext"
        );
    }
}
