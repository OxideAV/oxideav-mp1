//! Direct factory entry point for the MPEG-1 Audio Layer I encoder.
//!
//! This is the historical *direct* half of the workspace dual-API
//! convention: a caller that already knows it wants Layer I can build a
//! boxed [`oxideav_core::Encoder`] without going through the runtime
//! [`CodecRegistry`](oxideav_core::CodecRegistry). It is the exact same
//! construction the registry path performs in
//! [`register_codecs`](crate::register_codecs) — both routes end at
//! [`Mp1Encoder`](crate::codec::Mp1Encoder).
//!
//! ```no_run
//! use oxideav_core::{CodecId, CodecParameters};
//! let mut params = CodecParameters::audio(CodecId::new("mp1"));
//! params.sample_rate = Some(48_000);
//! params.channels = Some(2);
//! let mut enc = oxideav_mp1::encoder::make_encoder(&params).unwrap();
//! # let _ = &mut enc;
//! ```

use oxideav_core::{CodecParameters, Encoder, Error};

/// Build a boxed MPEG-1 Audio Layer I [`Encoder`] from `params`.
///
/// `sample_rate` (one of the six Layer I rates) and `channels` (1 or 2)
/// are required; `bit_rate` is optional and defaults to a sensible
/// per-rate value when absent. This is the direct-API twin of the
/// registry encoder factory installed by
/// [`register_codecs`](crate::register_codecs); both produce an
/// identical [`Mp1Encoder`](crate::codec::Mp1Encoder).
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>, Error> {
    crate::codec::make_encoder(params)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::{AudioFrame, CodecId, Frame};

    /// 384 samples/channel is the Layer I frame granularity (§2.4.2.1).
    const SAMPLES_PER_FRAME: u32 = 384;

    #[test]
    fn make_encoder_requires_sample_rate_and_channels() {
        // A bare params (no sample_rate / channels) must be rejected,
        // confirming the wrapper threads through the same validation the
        // registry factory performs.
        let bare = CodecParameters::audio(CodecId::new("mp1"));
        assert!(make_encoder(&bare).is_err());
    }

    #[test]
    fn make_encoder_builds_a_working_boxed_encoder() {
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(48_000);
        params.channels = Some(2);

        let mut enc = make_encoder(&params).expect("make_encoder");
        assert_eq!(enc.codec_id().as_str(), "mp1");
        // The factory fills in the output stream parameters.
        assert_eq!(enc.output_params().sample_rate, Some(48_000));
        assert_eq!(enc.output_params().channels, Some(2));

        // Drive one real encode: a 48 kHz sine, interleaved S16 stereo,
        // exactly 384 samples/channel.
        let nch = 2usize;
        let total = SAMPLES_PER_FRAME as usize * nch;
        let mut pcm = Vec::with_capacity(total * 2);
        for n in 0..SAMPLES_PER_FRAME as usize {
            let t = n as f64 / 48_000.0;
            let v = (2.0 * std::f64::consts::PI * 440.0 * t).sin();
            let s = (v * 16_000.0) as i16;
            for _ in 0..nch {
                pcm.extend_from_slice(&s.to_le_bytes());
            }
        }
        let frame = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: None,
            data: vec![pcm],
        };
        enc.send_frame(&Frame::Audio(frame)).unwrap();
        let pkt = enc.receive_packet().expect("receive_packet");
        // A real Layer I packet: starts with the 0xFFF sync and is
        // non-trivially sized.
        assert!(pkt.data.len() > 4, "encoded packet too small");
        assert_eq!(pkt.data[0], 0xFF);
        assert_eq!(pkt.data[1] & 0xF0, 0xF0);
    }
}
