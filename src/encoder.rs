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
/// identical [`Mp1Encoder`](crate::codec::Mp1Encoder). The optional
/// §2.4.1.4 CRC is **not** emitted (`protection_bit == 1`); use
/// [`make_encoder_with_crc`] to opt in.
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>, Error> {
    crate::codec::make_encoder(params)
}

/// Build a boxed MPEG-1 Audio Layer I [`Encoder`] from `params`, with
/// the optional §2.4.1.4 CRC `error_check()` field enabled.
///
/// Identical to [`make_encoder`] except the produced encoder writes
/// `protection_bit == 0` and a 16-bit CRC word over the Annex B Table
/// 3-B.5 Layer I protected fields (§2.4.3.1 polynomial
/// `G(X) = X^16 + X^15 + X^2 + 1`, init `0xFFFF`). A decoder running
/// in [`ConcealmentMode::Mute`](crate::ConcealmentMode::Mute) or
/// [`ConcealmentMode::RepeatPrevious`](crate::ConcealmentMode::RepeatPrevious)
/// will accept the CRC and continue to decode the frame; a bit-flip in
/// the protected region will trigger §2.4.3.1 concealment in the same
/// decoder.
pub fn make_encoder_with_crc(params: &CodecParameters) -> Result<Box<dyn Encoder>, Error> {
    crate::codec::make_encoder_with_crc(params)
}

/// Build a boxed MPEG-1 Audio **Layer II** [`Encoder`] from `params`.
///
/// Identical parameter contract to [`make_encoder`] (`sample_rate` and
/// `channels` required, `bit_rate` optional), but the produced encoder
/// dispatches to [`Mp1Layer2FrameEncoder`](crate::Mp1Layer2FrameEncoder)
/// inside the same [`Mp1Encoder`](crate::Mp1Encoder) trait-object
/// wrapper and consumes **1152** PCM samples per channel per
/// [`Encoder::send_frame`] call (the §2.4.2.1 Layer II frame
/// granularity) rather than the Layer I 384. The §2.4.1.4 CRC is
/// **not** emitted by default.
///
/// Defaults to a stereo midpoint on the §2.4.2.3 Layer II bitrate
/// ladder (128 kbit/s mono / 192 stereo at MPEG-1 rates; 64 / 96 at
/// 13818-3 §2.4.2.3 LSF rates).
pub fn make_encoder_layer2(params: &CodecParameters) -> Result<Box<dyn Encoder>, Error> {
    crate::codec::make_encoder_layer2(params)
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

    /// The direct-API `make_encoder_with_crc` factory threads the
    /// §2.4.1.4 CRC emission flag through to the boxed `Mp1Encoder`:
    /// produced frames carry `protection_bit == 0` and a verifying
    /// 16-bit CRC, where the plain `make_encoder` factory still emits
    /// `protection_bit == 1` and no CRC.
    #[test]
    fn make_encoder_with_crc_round_trip() {
        use crate::header::FrameHeader;
        let mut params = CodecParameters::audio(CodecId::new("mp1"));
        params.sample_rate = Some(48_000);
        params.channels = Some(1);

        // The opt-in factory writes a CRC the decoder-side helper
        // verifies clean.
        let mut crc_enc = make_encoder_with_crc(&params).expect("make_encoder_with_crc");
        let pcm = {
            let mut v = Vec::with_capacity(SAMPLES_PER_FRAME as usize * 2);
            for n in 0..SAMPLES_PER_FRAME as usize {
                let t = n as f64 / 48_000.0;
                let s = ((2.0 * std::f64::consts::PI * 1_000.0 * t).sin() * 16_000.0) as i16;
                v.extend_from_slice(&s.to_le_bytes());
            }
            v
        };
        let frame = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: None,
            data: vec![pcm.clone()],
        };
        crc_enc.send_frame(&Frame::Audio(frame)).unwrap();
        let crc_pkt = crc_enc.receive_packet().unwrap();
        let h = FrameHeader::parse(&crc_pkt.data).unwrap();
        assert!(
            h.has_crc(),
            "make_encoder_with_crc must clear protection_bit"
        );
        assert!(h
            .verify_crc(&crc_pkt.data[..4], &crc_pkt.data[4..])
            .expect("CRC region present")
            .is_good());

        // The plain factory keeps the no-CRC default.
        let mut plain_enc = make_encoder(&params).expect("make_encoder");
        let frame_plain = AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: None,
            data: vec![pcm],
        };
        plain_enc.send_frame(&Frame::Audio(frame_plain)).unwrap();
        let plain_pkt = plain_enc.receive_packet().unwrap();
        let hp = FrameHeader::parse(&plain_pkt.data).unwrap();
        assert!(!hp.has_crc(), "make_encoder must default to no CRC");

        // Both produce equal frame byte counts (§2.4.2.1 slot count is
        // header-derived and independent of CRC presence).
        assert_eq!(crc_pkt.data.len(), plain_pkt.data.len());
    }
}
