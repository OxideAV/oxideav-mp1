//! Direct factory entry point for the MPEG-1 Audio Layer I decoder.
//!
//! This is the historical *direct* half of the workspace dual-API
//! convention: a caller that already knows it wants Layer I can build a
//! boxed [`oxideav_core::Decoder`] without going through the runtime
//! [`CodecRegistry`](oxideav_core::CodecRegistry). It is the exact same
//! construction the registry path performs in
//! [`register_codecs`](crate::register_codecs) — both routes end at
//! [`Mp1Decoder`](crate::codec::Mp1Decoder).
//!
//! ```no_run
//! use oxideav_core::{CodecId, CodecParameters};
//! let params = CodecParameters::audio(CodecId::new("mp1"));
//! let mut dec = oxideav_mp1::decoder::make_decoder(&params).unwrap();
//! # let _ = &mut dec;
//! ```

use oxideav_core::{CodecParameters, Decoder, Error};

/// Build a boxed MPEG-1 Audio Layer I [`Decoder`] from `params`.
///
/// The channel count is read from the frame headers at decode time, so
/// `params` only needs to carry the codec id (`"mp1"`). This is the
/// direct-API twin of the registry decoder factory installed by
/// [`register_codecs`](crate::register_codecs); both produce an
/// identical [`Mp1Decoder`](crate::codec::Mp1Decoder).
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>, Error> {
    crate::codec::make_decoder(params)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::{CodecId, Frame, Packet, TimeBase};

    /// Append `n` bits (`val`, MSB-first) to a growing byte buffer.
    fn put(bytes: &mut Vec<u8>, acc: &mut u32, nbits: &mut u8, val: u32, n: u8) {
        for i in (0..n).rev() {
            let b = (val >> i) & 1;
            *acc = (*acc << 1) | b;
            *nbits += 1;
            if *nbits == 8 {
                bytes.push(*acc as u8);
                *acc = 0;
                *nbits = 0;
            }
        }
    }

    /// Hand-build a mono, no-CRC, 48 kHz Layer I frame that allocates
    /// subband 0 (nb=4) with the supplied scalefactor index and sample
    /// codes — enough to drive a real decode through the boxed object.
    fn build_mono_frame(scf_index: u32, sample_codes: &[u32; 12]) -> Vec<u8> {
        let word: u32 = (0xFFF << 20)
            | (1 << 19) // ID = MPEG-1
            | (0b11 << 17) // layer I
            | (1 << 16) // protection = 1 (no CRC)
            | (0b1000 << 12) // bitrate index (256 kbit/s)
            | (0b01 << 10) // 48 kHz
            | (0b11 << 6) // single_channel
            | (1 << 2); // original
        let mut frame = word.to_be_bytes().to_vec();

        let mut bytes = Vec::new();
        let mut acc = 0u32;
        let mut nbits = 0u8;
        // allocation: sb0 = nb 4 (0b0011); sb1..31 = 0.
        put(&mut bytes, &mut acc, &mut nbits, 0b0011, 4);
        for _ in 1..32 {
            put(&mut bytes, &mut acc, &mut nbits, 0, 4);
        }
        // scalefactor for sb0 (6 bits).
        put(&mut bytes, &mut acc, &mut nbits, scf_index, 6);
        // 12 samples for sb0, 4 bits each.
        for &c in sample_codes {
            put(&mut bytes, &mut acc, &mut nbits, c, 4);
        }
        if nbits > 0 {
            acc <<= 8 - nbits;
            bytes.push(acc as u8);
        }
        frame.extend_from_slice(&bytes);
        frame
    }

    #[test]
    fn make_decoder_builds_a_working_boxed_decoder() {
        let params = CodecParameters::audio(CodecId::new("mp1"));
        let mut dec = make_decoder(&params).expect("make_decoder");
        assert_eq!(dec.codec_id().as_str(), "mp1");

        let codes = [15u32; 12];
        let frame = build_mono_frame(0, &codes);
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), frame);
        dec.send_packet(&pkt).unwrap();
        let Frame::Audio(a) = dec.receive_frame().unwrap() else {
            panic!("expected audio frame");
        };
        // 384 samples/channel, mono, S16 -> 768 bytes.
        assert_eq!(a.samples, 384);
        assert_eq!(a.data.len(), 1);
        assert_eq!(a.data[0].len(), 384 * 2);
    }
}
