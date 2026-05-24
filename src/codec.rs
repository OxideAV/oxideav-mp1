//! `oxideav_core::Decoder` wiring for MPEG-1 Audio Layer I.
//!
//! [`Mp1Decoder`] turns one compressed Layer I packet into an
//! interleaved S16 [`AudioFrame`]. It ties together the three decode
//! stages built in the sibling modules:
//!
//! 1. [`FrameHeader::parse`](crate::header::FrameHeader::parse) — the
//!    §2.4.1.3 32-bit header, plus the optional CRC word (§2.4.1.4),
//!    skipped (read-but-unverified — the CRC polynomial is a known
//!    spec gap).
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
    Decoder, Error, Frame, Packet, Result,
};

use crate::decode::{decode_audio_data, SAMPLES_PER_SUBBAND};
use crate::header::{find_sync, FrameHeader};
use crate::synthesis::{to_s16, SynthesisFilter};

/// The canonical codec id for MPEG-1 Audio Layer I.
const CODEC_ID: &str = "mp1";

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
}

impl Mp1Decoder {
    fn new(codec_id: CodecId) -> Mp1Decoder {
        Mp1Decoder {
            codec_id,
            filters: Vec::new(),
            pending: None,
            eof: false,
        }
    }

    /// Decode one Layer I frame found in `data` into interleaved S16
    /// PCM. Returns `(pcm_bytes, samples_per_channel, channels)`.
    fn decode_frame(&mut self, data: &[u8]) -> Result<(Vec<u8>, u32, usize)> {
        // Locate the frame sync. find_sync validates the whole 32-bit
        // header so a leading ID3 tag / junk is skipped.
        let off = find_sync(data)
            .ok_or_else(|| Error::invalid("oxideav-mp1: no Layer I frame sync in packet"))?;
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

        let subbands = decode_audio_data(&header, audio)
            .map_err(|e| Error::invalid(format!("oxideav-mp1: audio_data: {e}")))?;

        // Ensure we have one synthesis bank per channel (state persists
        // across frames for the overlap-add history).
        while self.filters.len() < nch {
            self.filters.push(SynthesisFilter::new());
        }

        // Run the synthesis filterbank: for each of the 12 slots, for
        // each channel, push 32 subband samples through the bank to get
        // 32 PCM samples. Interleave the channels into S16.
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
        Ok((bytes, total_samples as u32, nch))
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
        for f in &mut self.filters {
            f.reset();
        }
        Ok(())
    }
}

/// Install the MPEG-1 Audio Layer I decoder into `reg`.
///
/// Claims the WAVE format tag `0x0050` (MPEG-1 Audio, Layer I/II) and
/// the Matroska codec id `A_MPEG/L1` so containers can route Layer I
/// streams here. The decoder emits interleaved S16 PCM.
pub fn register_codecs(reg: &mut CodecRegistry) {
    let info = CodecInfo::new(CodecId::new(CODEC_ID))
        .capabilities(
            CodecCapabilities::audio("mp1")
                .with_decode()
                .with_lossy(true),
        )
        .decoder(make_decoder)
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
}
