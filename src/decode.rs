//! MPEG-1 Audio **Layer I** audio-data decode (bit allocation,
//! scalefactors, sample requantization).
//!
//! Everything in this module is derived solely from ISO/IEC 11172-3
//! (1993), Layer I clauses:
//!
//! * §2.4.1.5 — `audio_data()` syntax: the per-subband 4-bit
//!   `allocation[ch][sb]` fields, the 6-bit `scalefactor[ch][sb]`
//!   read for every subband whose allocation is non-zero, and the
//!   `2..15`-bit `sample[ch][sb][s]` reads (12 samples per subband).
//! * §2.4.2.5 — semantics: the `allocation[ch][sb]` → *bits per
//!   sample* table (index 0 → 0 bits/no samples, index 1 → 2 bits,
//!   …, index 14 → 15 bits, index 15 → invalid), and the statement
//!   that the six scalefactor bits index Table 3-B.1 "LAYER I, II
//!   SCALEFACTORS".
//! * §2.4.3.2 — the audio-decoding process for Layer I: read `nb`
//!   bits per sample in the §2.4.1.5 order, **invert the first
//!   (MSB) bit**, interpret the result as a two's-complement
//!   fractional number whose MSB has weight −1, and apply the
//!   linear requantization formula. Samples in subbands carried in
//!   intensity_stereo mode (the upper `[bound, 32)` band in
//!   joint_stereo) are copied to both channels. The requantized
//!   value is then rescaled by the Table 3-B.1 multiplier.
//!
//! ## What this module produces
//!
//! For one Layer I frame it produces the 32 × 12 *requantized*
//! subband samples per channel (the [`SubbandSamples`] struct),
//! carrying the requantized fractional value `s''` from the
//! §2.4.3.2 linear formula together with the raw 6-bit scalefactor
//! **index** for each subband.
//!
//! ## Final rescale by Table 3-B.1
//!
//! The last step of §2.4.3.2 multiplies `s''` by the scalefactor
//! found in **3-Annex B, Table 3-B.1 "LAYER I, II SCALEFACTORS"**:
//! `s' = scalefactor[index] · s''`. Table 3-B.1 is now staged
//! (PDF page 51, transcribed into [`crate::tables::SCALEFACTORS`]),
//! so the per-subband requantized samples carry the index verbatim
//! ([`Subband::scalefactor_index`]) and the rescaled value is
//! produced on demand via [`Subband::rescaled_samples`] /
//! [`SubbandSamples::slot`]. The slot view feeds the §2.4.3.2
//! synthesis filterbank ([`crate::synthesis`]).

use crate::header::{FrameHeader, Mode};
use crate::tables::SCALEFACTORS;

/// Number of subbands in a Layer I frame (§2.4.1.5: `sb < 32`).
pub const SUBBANDS: usize = 32;

/// Number of subband samples per subband in one Layer I frame
/// (§2.4.1.5: `s < 12`; §2.4.2.1: 12 × 32 = 384 samples per frame).
pub const SAMPLES_PER_SUBBAND: usize = 12;

/// The §2.4.2.5 `allocation[ch][sb]` → *bits per sample* table for
/// Layer I.
///
/// The 4-bit allocation field is an unsigned integer used directly
/// as an index into this table:
///
/// | allocation | bits/sample |
/// |-----------:|------------:|
/// | 0          | 0 (no samples transferred) |
/// | 1          | 2 |
/// | 2          | 3 |
/// | …          | … |
/// | 14         | 15 |
/// | 15         | invalid |
///
/// Index 0 yields 0 (the subband carries no sample data); indices
/// 1..=14 yield 2..=15; index 15 is *invalid* and is represented by
/// `None`.
const ALLOCATION_BITS: [Option<u8>; 16] = [
    Some(0),  // 0  -> 0   (no samples transferred)
    Some(2),  // 1  -> 2
    Some(3),  // 2  -> 3
    Some(4),  // 3  -> 4
    Some(5),  // 4  -> 5
    Some(6),  // 5  -> 6
    Some(7),  // 6  -> 7
    Some(8),  // 7  -> 8
    Some(9),  // 8  -> 9
    Some(10), // 9  -> 10
    Some(11), // 10 -> 11
    Some(12), // 11 -> 12
    Some(13), // 12 -> 13
    Some(14), // 13 -> 14
    Some(15), // 14 -> 15
    None,     // 15 -> invalid
];

/// Map a raw 4-bit `allocation[ch][sb]` value to the number of bits
/// (`nb`) per sample for that subband, per the §2.4.2.5 Layer I
/// table.
///
/// Returns `Some(0)` for allocation `0` (no samples), `Some(nb)`
/// for `1..=14`, and `None` for the *invalid* allocation `15`.
pub fn allocation_bits(allocation: u8) -> Option<u8> {
    ALLOCATION_BITS.get(allocation as usize).copied().flatten()
}

/// Errors that can arise while decoding Layer I audio data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The bitstream ran out of bits while a field was being read.
    UnexpectedEnd,
    /// An `allocation[ch][sb]` field held the *invalid* value `15`
    /// (§2.4.2.5). Carries the channel and subband for diagnostics.
    InvalidAllocation { channel: usize, subband: usize },
    /// The free-format / forbidden bitrate condition: the frame
    /// length (and therefore the audio-data extent) cannot be
    /// established from the header alone, so a decode of one
    /// self-contained frame is not attempted here.
    UndeterminedFrameLength,
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DecodeError::UnexpectedEnd => {
                write!(f, "bitstream ended before audio data was fully read")
            }
            DecodeError::InvalidAllocation { channel, subband } => write!(
                f,
                "allocation 0b1111 (invalid) at channel {channel}, subband {subband}"
            ),
            DecodeError::UndeterminedFrameLength => {
                write!(f, "frame length undetermined (free/forbidden bitrate)")
            }
        }
    }
}

impl std::error::Error for DecodeError {}

/// A big-endian, most-significant-bit-first reader over a byte
/// slice (§2.3: "The byte order of multi-byte words is most
/// significant byte first"; the bitstream syntax of §2.4.1 is read
/// MSB-first within each byte).
#[derive(Debug)]
pub struct BitReader<'a> {
    bytes: &'a [u8],
    /// Absolute bit position from the start of `bytes`.
    bit_pos: usize,
}

impl<'a> BitReader<'a> {
    /// Create a reader positioned at the first bit of `bytes`.
    pub fn new(bytes: &'a [u8]) -> BitReader<'a> {
        BitReader { bytes, bit_pos: 0 }
    }

    /// Number of bits already consumed.
    pub fn bits_read(&self) -> usize {
        self.bit_pos
    }

    /// Number of bits still available.
    pub fn bits_remaining(&self) -> usize {
        self.bytes.len() * 8 - self.bit_pos
    }

    /// Read `n` bits (`0 <= n <= 32`) MSB-first as an unsigned
    /// integer. Returns [`DecodeError::UnexpectedEnd`] if fewer than
    /// `n` bits remain.
    pub fn read_bits(&mut self, n: u8) -> Result<u32, DecodeError> {
        debug_assert!(n <= 32, "read_bits supports up to 32 bits");
        let n = n as usize;
        if n == 0 {
            return Ok(0);
        }
        if self.bits_remaining() < n {
            return Err(DecodeError::UnexpectedEnd);
        }
        let mut value: u32 = 0;
        for _ in 0..n {
            let byte = self.bytes[self.bit_pos >> 3];
            let bit = (byte >> (7 - (self.bit_pos & 7))) & 1;
            value = (value << 1) | u32::from(bit);
            self.bit_pos += 1;
        }
        Ok(value)
    }
}

/// Apply the §2.4.3.2 Layer I sample requantization to one raw
/// `nb`-bit sample `code`.
///
/// Per §2.4.3.2: after the `nb` bits have been gathered, **the
/// first (MSB) bit is inverted**; the resulting `nb`-bit number is a
/// two's-complement fraction `s'''` whose MSB has weight −1
/// (`s''' = signed / 2^(nb-1)`). The requantized value `s''` is then
/// obtained from the linear formula
///
/// ```text
/// s'' = (2^nb / (2^nb − 1)) · (s''' + 2^(−nb+1))
/// ```
///
/// `nb` must be in `2..=15` (the Layer I sample widths from the
/// §2.4.2.5 allocation table); `code` is the raw unsigned `nb`-bit
/// value read from the bitstream.
pub fn requantize(code: u32, nb: u8) -> f64 {
    debug_assert!((2..=15).contains(&nb), "Layer I sample width 2..=15");
    let nb_u = nb as u32;
    // Invert the first (MSB) bit (§2.4.3.2).
    let msb = 1u32 << (nb_u - 1);
    let inverted = code ^ msb;
    // Interpret the nb-bit value as two's complement: if the MSB is
    // set the number is negative (MSB weight −1).
    let signed: i64 = if inverted & msb != 0 {
        i64::from(inverted) - (1i64 << nb_u)
    } else {
        i64::from(inverted)
    };
    // s''' is the fraction in [-1, 1): signed / 2^(nb-1).
    let s3 = signed as f64 / (1u64 << (nb_u - 1)) as f64;
    // Linear formula: s'' = (2^nb/(2^nb-1)) * (s''' + 2^(-nb+1)).
    let two_pow_nb = (1u64 << nb_u) as f64;
    let gain = two_pow_nb / (two_pow_nb - 1.0);
    let offset = 2f64.powi(-(nb as i32) + 1);
    gain * (s3 + offset)
}

/// One decoded subband: its 6-bit scalefactor index and the 12
/// requantized samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Subband {
    /// Whether the subband carries audio data. `false` when
    /// `allocation[ch][sb] == 0` (§2.4.1.5: "For code '0000' no
    /// samples are transferred"); in that case `bits` is 0,
    /// `scalefactor_index` is meaningless (`0`), and every entry of
    /// `samples` is `0.0` (§2.4.3.2: "If a subband has no bits
    /// allocated to it, the samples in that subband are set to
    /// zero").
    pub allocated: bool,
    /// Number of bits per sample (`nb`) read from §2.4.2.5; `0` for
    /// an unallocated subband.
    pub bits: u8,
    /// The raw 6-bit `scalefactor[ch][sb]` index into Table 3-B.1
    /// (§2.4.2.5). The final multiply by the Table 3-B.1 value is a
    /// spec gap — see the module docs.
    pub scalefactor_index: u8,
    /// The 12 requantized samples `s''` from the §2.4.3.2 linear
    /// formula (before the Table 3-B.1 rescale).
    pub samples: [f64; SAMPLES_PER_SUBBAND],
}

impl Default for Subband {
    fn default() -> Subband {
        Subband {
            allocated: false,
            bits: 0,
            scalefactor_index: 0,
            samples: [0.0; SAMPLES_PER_SUBBAND],
        }
    }
}

impl Subband {
    /// The §2.4.3.2 rescaled samples `s' = scalefactor[index] · s''`,
    /// applying the Table 3-B.1 multiplier
    /// ([`crate::tables::SCALEFACTORS`]) to each stored requantized
    /// value `s''`.
    ///
    /// An unallocated subband (no bits) returns all zeros — §2.4.3.2:
    /// "If a subband has no bits allocated to it, the samples in that
    /// subband are set to zero".
    pub fn rescaled_samples(&self) -> [f64; SAMPLES_PER_SUBBAND] {
        if !self.allocated {
            return [0.0; SAMPLES_PER_SUBBAND];
        }
        let factor = SCALEFACTORS[(self.scalefactor_index as usize) & 0x3F];
        let mut out = [0.0; SAMPLES_PER_SUBBAND];
        for (o, &s) in out.iter_mut().zip(self.samples.iter()) {
            *o = factor * s;
        }
        out
    }
}

/// The requantized subband samples for one Layer I frame: one
/// [`Subband`] per (channel, subband), `nch` channels × 32 subbands
/// (§2.4.1.5 / §2.4.3.2).
#[derive(Debug, Clone, PartialEq)]
pub struct SubbandSamples {
    /// Number of channels actually decoded (1 for single_channel, 2
    /// otherwise).
    pub channels: usize,
    /// `[channel][subband]` decoded subbands. Only the first
    /// `channels` rows are meaningful.
    pub subbands: [[Subband; SUBBANDS]; 2],
}

impl SubbandSamples {
    fn empty(channels: usize) -> SubbandSamples {
        SubbandSamples {
            channels,
            subbands: [[Subband::default(); SUBBANDS]; 2],
        }
    }

    /// A silent frame: `channels` channels, every subband unallocated
    /// (all-zero requantized samples). Feeding this through the
    /// synthesis filterbank produces silence for the new samples while
    /// the overlap-add history rings out — the §2.4.3.1 "muting"
    /// concealment shape.
    pub fn silent(channels: usize) -> SubbandSamples {
        SubbandSamples::empty(channels.clamp(1, 2))
    }

    /// The 32 rescaled subband samples `s'` for channel `ch` and
    /// sample-slot `slot` (`slot < `[`SAMPLES_PER_SUBBAND`]`), laid out
    /// as `[subband 0 .. subband 31]` ready to feed the §2.4.3.2
    /// synthesis filterbank ([`crate::synthesis::SynthesisFilter`]).
    ///
    /// Each entry applies the Table 3-B.1 rescale (see
    /// [`Subband::rescaled_samples`]); unallocated subbands contribute
    /// zero.
    pub fn slot(&self, ch: usize, slot: usize) -> [f64; SUBBANDS] {
        let mut out = [0.0f64; SUBBANDS];
        for (sb, o) in out.iter_mut().enumerate() {
            let band = &self.subbands[ch][sb];
            if band.allocated {
                let factor = SCALEFACTORS[(band.scalefactor_index as usize) & 0x3F];
                *o = factor * band.samples[slot];
            }
        }
        out
    }
}

/// The intensity_stereo `bound` for a frame: the first subband
/// whose allocation is shared between the two channels (§2.4.1.5).
///
/// For joint_stereo the bound comes from `mode_extension`
/// (§2.4.2.3); for every other mode the whole spectrum is
/// per-channel, i.e. the bound equals 32 (`bound == SUBBANDS`).
fn stereo_bound(header: &FrameHeader) -> usize {
    match header.mode {
        Mode::JointStereo => header.mode_extension.bound() as usize,
        // Stereo / dual_channel / single_channel: no shared band.
        _ => SUBBANDS,
    }
}

/// Decode the Layer I `audio_data()` (§2.4.1.5 / §2.4.3.2) for one
/// frame from `data`, the bitstream slice that begins immediately
/// after the header (and the 16-bit CRC word, when present).
///
/// Produces 32 × 12 requantized subband samples per channel. The
/// channel count and the joint_stereo `bound` are taken from
/// `header`. The returned samples carry the §2.4.3.2 requantized
/// value `s''`; the final rescale by Table 3-B.1 is a spec gap (see
/// module docs).
///
/// Errors if a field runs past the end of `data`
/// ([`DecodeError::UnexpectedEnd`]) or an allocation field is the
/// invalid value `15` ([`DecodeError::InvalidAllocation`]).
// The audio_data() reads of §2.4.1.5 happen in a strict spec-mandated
// order (allocation, then scalefactors, then 12 sample passes), and
// each pass writes into the parallel `nbits` and `out.subbands`
// arrays at the same `[ch][sb]` index. Index-based loops are the
// faithful expression of that nested syntax; rewriting them as
// iterator zips of two simultaneously-borrowed arrays would obscure
// the §2.4.1.5 structure without changing behaviour.
#[allow(clippy::needless_range_loop)]
pub fn decode_audio_data(header: &FrameHeader, data: &[u8]) -> Result<SubbandSamples, DecodeError> {
    let nch = header.channels() as usize;
    let bound = stereo_bound(header).min(SUBBANDS);
    let mut reader = BitReader::new(data);
    let mut out = SubbandSamples::empty(nch);

    // --- allocation[ch][sb] (§2.4.1.5) -------------------------
    // Per-channel bit width for each (ch, sb); 0 means unallocated.
    let mut nbits = [[0u8; SUBBANDS]; 2];
    // Low band [0, bound): one allocation per channel.
    for sb in 0..bound {
        for ch in 0..nch {
            let alloc = reader.read_bits(4)? as u8;
            let nb = allocation_bits(alloc).ok_or(DecodeError::InvalidAllocation {
                channel: ch,
                subband: sb,
            })?;
            nbits[ch][sb] = nb;
            out.subbands[ch][sb].allocated = nb != 0;
            out.subbands[ch][sb].bits = nb;
        }
    }
    // Upper band [bound, 32): one allocation read, shared by both
    // channels (§2.4.1.5: `allocation[1][sb] = allocation[0][sb]`).
    for sb in bound..SUBBANDS {
        let alloc = reader.read_bits(4)? as u8;
        let nb = allocation_bits(alloc).ok_or(DecodeError::InvalidAllocation {
            channel: 0,
            subband: sb,
        })?;
        for ch in 0..nch {
            nbits[ch][sb] = nb;
            out.subbands[ch][sb].allocated = nb != 0;
            out.subbands[ch][sb].bits = nb;
        }
    }

    // --- scalefactor[ch][sb] (§2.4.1.5) ------------------------
    // For every subband with non-zero allocation, six bits.
    // Iterate sb-major, ch-minor exactly as §2.4.1.5 lists.
    for sb in 0..SUBBANDS {
        for ch in 0..nch {
            if nbits[ch][sb] != 0 {
                // In the shared upper band only allocation[0] drove
                // the read; the syntax still reads one scalefactor
                // per channel for sb<bound but, for sb>=bound, the
                // `if (allocation[ch][sb]!=0)` guard holds for both
                // channels because allocation was copied. The §2.4.1.5
                // scalefactor loop runs over all 32 subbands and both
                // channels, so each allocated (ch,sb) reads its own
                // six bits.
                let scf = reader.read_bits(6)? as u8;
                out.subbands[ch][sb].scalefactor_index = scf;
            }
        }
    }

    // --- sample[ch][sb][s] (§2.4.1.5 / §2.4.3.2) ---------------
    // 12 samples, outermost loop over s, then the low band per
    // channel, then the upper (shared) band read once and copied.
    for s in 0..SAMPLES_PER_SUBBAND {
        for sb in 0..bound {
            for ch in 0..nch {
                let nb = nbits[ch][sb];
                if nb != 0 {
                    let code = reader.read_bits(nb)?;
                    out.subbands[ch][sb].samples[s] = requantize(code, nb);
                }
            }
        }
        for sb in bound..SUBBANDS {
            let nb = nbits[0][sb];
            if nb != 0 {
                let code = reader.read_bits(nb)?;
                let value = requantize(code, nb);
                // Intensity_stereo: copy the one decoded sample to
                // both channels (§2.4.3.2).
                for ch in 0..nch {
                    out.subbands[ch][sb].samples[s] = value;
                }
            }
        }
    }

    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::{Bitrate, Emphasis, Id, ModeExtension};

    /// Build a Layer I frame header struct directly (no bitstream
    /// round-trip needed for the decode tests).
    fn header(mode: Mode, mode_ext: u8) -> FrameHeader {
        FrameHeader {
            id: Id::Mpeg,
            protection: true, // no CRC
            bitrate: Bitrate::Fixed(256),
            sampling_frequency: 48_000,
            padding: false,
            private: false,
            mode,
            mode_extension: ModeExtension(mode_ext),
            copyright: false,
            original: true,
            emphasis: Emphasis::None,
        }
    }

    // ---- §2.4.2.5 allocation -> bits table --------------------

    #[test]
    fn allocation_bits_table() {
        assert_eq!(allocation_bits(0), Some(0)); // no samples
        for idx in 1u8..=14 {
            assert_eq!(allocation_bits(idx), Some(idx + 1), "alloc {idx}");
        }
        assert_eq!(allocation_bits(15), None); // invalid
    }

    // ---- BitReader -------------------------------------------

    #[test]
    fn bit_reader_msb_first() {
        // 0b1010_0110, 0b1100_0000
        let mut r = BitReader::new(&[0xA6, 0xC0]);
        assert_eq!(r.read_bits(4).unwrap(), 0b1010);
        assert_eq!(r.read_bits(4).unwrap(), 0b0110);
        assert_eq!(r.read_bits(2).unwrap(), 0b11);
        assert_eq!(r.bits_read(), 10);
    }

    #[test]
    fn bit_reader_spans_bytes() {
        let mut r = BitReader::new(&[0b1111_0000, 0b1111_0000]);
        // read 6 bits then 6 bits crosses the byte boundary.
        assert_eq!(r.read_bits(6).unwrap(), 0b111100);
        assert_eq!(r.read_bits(6).unwrap(), 0b001111);
    }

    #[test]
    fn bit_reader_zero_bits() {
        let mut r = BitReader::new(&[0xFF]);
        assert_eq!(r.read_bits(0).unwrap(), 0);
        assert_eq!(r.bits_read(), 0);
    }

    #[test]
    fn bit_reader_runs_out() {
        let mut r = BitReader::new(&[0xFF]);
        assert_eq!(r.read_bits(8).unwrap(), 0xFF);
        assert_eq!(r.read_bits(1), Err(DecodeError::UnexpectedEnd));
    }

    // ---- §2.4.3.2 requantization ------------------------------

    /// The §2.4.3.2 requantization values for nb = 2, derived
    /// directly from the formula (no external source):
    /// after MSB inversion and the linear formula, the four codes
    /// map to {-2/3, 0, 2/3, 4/3}.
    #[test]
    fn requantize_nb2() {
        let got: Vec<f64> = (0..4).map(|c| requantize(c, 2)).collect();
        let want = [-2.0 / 3.0, 0.0, 2.0 / 3.0, 4.0 / 3.0];
        for (g, w) in got.iter().zip(want.iter()) {
            assert!((g - w).abs() < 1e-12, "got {g}, want {w}");
        }
    }

    #[test]
    fn requantize_nb3_midpoint_is_zero() {
        // For nb=3 the inverted-MSB midpoint (code 4 -> inverted 0)
        // lands at fraction 0; the +2^(-nb+1) offset shifts it.
        // The formula gives 2.0/7.0 at the zero-fraction point.
        assert!((requantize(4, 3) - 2.0 / 7.0).abs() < 1e-12);
        // The most negative code (0) maps to -6/7.
        assert!((requantize(0, 3) - (-6.0 / 7.0)).abs() < 1e-12);
        // The most positive code (7) maps to +8/7.
        assert!((requantize(7, 3) - 8.0 / 7.0).abs() < 1e-12);
    }

    #[test]
    fn requantize_nb4_endpoints() {
        // nb=4: most-negative code 0 -> -14/15, code 7 -> 0,
        // most-positive code 15 -> 16/15.
        assert!((requantize(0, 4) - (-14.0 / 15.0)).abs() < 1e-12);
        assert!((requantize(7, 4) - 0.0).abs() < 1e-12);
        assert!((requantize(15, 4) - 16.0 / 15.0).abs() < 1e-12);
    }

    #[test]
    fn requantize_is_monotonic_in_code() {
        // The linear formula is strictly increasing in the code.
        for nb in 2u8..=15 {
            let mut prev = f64::NEG_INFINITY;
            for code in 0..(1u32 << nb) {
                let v = requantize(code, nb);
                assert!(v > prev, "nb={nb} code={code} not increasing");
                prev = v;
            }
        }
    }

    // ---- decode_audio_data, mono ------------------------------

    /// A single-channel frame where subband 0 has allocation 1
    /// (nb=2) and every other subband has allocation 0 (no samples).
    /// We construct the exact bitstream by hand, MSB-first, per
    /// §2.4.1.5.
    #[test]
    fn decode_mono_one_subband() {
        // allocation: sb0 ch0 = 0b0001 (nb=2); sb1..31 ch0 = 0b0000.
        // scalefactor: only sb0 allocated -> 6 bits.
        // samples: 12 samples of 2 bits each, only sb0.
        let mut bits: Vec<u8> = Vec::new();
        let mut acc: u32 = 0;
        let mut nbits_acc = 0u8;
        let push = |val: u32, n: u8, bits: &mut Vec<u8>, acc: &mut u32, nbits_acc: &mut u8| {
            for i in (0..n).rev() {
                let b = (val >> i) & 1;
                *acc = (*acc << 1) | b;
                *nbits_acc += 1;
                if *nbits_acc == 8 {
                    bits.push(*acc as u8);
                    *acc = 0;
                    *nbits_acc = 0;
                }
            }
        };
        // 32 allocation fields, 4 bits each.
        push(0b0001, 4, &mut bits, &mut acc, &mut nbits_acc); // sb0 nb=2
        for _ in 1..SUBBANDS {
            push(0b0000, 4, &mut bits, &mut acc, &mut nbits_acc);
        }
        // one scalefactor (sb0) - index 17.
        push(17, 6, &mut bits, &mut acc, &mut nbits_acc);
        // 12 samples for sb0, each 2 bits. Use the pattern
        // 0,1,2,3,0,1,2,3,0,1,2,3.
        let sample_codes: [u32; 12] = [0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3];
        for &c in &sample_codes {
            push(c, 2, &mut bits, &mut acc, &mut nbits_acc);
        }
        // flush remaining bits (left-aligned in the final byte).
        if nbits_acc > 0 {
            acc <<= 8 - nbits_acc;
            bits.push(acc as u8);
        }

        let h = header(Mode::SingleChannel, 0);
        let s = decode_audio_data(&h, &bits).expect("decode");
        assert_eq!(s.channels, 1);
        // sb0 allocated, nb=2, scf index 17.
        assert!(s.subbands[0][0].allocated);
        assert_eq!(s.subbands[0][0].bits, 2);
        assert_eq!(s.subbands[0][0].scalefactor_index, 17);
        // samples match the requant of each code at nb=2.
        for (i, &c) in sample_codes.iter().enumerate() {
            let want = requantize(c, 2);
            assert!((s.subbands[0][0].samples[i] - want).abs() < 1e-12);
        }
        // every other subband is unallocated and zero.
        for sb in 1..SUBBANDS {
            assert!(!s.subbands[0][sb].allocated);
            assert_eq!(s.subbands[0][sb].samples, [0.0; SAMPLES_PER_SUBBAND]);
        }
    }

    #[test]
    fn decode_rejects_invalid_allocation() {
        // First allocation field = 0b1111 (invalid).
        let data = [0xF0u8; 64];
        let h = header(Mode::SingleChannel, 0);
        assert_eq!(
            decode_audio_data(&h, &data),
            Err(DecodeError::InvalidAllocation {
                channel: 0,
                subband: 0
            })
        );
    }

    #[test]
    fn decode_truncated_is_error() {
        // Only one byte: the allocation reads will run out.
        let h = header(Mode::SingleChannel, 0);
        assert_eq!(
            decode_audio_data(&h, &[0x10]),
            Err(DecodeError::UnexpectedEnd)
        );
    }

    /// An all-zero-allocation mono frame: 32 allocation nibbles of
    /// 0, no scalefactors, no samples. Every subband is zeroed.
    #[test]
    fn decode_all_unallocated() {
        // 32 * 4 = 128 bits = 16 bytes, all zero.
        let data = [0u8; 16];
        let h = header(Mode::SingleChannel, 0);
        let s = decode_audio_data(&h, &data).expect("decode");
        for sb in 0..SUBBANDS {
            assert!(!s.subbands[0][sb].allocated);
            assert_eq!(s.subbands[0][sb].bits, 0);
            assert_eq!(s.subbands[0][sb].samples, [0.0; SAMPLES_PER_SUBBAND]);
        }
    }

    // ---- joint-stereo bound (§2.4.1.5 upper-band sharing) -----

    #[test]
    fn joint_stereo_bound_from_mode_extension() {
        // mode_extension 0b01 -> bound 8.
        let h = header(Mode::JointStereo, 0b01);
        assert_eq!(stereo_bound(&h), 8);
        // stereo / dual / mono -> full 32.
        assert_eq!(stereo_bound(&header(Mode::Stereo, 0)), 32);
        assert_eq!(stereo_bound(&header(Mode::DualChannel, 0)), 32);
        assert_eq!(stereo_bound(&header(Mode::SingleChannel, 0)), 32);
    }

    /// A joint-stereo frame with bound 4: subbands [0,4) carry a
    /// per-channel allocation, [4,32) share one allocation copied to
    /// both channels and the sample stream is read once and copied
    /// (§2.4.3.2 intensity_stereo). We allocate only sb4 (in the
    /// shared band) with nb=2 and confirm both channels get the same
    /// requantized samples.
    #[test]
    fn decode_joint_stereo_shared_upper_band() {
        let mut bits: Vec<u8> = Vec::new();
        let mut acc: u32 = 0;
        let mut nbits_acc = 0u8;
        let push = |val: u32, n: u8, bits: &mut Vec<u8>, acc: &mut u32, nbits_acc: &mut u8| {
            for i in (0..n).rev() {
                let b = (val >> i) & 1;
                *acc = (*acc << 1) | b;
                *nbits_acc += 1;
                if *nbits_acc == 8 {
                    bits.push(*acc as u8);
                    *acc = 0;
                    *nbits_acc = 0;
                }
            }
        };
        // bound = 4 (mode_extension 0b00). Low band [0,4): per
        // channel, all 0. That's 4 subbands * 2 channels * 4 bits.
        for _ in 0..4 {
            for _ in 0..2 {
                push(0b0000, 4, &mut bits, &mut acc, &mut nbits_acc);
            }
        }
        // Upper band [4,32): one allocation each. sb4 = 0b0001
        // (nb=2), the rest 0.
        push(0b0001, 4, &mut bits, &mut acc, &mut nbits_acc); // sb4
        for _ in 5..SUBBANDS {
            push(0b0000, 4, &mut bits, &mut acc, &mut nbits_acc);
        }
        // scalefactor loop (sb-major, ch-minor over all 32 subbands).
        // Only sb4 is allocated; allocation was copied to both
        // channels, so the §2.4.1.5 scalefactor loop reads one
        // scalefactor per channel for sb4 -> two reads.
        push(20, 6, &mut bits, &mut acc, &mut nbits_acc); // sb4 ch0
        push(21, 6, &mut bits, &mut acc, &mut nbits_acc); // sb4 ch1
                                                          // samples: 12 samples, sb4 read once (shared), 2 bits each.
        let sample_codes: [u32; 12] = [3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0];
        for &c in &sample_codes {
            push(c, 2, &mut bits, &mut acc, &mut nbits_acc);
        }
        if nbits_acc > 0 {
            acc <<= 8 - nbits_acc;
            bits.push(acc as u8);
        }

        let h = header(Mode::JointStereo, 0b00); // bound 4
        let s = decode_audio_data(&h, &bits).expect("decode");
        assert_eq!(s.channels, 2);
        // sb4 allocated in both channels, nb=2.
        assert!(s.subbands[0][4].allocated);
        assert!(s.subbands[1][4].allocated);
        assert_eq!(s.subbands[0][4].bits, 2);
        assert_eq!(s.subbands[1][4].bits, 2);
        // distinct scalefactor indices were read per channel.
        assert_eq!(s.subbands[0][4].scalefactor_index, 20);
        assert_eq!(s.subbands[1][4].scalefactor_index, 21);
        // the requantized sample stream is identical in both channels
        // (intensity_stereo copy, §2.4.3.2).
        for (i, &code) in sample_codes.iter().enumerate() {
            let want = requantize(code, 2);
            assert!((s.subbands[0][4].samples[i] - want).abs() < 1e-12);
            assert_eq!(s.subbands[0][4].samples[i], s.subbands[1][4].samples[i]);
        }
    }

    // ---- stereo (two independent channels) --------------------

    /// A plain-stereo frame (bound 32): each of the two channels has
    /// its own allocation. We allocate sb0 ch0 (nb=2) and sb0 ch1
    /// (nb=3) and confirm the two channels decode independently.
    #[test]
    fn decode_stereo_independent_channels() {
        let mut bits: Vec<u8> = Vec::new();
        let mut acc: u32 = 0;
        let mut nbits_acc = 0u8;
        let push = |val: u32, n: u8, bits: &mut Vec<u8>, acc: &mut u32, nbits_acc: &mut u8| {
            for i in (0..n).rev() {
                let b = (val >> i) & 1;
                *acc = (*acc << 1) | b;
                *nbits_acc += 1;
                if *nbits_acc == 8 {
                    bits.push(*acc as u8);
                    *acc = 0;
                    *nbits_acc = 0;
                }
            }
        };
        // bound = 32 -> all 32 subbands per channel. sb0: ch0=0b0001
        // (nb=2), ch1=0b0010 (nb=3). Remaining subbands 0.
        push(0b0001, 4, &mut bits, &mut acc, &mut nbits_acc); // sb0 ch0
        push(0b0010, 4, &mut bits, &mut acc, &mut nbits_acc); // sb0 ch1
        for _ in 1..SUBBANDS {
            push(0b0000, 4, &mut bits, &mut acc, &mut nbits_acc); // ch0
            push(0b0000, 4, &mut bits, &mut acc, &mut nbits_acc); // ch1
        }
        // scalefactors: sb0 ch0, then sb0 ch1.
        push(5, 6, &mut bits, &mut acc, &mut nbits_acc); // ch0
        push(9, 6, &mut bits, &mut acc, &mut nbits_acc); // ch1
                                                         // samples: for each s, sb0 ch0 (2 bits) then sb0 ch1 (3 bits).
        let codes_ch0: [u32; 12] = [0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3];
        let codes_ch1: [u32; 12] = [7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4];
        for s in 0..12 {
            push(codes_ch0[s], 2, &mut bits, &mut acc, &mut nbits_acc);
            push(codes_ch1[s], 3, &mut bits, &mut acc, &mut nbits_acc);
        }
        if nbits_acc > 0 {
            acc <<= 8 - nbits_acc;
            bits.push(acc as u8);
        }

        let h = header(Mode::Stereo, 0);
        let s = decode_audio_data(&h, &bits).expect("decode");
        assert_eq!(s.channels, 2);
        assert_eq!(s.subbands[0][0].bits, 2);
        assert_eq!(s.subbands[1][0].bits, 3);
        assert_eq!(s.subbands[0][0].scalefactor_index, 5);
        assert_eq!(s.subbands[1][0].scalefactor_index, 9);
        for i in 0..12 {
            assert!((s.subbands[0][0].samples[i] - requantize(codes_ch0[i], 2)).abs() < 1e-12);
            assert!((s.subbands[1][0].samples[i] - requantize(codes_ch1[i], 3)).abs() < 1e-12);
        }
    }
}
