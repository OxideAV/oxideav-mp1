//! MPEG-1 / MPEG-2 LSF Audio **Layer II** audio-data decode.
//!
//! Implements the Layer II portion of ISO/IEC 11172-3 (1993):
//!
//! * §2.4.1.6 `audio_data()` syntax — the per-subband nbal-bit
//!   `allocation[ch][sb]` field (indexing one of Tables 3-B.2a..d
//!   selected by `(sampling_frequency, bitrate_per_channel)`), the
//!   per-subband 2-bit `scfsi[ch][sb]`, the 1..3 scalefactor reads
//!   per (ch, sb) selected by `scfsi`, and the **12** granules of one
//!   triplet each — `for (gr=0; gr<12; gr++)` per the syntax page,
//!   each iteration emitting three sub-band samples (either one
//!   grouped `samplecode` of 5..10 bits or three separable
//!   `sample[ch][sb][3*gr+s]` reads of 3..16 bits each).
//! * §2.4.2.6 — field semantics for `allocation`, `scfsi`,
//!   `scalefactor`, `grouping`, `samplecode`, `sample`.
//! * §2.4.3.3 — the Layer II decoding process: the per-subband
//!   `nlevels` lookup via Tables 3-B.2x, the per-`nlevels`
//!   classes-of-quantization lookup via Table 3-B.4 (giving `C`,
//!   `D`, the grouping flag, the number of samples per codeword, and
//!   the bit count per codeword), the degrouping algorithm
//!   `for (i=0;i<3;i++) { s[i] = c % nlevels; c = c DIV nlevels }`,
//!   the §2.4.3.3.4 linear requantization formula
//!   `s'' = C * (s''' + D)`, and the final rescale by the Table 3-B.1
//!   multiplier.
//! * Table 3-B.1 ("Layer I, II scalefactors") — already transcribed
//!   in [`crate::tables::SCALEFACTORS`] and shared with the Layer I
//!   path.
//!
//! ## What this module produces
//!
//! For one Layer II frame this module produces **36 rescaled samples
//! per subband per channel** (`12 syntax-granules × 3 samples`); when
//! fed into the §2.4.3.2 polyphase synthesis filterbank
//! ([`crate::synthesis::SynthesisFilter`]) one slot at a time across
//! `slot in 0..36`, this yields `1152` PCM samples per channel —
//! the Layer II frame size per §2.4.2.1.
//!
//! The intensity_stereo upper band (`[bound, sblimit)` in joint_stereo
//! mode) reads one shared sample stream that is copied into both
//! channels per §2.4.2.6 / §2.4.3.3 ("For subbands in
//! intensity_stereo mode the coded representation of the samplecode
//! is valid for both channels").

use crate::decode::{BitReader, DecodeError, SUBBANDS};
use crate::header::{crc16_update_bits, CrcStatus, FrameHeader, Mode, CRC16_INIT};
use crate::tables::SCALEFACTORS;
use crate::tables_layer2::{layer2_bit_allocation_table, AllocationTable, QuantClass};

/// Number of *syntax-level* granules in a Layer II `audio_data()` —
/// the outer `for (gr=0; gr<12; gr++)` loop runs twelve times per
/// channel per subband (§2.4.1.6 PDF page 16). Each iteration carries
/// one triplet of three samples, so 12 × 3 = 36 sub-band samples per
/// subband per channel per frame.
pub const SYNTAX_GRANULES: usize = 12;
/// Three sub-band samples per granule (the `sample[..][..][3*gr+s]`
/// `s = 0..3` inner loop or the single grouped `samplecode` that
/// degroups to three samples).
pub const SAMPLES_PER_GRANULE: usize = 3;
/// Number of "scalefactor parts" the 36 subband samples are divided
/// into for scalefactor purposes (§2.4.2.6 / §2.4.3.3.2 "three equal
/// parts of 12 subband samples"). Four syntax-granules per part.
pub const SCALEFACTOR_PARTS: usize = 3;
/// 36 sub-band samples per subband per channel per Layer II frame.
pub const LAYER2_SAMPLES_PER_SUBBAND: usize = SYNTAX_GRANULES * SAMPLES_PER_GRANULE;
/// 1152 audio samples per channel per Layer II frame (§2.4.2.1):
/// `36 sub-band samples × 32 subbands = 1152` after the synthesis
/// filterbank (the filterbank emits 32 PCM samples per sub-band
/// sample-slot, so per channel: 36 slots × 32 = 1152).
pub const LAYER2_SAMPLES_PER_FRAME: usize = LAYER2_SAMPLES_PER_SUBBAND * 32;

/// One Layer II subband's decoded state for one channel: the raw
/// allocation index that was read, the up-to-three Table-3-B.1
/// scalefactor indices (per §2.4.3.3.2 / §2.4.3.3.3, one per
/// scalefactor part), and the 36 *rescaled* sub-band samples
/// (`s' = factor[scf_part(slot)] · s''` for the §2.4.3.3.4 `s''`).
///
/// `allocation == 0` (no samples transferred) leaves `samples` at all
/// zeros and the scalefactor indices unused.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Layer2Subband {
    /// Raw allocation index read from `audio_data()` for this
    /// (ch, sb). Resolved to a quantization class via the per-frame
    /// allocation table + Table 3-B.4 in [`AllocationTable::quant_class`].
    pub allocation: u8,
    /// One Table 3-B.1 scalefactor *index* per scalefactor part
    /// (part 0 covers slots 0..12, part 1 covers 12..24, part 2 covers
    /// 24..36 per §2.4.2.6).
    pub scalefactor_indices: [u8; SCALEFACTOR_PARTS],
    /// 36 rescaled sub-band sample values `s'`.
    pub samples: [f64; LAYER2_SAMPLES_PER_SUBBAND],
}

impl Default for Layer2Subband {
    fn default() -> Layer2Subband {
        Layer2Subband {
            allocation: 0,
            scalefactor_indices: [0; SCALEFACTOR_PARTS],
            samples: [0.0; LAYER2_SAMPLES_PER_SUBBAND],
        }
    }
}

/// All decoded subband samples for one Layer II frame, both channels.
#[derive(Debug, Clone)]
pub struct Layer2Subbands {
    /// Number of channels actually decoded (1 for single_channel, 2
    /// otherwise).
    pub channels: usize,
    /// `[channel][subband]` decoded subbands. Only the first
    /// `channels` rows are meaningful. Subbands at or above
    /// [`sblimit`](Self::sblimit) are not present in the bitstream and
    /// remain at the [`Layer2Subband::default`] all-zero state.
    pub subbands: Vec<[Layer2Subband; SUBBANDS]>,
    /// The `sblimit` selected by the per-frame [`AllocationTable`]
    /// (§2.4.3.3.1). Subbands `[sblimit, 32)` are forced to zero.
    pub sblimit: usize,
}

impl Layer2Subbands {
    fn empty(channels: usize, sblimit: usize) -> Layer2Subbands {
        Layer2Subbands {
            channels,
            subbands: vec![[Layer2Subband::default(); SUBBANDS]; 2],
            sblimit,
        }
    }

    /// A silent Layer II frame: `channels` channels, `sblimit = 0`
    /// (so every (sb, slot) sample is forced to zero by
    /// [`slot`](Self::slot)).
    ///
    /// Feeding this through the §2.4.3.2 synthesis filterbank produces
    /// silence for the new samples while the overlap-add history rings
    /// out — the §2.4.3.1 "muting" concealment shape for Layer II.
    pub fn silent(channels: usize) -> Layer2Subbands {
        Layer2Subbands::empty(channels.clamp(1, 2), 0)
    }

    /// The 32 sub-band sample values for channel `ch` and sample-slot
    /// `slot` (`slot < `[`LAYER2_SAMPLES_PER_SUBBAND`]), laid out as
    /// `[subband 0 .. subband 31]` ready to feed the synthesis
    /// filterbank ([`crate::synthesis::SynthesisFilter`]).
    ///
    /// Subbands at or above `sblimit` are forced to zero per
    /// §2.4.3.3.1; subbands below `sblimit` with allocation 0 also
    /// carry zero per §2.4.3.3.5 ("if a subband has no bits allocated
    /// to it, the samples in that subband are set to zero").
    pub fn slot(&self, ch: usize, slot: usize) -> [f64; SUBBANDS] {
        let mut out = [0.0f64; SUBBANDS];
        for (sb, o) in out.iter_mut().enumerate().take(self.sblimit) {
            let band = &self.subbands[ch][sb];
            if band.allocation != 0 {
                *o = band.samples[slot];
            }
        }
        out
    }
}

/// Total number of bits the Layer II §2.4.1.6 `allocation` field
/// occupies for the given header.
///
/// §2.4.1.6 reads the per-channel allocation field below the
/// `stereo_bound` and one shared allocation field in the
/// `[bound, sblimit)` upper band (intensity_stereo). Each subband's
/// allocation is `nbal(sb)` bits wide per the per-frame Table 3-B.2x.
fn layer2_allocation_field_bits(header: &FrameHeader, table: &'static AllocationTable) -> usize {
    let nch = header.channels() as usize;
    let sblimit = table.sblimit();
    let bound = stereo_bound(header, sblimit);
    let mut bits = 0usize;
    for sb in 0..bound {
        bits += (table.nbal(sb) as usize) * nch;
    }
    for sb in bound..sblimit {
        bits += table.nbal(sb) as usize;
    }
    bits
}

/// Walk the §2.4.1.6 allocation field in `data` (starting at bit `0`)
/// and return the total number of bits the §2.4.1.6 `scfsi` field
/// occupies: 2 bits per (ch, sb) below `sblimit` whose decoded
/// allocation is non-zero.
///
/// `data` is the bitstream slice that begins at the start of the
/// allocation field (i.e. `after_header[2..]` in a CRC-protected
/// frame, or `after_header` itself when no CRC is present). Returns
/// `None` if the slice is too short to contain the allocation field
/// or if an allocation index points at an empty (`-`) slot in the
/// per-subband Table 3-B.2x row.
fn layer2_scfsi_field_bits(
    header: &FrameHeader,
    table: &'static AllocationTable,
    data: &[u8],
) -> Option<usize> {
    let nch = header.channels() as usize;
    let sblimit = table.sblimit();
    let bound = stereo_bound(header, sblimit);
    let mut reader = BitReader::new(data);
    let mut bits = 0usize;
    // Low band: per-channel allocation, per-channel scfsi.
    for sb in 0..bound {
        let nbal = table.nbal(sb);
        for _ch in 0..nch {
            let alloc = reader.read_bits(nbal).ok()? as u8;
            if alloc != 0 {
                // Validate that the allocation index points at a real
                // class — an invalid index here would also wreck the
                // scalefactor/sample stage, so reject early.
                table.quant_class(sb, alloc)?;
                bits += 2;
            }
        }
    }
    // Upper band: one shared allocation; one scfsi *per channel* per
    // §2.4.1.6 (scfsi is always per-channel even in the shared
    // upper band — the syntax page lists `for(ch=0;ch<nch;ch++)`
    // around the scfsi read for every sb below sblimit).
    for sb in bound..sblimit {
        let nbal = table.nbal(sb);
        let alloc = reader.read_bits(nbal).ok()? as u8;
        if alloc != 0 {
            table.quant_class(sb, alloc)?;
            bits += 2 * nch;
        }
    }
    Some(bits)
}

/// Compute the §2.4.3.1 CRC-16 over the Layer II protected fields for a
/// frame (Annex B Table 3-B.5: header bits 16…31, bit allocation,
/// scalefactor selection information).
///
/// `header_bytes` is the four-byte frame header and `allocation_and_scfsi`
/// is the bitstream slice that begins immediately at the allocation
/// field (i.e. `after_header[2..]` for a CRC-protected frame, since the
/// CRC word sits between the header and the allocation field). Returns
/// the 16-bit CRC, or `None` if the slice is too short for the protected
/// region or contains an invalid allocation index.
pub fn compute_layer2_crc(
    header: &FrameHeader,
    header_bytes: &[u8],
    allocation_and_scfsi: &[u8],
) -> Option<u16> {
    if header_bytes.len() < 4 {
        return None;
    }
    let table = layer2_bit_allocation_table(header);
    let alloc_bits = layer2_allocation_field_bits(header, table);
    let scfsi_bits = layer2_scfsi_field_bits(header, table, allocation_and_scfsi)?;
    let total_bits = alloc_bits + scfsi_bits;
    if allocation_and_scfsi.len() * 8 < total_bits {
        return None;
    }
    let mut reg = crc16_update_bits(CRC16_INIT, &header_bytes[2..4], 16);
    reg = crc16_update_bits(reg, allocation_and_scfsi, total_bits);
    Some(reg)
}

/// Read **and verify** the §2.4.1.4 / §2.4.3.1 `error_check()` word for
/// a Layer II frame, returning the resulting [`CrcStatus`].
///
/// `header_bytes` is the four-byte frame header and `after_header` is the
/// bitstream slice that begins immediately after the four header bytes.
/// When [`FrameHeader::has_crc`] is false this returns [`CrcStatus::Absent`]
/// without consuming any bytes. When a CRC is expected the §2.4.3.1
/// CRC-16 is computed over header bits 16…31 + the §2.4.1.6 allocation
/// field + the §2.4.1.6 scfsi field (per Annex B Table 3-B.5) and
/// compared with the stored word; returns [`CrcStatus::Ok`] or
/// [`CrcStatus::Mismatch`]. Returns `None` if a CRC is expected but the
/// slice is too short to hold the CRC word plus the full protected
/// region.
pub fn verify_layer2_crc(
    header: &FrameHeader,
    header_bytes: &[u8],
    after_header: &[u8],
) -> Option<CrcStatus> {
    if !header.has_crc() {
        return Some(CrcStatus::Absent);
    }
    if header_bytes.len() < 4 || after_header.len() < 2 {
        return None;
    }
    let stored = u16::from_be_bytes([after_header[0], after_header[1]]);
    let computed = compute_layer2_crc(header, header_bytes, &after_header[2..])?;
    Some(if computed == stored {
        CrcStatus::Ok(stored)
    } else {
        CrcStatus::Mismatch { stored, computed }
    })
}

/// The intensity_stereo `bound` for a Layer II frame: the first
/// subband whose allocation is shared between channels (§2.4.1.6).
///
/// For joint_stereo the bound comes from `mode_extension` (§2.4.2.3,
/// `{4, 8, 12, 16}`). Clamped to `sblimit` since subbands at or above
/// sblimit are not in the bitstream regardless.
fn stereo_bound(header: &FrameHeader, sblimit: usize) -> usize {
    let raw = match header.mode {
        Mode::JointStereo => header.mode_extension.bound() as usize,
        _ => SUBBANDS,
    };
    raw.min(sblimit)
}

/// §2.4.3.3.4 Layer II requantization for one triplet, given the
/// quantization class (`C`, `D`, grouping flag, `nbits`) resolved from
/// the per-frame allocation table.
///
/// Returns the three §2.4.3.3.4 requantized values `s''` *before* the
/// Table 3-B.1 rescale.
fn requantize_triplet(class: &QuantClass, codes: [u32; 3]) -> [f64; 3] {
    let mut out = [0.0; 3];
    let nb = class.bits_per_sample();
    let msb = 1u32 << (nb - 1);
    for (o, c) in out.iter_mut().zip(codes.iter()) {
        // §2.4.3.3.4: "the first bit of each of the three codes has to
        // be inverted, and the resulting numbers should be regarded as
        // two's complement fractional numbers, where the MSB represents
        // the value -1".
        let inverted = c ^ msb;
        let signed = if inverted & msb != 0 {
            i64::from(inverted) - (1i64 << nb)
        } else {
            i64::from(inverted)
        };
        let s_frac = signed as f64 / (1u64 << (nb - 1)) as f64;
        // §2.4.3.3.4 linear formula: s'' = C * (s''' + D).
        *o = class.c * (s_frac + class.d);
    }
    out
}

/// Decode one Layer II `audio_data()` block (§2.4.1.6 / §2.4.3.3) from
/// `data`, the bitstream slice that begins immediately after the
/// header (and the 16-bit CRC word, when present).
///
/// `header` selects the per-frame bit-allocation table (B.2a / B.2b /
/// B.2c / B.2d), the channel count, and the joint-stereo bound; the
/// returned [`Layer2Subbands`] carries 36 rescaled samples per subband
/// per channel, ready for the §2.4.3.2 synthesis filterbank.
///
/// Errors mirror Layer I's [`DecodeError`]: [`DecodeError::UnexpectedEnd`]
/// when a field runs past `data` and [`DecodeError::InvalidAllocation`]
/// when an allocation index points at an empty (`-`) slot in the
/// per-subband row of the chosen Tables 3-B.2x.
// The §2.4.1.6 syntax is a strict spec-mandated sequence of allocation
// reads, then scfsi reads, then scalefactor reads, then 12 granules of
// per-(sb, ch) triplet reads. Index-based loops are the faithful
// expression of that nested syntax.
#[allow(clippy::needless_range_loop)]
pub fn decode_layer2_audio_data(
    header: &FrameHeader,
    data: &[u8],
) -> Result<Layer2Subbands, DecodeError> {
    let nch = header.channels() as usize;
    let table = layer2_bit_allocation_table(header);
    let sblimit = table.sblimit();
    let bound = stereo_bound(header, sblimit);

    let mut reader = BitReader::new(data);
    let mut out = Layer2Subbands::empty(nch, sblimit);

    // --- allocation[ch][sb] (§2.4.1.6) ----------------------------
    // Low band [0, bound): per-channel nbal-bit allocation.
    for sb in 0..bound {
        let nbal = table.nbal(sb);
        for ch in 0..nch {
            let alloc = reader.read_bits(nbal)? as u8;
            if alloc != 0 && table.quant_class(sb, alloc).is_none() {
                return Err(DecodeError::InvalidAllocation {
                    channel: ch,
                    subband: sb,
                });
            }
            out.subbands[ch][sb].allocation = alloc;
        }
    }
    // Upper band [bound, sblimit): one allocation read, shared by both
    // channels (intensity_stereo).
    for sb in bound..sblimit {
        let nbal = table.nbal(sb);
        let alloc = reader.read_bits(nbal)? as u8;
        if alloc != 0 && table.quant_class(sb, alloc).is_none() {
            return Err(DecodeError::InvalidAllocation {
                channel: 0,
                subband: sb,
            });
        }
        for ch in 0..nch {
            out.subbands[ch][sb].allocation = alloc;
        }
    }

    // --- scfsi[ch][sb] (§2.4.1.6) ---------------------------------
    // 2 bits per (ch, sb) for every subband below sblimit with a
    // nonzero allocation. In the shared upper band the allocation was
    // copied to both channels, so the §2.4.1.6 `for (ch=0; ch<nch; ch++)`
    // loop still reads one scfsi per channel.
    let mut scfsi = [[0u8; SUBBANDS]; 2];
    for sb in 0..sblimit {
        for ch in 0..nch {
            if out.subbands[ch][sb].allocation != 0 {
                scfsi[ch][sb] = reader.read_bits(2)? as u8;
            }
        }
    }

    // --- scalefactor[ch][sb][p] (§2.4.1.6 / §2.4.3.3.2-3) ---------
    // §2.4.2.6 scfsi → scalefactor schedule:
    //   '00' : three scfs, one per part 0/1/2.
    //   '01' : two scfs, first for parts 0+1, second for part 2.
    //   '10' : one scf, valid for all three parts.
    //   '11' : two scfs, first for part 0, second for parts 1+2.
    for sb in 0..sblimit {
        for ch in 0..nch {
            if out.subbands[ch][sb].allocation == 0 {
                continue;
            }
            let selector = scfsi[ch][sb];
            let band = &mut out.subbands[ch][sb];
            match selector {
                0b00 => {
                    band.scalefactor_indices[0] = reader.read_bits(6)? as u8;
                    band.scalefactor_indices[1] = reader.read_bits(6)? as u8;
                    band.scalefactor_indices[2] = reader.read_bits(6)? as u8;
                }
                0b01 => {
                    let s0 = reader.read_bits(6)? as u8;
                    let s2 = reader.read_bits(6)? as u8;
                    band.scalefactor_indices = [s0, s0, s2];
                }
                0b11 => {
                    let s0 = reader.read_bits(6)? as u8;
                    let s12 = reader.read_bits(6)? as u8;
                    band.scalefactor_indices = [s0, s12, s12];
                }
                _ => {
                    // scfsi == '10': a single scf for every part.
                    let s = reader.read_bits(6)? as u8;
                    band.scalefactor_indices = [s, s, s];
                }
            }
        }
    }

    // --- samples (§2.4.1.6 / §2.4.3.3.4) --------------------------
    // 12 syntax-granules of one triplet per (sb, ch). For each granule:
    //   low band: per-channel triplet reads,
    //   upper band: one shared triplet read mirrored into both channels.
    for gr in 0..SYNTAX_GRANULES {
        for sb in 0..bound {
            for ch in 0..nch {
                read_and_store_triplet(
                    &mut reader,
                    &mut out,
                    table,
                    sb,
                    ch,
                    gr,
                    /*shared=*/ false,
                )?;
            }
        }
        for sb in bound..sblimit {
            read_and_store_triplet(
                &mut reader,
                &mut out,
                table,
                sb,
                /*ch=*/ 0,
                gr,
                /*shared=*/ nch == 2,
            )?;
        }
    }

    Ok(out)
}

/// One triplet read: either ONE grouped `samplecode` (3 samples in a
/// single integer code) or THREE separable `sample` reads, per the
/// grouping flag in the per-subband quantization class
/// (§2.4.3.3.4). Stores the three rescaled samples into the matching
/// part slot of `out.subbands[ch][sb]`. When `shared` is `true` the
/// same three samples are mirrored into channel 1 (intensity_stereo).
fn read_and_store_triplet(
    reader: &mut BitReader<'_>,
    out: &mut Layer2Subbands,
    table: &AllocationTable,
    sb: usize,
    ch: usize,
    triplet_index: usize,
    shared: bool,
) -> Result<(), DecodeError> {
    let alloc = out.subbands[ch][sb].allocation;
    if alloc == 0 {
        return Ok(());
    }
    let class = table
        .quant_class(sb, alloc)
        .ok_or(DecodeError::InvalidAllocation {
            channel: ch,
            subband: sb,
        })?;

    let codes = if class.grouping {
        // §2.4.3.3.4 degroup: c % nlevels, then c /= nlevels, three times.
        let mut c = reader.read_bits(class.bits_per_codeword)?;
        let n = class.nlevels as u32;
        let mut s = [0u32; 3];
        for code in s.iter_mut() {
            *code = c % n;
            c /= n;
        }
        s
    } else {
        let nb = class.bits_per_codeword;
        [
            reader.read_bits(nb)?,
            reader.read_bits(nb)?,
            reader.read_bits(nb)?,
        ]
    };

    let s_dp = requantize_triplet(class, codes);

    // Each of the 12 syntax-granules contributes one triplet (3
    // sub-band samples) to a 36-sample part of the frame. The frame is
    // divided into 3 scalefactor parts of 12 sub-band samples each
    // (§2.4.2.6); 4 triplets fall in each part.
    let part = triplet_index / 4;
    let base = triplet_index * 3;

    {
        let band = &mut out.subbands[ch][sb];
        let scf_idx = band.scalefactor_indices[part] as usize & 0x3F;
        let factor = SCALEFACTORS[scf_idx];
        for (i, &v) in s_dp.iter().enumerate() {
            band.samples[base + i] = factor * v;
        }
    }
    if shared {
        let other = 1 - ch;
        if other < out.channels {
            let band = &mut out.subbands[other][sb];
            let scf_idx = band.scalefactor_indices[part] as usize & 0x3F;
            let factor = SCALEFACTORS[scf_idx];
            for (i, &v) in s_dp.iter().enumerate() {
                band.samples[base + i] = factor * v;
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::{Bitrate, Emphasis, Id, Layer, ModeExtension};

    fn mono_header(sampling_frequency: u32, kbps: u16) -> FrameHeader {
        FrameHeader {
            id: Id::Mpeg,
            layer: Layer::II,
            protection: true,
            bitrate: Bitrate::Fixed(kbps),
            sampling_frequency,
            padding: false,
            private: false,
            mode: Mode::SingleChannel,
            mode_extension: ModeExtension(0),
            copyright: false,
            original: true,
            emphasis: Emphasis::None,
        }
    }

    /// All-zero allocation field: every subband ends up with
    /// `allocation == 0` and the decoder reads exactly
    /// `sum_of_nbal` bits without proceeding into scfsi /
    /// scalefactor / sample reads.
    #[test]
    fn all_unallocated_mono_44k1_b2a() {
        // 64 kbit/s mono at 44.1 kHz selects B.2a (sblimit=27, sum of
        // nbal = 88 bits = 11 bytes). Zero-fill those 11 bytes and the
        // decoder must succeed with every subband at allocation 0.
        let h = mono_header(44_100, 64);
        let data = vec![0u8; 32];
        let s = decode_layer2_audio_data(&h, &data).expect("decode");
        assert_eq!(s.channels, 1);
        assert_eq!(s.sblimit, 27);
        for sb in 0..32 {
            assert_eq!(s.subbands[0][sb].allocation, 0);
            for &v in s.subbands[0][sb].samples.iter() {
                assert_eq!(v, 0.0);
            }
        }
    }

    #[test]
    fn requantize_triplet_endpoints_nbits3() {
        // The Table B.4 row "nlevels=7, C=8/7, D=0.25, grouping=no,
        // 1 sample/codeword, 3 bits/codeword" — its nbits is the
        // standalone sample width.
        let class = QuantClass {
            nlevels: 7,
            c: 8.0 / 7.0,
            d: 2f64.powi(-2),
            grouping: false,
            samples_per_codeword: 1,
            bits_per_codeword: 3,
        };
        let out = requantize_triplet(&class, [0, 4, 7]);
        // 0 inverted is 4 (msb 100), signed = 4-8 = -4 → frac -1 →
        // s'' = (8/7)*(-1 + 0.25) = -6/7.
        assert!((out[0] - (-6.0 / 7.0)).abs() < 1e-12, "got {}", out[0]);
        // 4 inverted is 0 → s'' = (8/7)*0.25 = 2/7.
        assert!((out[1] - (2.0 / 7.0)).abs() < 1e-12, "got {}", out[1]);
        // 7 inverted is 3 → frac 3/4 → s'' = (8/7)*1.0 = 8/7.
        assert!((out[2] - (8.0 / 7.0)).abs() < 1e-12, "got {}", out[2]);
    }

    // ---- §2.4.3.1 + Table 3-B.5 CRC walk -------------------------

    /// Pack a sequence of (bit_width, value) pairs MSB-first into a
    /// byte vector — the same MSB-first layout the §2.4.1.6 bitstream
    /// uses for `allocation` / `scfsi` / `scalefactor` / `sample`
    /// fields. Final partial byte is zero-padded on the low side.
    fn pack_msb_first(fields: &[(u8, u32)]) -> Vec<u8> {
        let mut bytes = Vec::new();
        let mut acc: u32 = 0;
        let mut nbits: u8 = 0;
        for &(w, v) in fields {
            for i in (0..w).rev() {
                let b = (v >> i) & 1;
                acc = (acc << 1) | b;
                nbits += 1;
                if nbits == 8 {
                    bytes.push(acc as u8);
                    acc = 0;
                    nbits = 0;
                }
            }
        }
        if nbits > 0 {
            acc <<= 8 - nbits;
            bytes.push(acc as u8);
        }
        bytes
    }

    /// Build a CRC-protected mono Layer II header at 64 kbit/s, 44.1
    /// kHz (selects Table 3-B.2a, sblimit = 27). Returns the four-byte
    /// header word.
    fn protected_mono_l2_header() -> [u8; 4] {
        // Layer II (layer 0b10), MPEG-1 (ID 1), protection=0,
        // bitrate_index 0b0100 (64 kbit/s on the Layer II MPEG-1
        // ladder), 44.1 kHz (0b00 — implicit zero), no padding,
        // single_channel (0b11), mode_ext 0, no copyright, original,
        // no emphasis. Zero-valued fields contribute nothing to the
        // OR and are omitted (clippy `identity_op`).
        let word: u32 = (0xFFF << 20)
            | (1 << 19)        // ID = MPEG-1
            | (0b10 << 17)     // layer II
            // protection bit (16) = 0 -> CRC present
            | (0b0100 << 12)   // bitrate index 0b0100 -> 64 kbit/s on L2 MPEG-1 ladder
            // sampling_frequency (10..11) = 0b00 -> 44.1 kHz (omitted)
            | (0b11 << 6)      // single_channel
            | (1 << 2); // original
        word.to_be_bytes()
    }

    #[test]
    fn allocation_field_bits_mono_b2a() {
        // 64 kbit/s mono @ 44.1 kHz -> B.2a (sum_of_nbal = 88, sblimit
        // = 27). Mono single_channel: bound = sblimit, so allocation
        // bits = 1 * sum_of_nbal = 88.
        let header = protected_mono_l2_header();
        let h = FrameHeader::parse(&header).unwrap();
        let table = layer2_bit_allocation_table(&h);
        assert_eq!(table.sblimit(), 27);
        assert_eq!(layer2_allocation_field_bits(&h, table), 88);
    }

    #[test]
    fn scfsi_field_bits_zero_when_no_subband_allocated() {
        // All-zero allocation field: scfsi field carries zero bits.
        let header = protected_mono_l2_header();
        let h = FrameHeader::parse(&header).unwrap();
        let table = layer2_bit_allocation_table(&h);
        // 88 allocation bits = 11 bytes; pad some trailing room.
        let data = vec![0u8; 32];
        assert_eq!(layer2_scfsi_field_bits(&h, table, &data), Some(0));
    }

    #[test]
    fn scfsi_field_bits_counts_each_allocated_subband() {
        // 64 kbit/s mono @ 44.1 kHz selects B.2a. Build an allocation
        // field with three subbands allocated (sb0 nbal=4 -> alloc 1,
        // sb1 nbal=4 -> alloc 0, sb2 nbal=4 -> alloc 2, then zeros).
        // Each allocated subband contributes 2 scfsi bits, so 3
        // allocated subbands -> 6 scfsi bits.
        let header = protected_mono_l2_header();
        let h = FrameHeader::parse(&header).unwrap();
        let table = layer2_bit_allocation_table(&h);
        // First three subbands have nbal=4. Build sb0=1, sb1=0, sb2=2,
        // then all-zero for the remaining 24 subbands (nbal mixes 4/3/2).
        let mut fields: Vec<(u8, u32)> = vec![(4, 1), (4, 0), (4, 2)];
        for sb in 3..table.sblimit() {
            fields.push((table.nbal(sb), 0));
        }
        let bytes = pack_msb_first(&fields);
        assert_eq!(layer2_scfsi_field_bits(&h, table, &bytes), Some(4));
    }

    #[test]
    fn layer2_crc_round_trip_compute_then_verify_ok() {
        // Synthesise a protected mono Layer II frame, compute the CRC
        // over its allocation+scfsi region with `compute_layer2_crc`,
        // store the CRC right after the header, and confirm
        // `verify_layer2_crc` reports Ok.
        let header = protected_mono_l2_header();
        let h = FrameHeader::parse(&header).unwrap();
        let table = layer2_bit_allocation_table(&h);
        // Build a deliberate allocation field: sb0 nbal=4 -> 1
        // (nlevels=3, grouped); sb1 nbal=4 -> 2 (nlevels=5, grouped);
        // remaining subbands zero. The two scfsi reads (sb0, sb1) both
        // take selector 0b00 (three scalefactors), but only the
        // allocation + scfsi region is CRC-protected so we pick the
        // smallest valid scfsi pattern (all zeros) and pad enough audio
        // bytes for the decoder to work later.
        let mut alloc_fields: Vec<(u8, u32)> = vec![(4, 1), (4, 2)];
        for sb in 2..table.sblimit() {
            alloc_fields.push((table.nbal(sb), 0));
        }
        // scfsi (2 bits per allocated subband; here 2 subbands -> 4 bits)
        // — pick selector 0b00 each.
        let mut combined_fields = alloc_fields.clone();
        combined_fields.push((2, 0b00));
        combined_fields.push((2, 0b00));
        // Then a scalefactor field per part per subband (6 bits × 3
        // parts × 2 subbands = 36 bits). All-zero scalefactor indices
        // produce a valid (if quiet) Layer II frame.
        for _ in 0..(3 * 2) {
            combined_fields.push((6, 0));
        }
        // Sample data: 12 syntax-granules × 2 subbands × 1 codeword.
        // Grouped class nlevels=3 uses 5 bits per codeword (Table 3-B.4);
        // nlevels=5 uses 7. All-zero codewords are valid (they decode
        // to non-zero samples but that doesn't affect the CRC).
        for _ in 0..12 {
            combined_fields.push((5, 0));
            combined_fields.push((7, 0));
        }
        // Round up to a whole number of bytes for the rest of the frame.
        let body = pack_msb_first(&combined_fields);

        let crc = compute_layer2_crc(&h, &header, &body).expect("compute_layer2_crc");

        // Stage the bitstream: header + CRC word (MSB-first) + body.
        let mut after = crc.to_be_bytes().to_vec();
        after.extend_from_slice(&body);

        match verify_layer2_crc(&h, &header, &after) {
            Some(CrcStatus::Ok(stored)) => assert_eq!(stored, crc),
            other => panic!("expected CrcStatus::Ok, got {other:?}"),
        }
        assert!(verify_layer2_crc(&h, &header, &after).unwrap().is_good());
    }

    #[test]
    fn layer2_crc_detects_corruption_in_allocation_field() {
        let header = protected_mono_l2_header();
        let h = FrameHeader::parse(&header).unwrap();
        let table = layer2_bit_allocation_table(&h);
        // Single allocated subband (sb0 alloc=1), zeros elsewhere.
        let mut fields: Vec<(u8, u32)> = vec![(4, 1)];
        for sb in 1..table.sblimit() {
            fields.push((table.nbal(sb), 0));
        }
        // One scfsi read for sb0 (2 bits).
        fields.push((2, 0b00));
        // Three scalefactor indices for sb0 (6 bits each).
        for _ in 0..3 {
            fields.push((6, 0));
        }
        // 12 syntax-granules of one 5-bit codeword.
        for _ in 0..12 {
            fields.push((5, 0));
        }
        let body = pack_msb_first(&fields);
        let crc = compute_layer2_crc(&h, &header, &body).unwrap();
        let mut after = crc.to_be_bytes().to_vec();
        after.extend_from_slice(&body);

        // Flip a bit inside the first byte of the allocation field
        // (after the 2-byte CRC), changing sb0's allocation from 1 to
        // some other still-valid value. The flipped bit is within the
        // first 88 protected bits, so the CRC must mismatch.
        after[2] ^= 0b0001_0000; // flip a high bit in the alloc field's nibble
        match verify_layer2_crc(&h, &header, &after) {
            Some(CrcStatus::Mismatch { stored, computed }) => {
                assert_eq!(stored, crc);
                assert_ne!(computed, crc);
            }
            other => panic!("expected Mismatch, got {other:?}"),
        }
    }

    #[test]
    fn layer2_crc_detects_corruption_in_header_bits_16_31() {
        // Two protected Layer II headers that differ only in header
        // bits 16..31 (different bitrate_index) produce different CRCs
        // over the same allocation/scfsi region.
        let header_a = protected_mono_l2_header();
        let h_a = FrameHeader::parse(&header_a).unwrap();
        // Build header_b: same as A but bitrate_index 0b0011 (56
        // kbit/s on the L2 MPEG-1 ladder).
        let word_b: u32 = (0xFFF << 20)
            | (1 << 19)
            | (0b10 << 17)
            | (0b0011 << 12)
            // sampling 0b00 (44.1 kHz) and other zero-valued fields omitted.
            | (0b11 << 6)
            | (1 << 2);
        let header_b = word_b.to_be_bytes();
        let h_b = FrameHeader::parse(&header_b).unwrap();
        // Same allocation/scfsi body.
        let table = layer2_bit_allocation_table(&h_a);
        let mut fields: Vec<(u8, u32)> = vec![(4, 1)];
        for sb in 1..table.sblimit() {
            fields.push((table.nbal(sb), 0));
        }
        fields.push((2, 0b00));
        let body = pack_msb_first(&fields);
        let crc_a = compute_layer2_crc(&h_a, &header_a, &body).unwrap();
        let crc_b = compute_layer2_crc(&h_b, &header_b, &body).unwrap();
        assert_ne!(
            crc_a, crc_b,
            "CRC must cover header bits 16..31 — different bitrate_index must produce different CRC"
        );
    }

    #[test]
    fn layer2_crc_detects_corruption_in_scfsi_field() {
        // Build a frame with one allocated subband. Flip a bit inside
        // the scfsi field (which sits AFTER the 88-bit allocation
        // field). The scfsi is part of the protected set per Table
        // 3-B.5, so the CRC must mismatch when the scfsi bit changes.
        let header = protected_mono_l2_header();
        let h = FrameHeader::parse(&header).unwrap();
        let table = layer2_bit_allocation_table(&h);
        let mut fields: Vec<(u8, u32)> = vec![(4, 1)];
        for sb in 1..table.sblimit() {
            fields.push((table.nbal(sb), 0));
        }
        // scfsi for sb0: selector 0b00 (then 6+6+6 scf bits and the
        // sample data follow, but those are unprotected).
        fields.push((2, 0b00));
        for _ in 0..3 {
            fields.push((6, 0));
        }
        for _ in 0..12 {
            fields.push((5, 0));
        }
        let body = pack_msb_first(&fields);
        let crc_ok = compute_layer2_crc(&h, &header, &body).unwrap();

        // Build a second body where the scfsi bits are flipped to 0b11.
        let mut fields2 = fields.clone();
        fields2[table.sblimit()] = (2, 0b11); // index 27 is the scfsi entry
        let body2 = pack_msb_first(&fields2);
        let crc_diff = compute_layer2_crc(&h, &header, &body2).unwrap();
        assert_ne!(
            crc_ok, crc_diff,
            "CRC must cover the scfsi field — different scfsi must produce different CRC"
        );
    }

    #[test]
    fn layer2_verify_crc_absent_when_protection_set() {
        // protection_bit == 1 -> no CRC; verify reports Absent without
        // consuming bytes.
        let word: u32 = (0xFFF << 20)
            | (1 << 19)
            | (0b10 << 17)
            | (1 << 16) // protection bit = 1 (no CRC)
            | (0b0100 << 12)
            // sampling 0b00 (44.1 kHz) omitted (clippy `identity_op`).
            | (0b11 << 6)
            | (1 << 2);
        let header = word.to_be_bytes();
        let h = FrameHeader::parse(&header).unwrap();
        assert!(!h.has_crc());
        assert_eq!(verify_layer2_crc(&h, &header, &[]), Some(CrcStatus::Absent));
    }

    #[test]
    fn layer2_verify_crc_none_when_truncated() {
        let header = protected_mono_l2_header();
        let h = FrameHeader::parse(&header).unwrap();
        // Only one byte after the header: not enough for the CRC word.
        assert_eq!(verify_layer2_crc(&h, &header, &[0xAB]), None);
        // CRC word present but allocation+scfsi field truncated.
        assert_eq!(verify_layer2_crc(&h, &header, &[0x00, 0x00, 0x00]), None);
    }

    /// A grouped class: nlevels = 3, 5 bits per codeword. Degroup
    /// must produce three valid digit values from one combined int.
    #[test]
    fn grouped_degroup_3_levels() {
        let class = QuantClass {
            nlevels: 3,
            c: 4.0 / 3.0,
            d: 0.5,
            grouping: true,
            samples_per_codeword: 3,
            bits_per_codeword: 5,
        };
        // c = 0*1 + 1*3 + 2*9 = 21 (within 0..27 ≤ 2^5 = 32).
        // Degroup: s[0] = 21%3 = 0; c=7; s[1]=7%3=1; c=2; s[2]=2%3=2.
        // Then each s[i] inverted (msb of nbits_per_sample = ceil log2 3
        // — see bits_per_sample(): the *per-sample* sign width). The
        // §2.4.3.3.4 inversion is the most-significant bit of the
        // per-sample code, treating each separated code as a
        // nbits-per-sample-wide unsigned int. For nlevels=3 that
        // per-sample width is 2 (ceil log2 3) — checked in tables_layer2.
        let codes = {
            let mut c = 21u32;
            let mut s = [0u32; 3];
            for code in s.iter_mut() {
                *code = c % 3;
                c /= 3;
            }
            s
        };
        assert_eq!(codes, [0, 1, 2]);
        // Self-check the requantize for nbits_per_sample = 2 (per
        // Table B.4 grouped n=3 means 2-bit per-sample after degroup).
        // 0 inverted -> 2 -> signed 2-4 = -2 -> frac -1 -> s'' =
        // (4/3)*(-1 + 0.5) = -2/3.
        // 1 inverted -> 3 -> signed 3-4 = -1 -> frac -0.5 -> s'' =
        // (4/3)*(0) = 0.
        // 2 inverted -> 0 -> 0/2 = 0 -> s'' = (4/3)*0.5 = 2/3.
        let s_dp = requantize_triplet(&class, codes);
        assert!((s_dp[0] - (-2.0 / 3.0)).abs() < 1e-12);
        assert!(s_dp[1].abs() < 1e-12);
        assert!((s_dp[2] - (2.0 / 3.0)).abs() < 1e-12);
    }
}
