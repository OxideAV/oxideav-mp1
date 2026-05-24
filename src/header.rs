//! MPEG-1 / MPEG-2 LSF Audio **Layer I** frame-header parsing.
//!
//! Everything in this module is derived solely from ISO/IEC 11172-3
//! (1993), the MPEG-1 audio standard (Layer I clauses), and from
//! ISO/IEC 13818-3 (1997) §2.4.2.3 which redefines the
//! `sampling_frequency` and Layer I `bitrate_index` ladders when the
//! `ID` bit is `0` (Lower Sampling Frequencies / LSF mode, adding
//! 16 / 22.05 / 24 kHz):
//!
//! * §2.4.1.3 — the 32-bit `header()` field layout (common to MPEG-1
//!   and MPEG-2 LSF Layer I).
//! * §2.4.1.4 — the optional `error_check()` (16-bit CRC) that follows
//!   the header when `protection_bit == 0`.
//! * §2.4.2.1 — frame definition: in Layer I a frame carries 384
//!   samples and consists of an integer number of *slots*, each slot
//!   being four bytes (unchanged by LSF — only Layer III shrinks to
//!   576 samples per LSF frame).
//! * §2.4.2.3 (11172-3) — the MPEG-1 field semantics, the `ID==1`
//!   sampling-frequency table (32 / 44.1 / 48 kHz) and Layer I
//!   bitrate ladder (32 / 64 / ... / 448 kbit/s).
//! * §2.4.2.3 (13818-3) — the LSF redefinitions used when `ID==0`:
//!   the `sampling_frequency` table maps to 16 / 22.05 / 24 kHz, the
//!   Layer I bitrate ladder is 32 / 48 / 56 / 64 / 80 / 96 / 112 /
//!   128 / 144 / 160 / 176 / 192 / 224 / 256 kbit/s.
//! * §2.4.3.1 — synchronization and the frame-length relationship
//!   (slot distance `N` between consecutive syncwords). The Layer I
//!   slot-count formula `N = floor(12·bitrate/Fs) + padding` is
//!   identical in both editions; only the operands change.
//!
//! This module implements **only** the common header plus Layer I
//! frame sync and frame-length computation. The bit-allocation,
//! scalefactor and subband-synthesis stages (§2.4.2.5 / §2.4.3.2) are
//! intentionally out of scope for this layer.

/// Number of subband samples carried by one Layer I frame (§2.4.2.1).
///
/// One Layer I frame holds information for 384 audio samples
/// (12 samples × 32 subbands).
pub const LAYER1_SAMPLES_PER_FRAME: u32 = 384;

/// Size of a Layer I *slot*, in bytes (§2.4.2.1, and the glossary
/// definition of "Slot": "In Layer I a slot equals four bytes").
pub const LAYER1_SLOT_BYTES: u32 = 4;

/// The 11172-3 §2.4.2.3 `bitrate_index` → bitrate ladder, **Layer I
/// column only** (`ID == 1`, MPEG-1), in kbit/s.
///
/// Index `0b0000` is the *free format* condition (signalled here as
/// [`Bitrate::Free`]); index `0b1111` is *forbidden*
/// ([`Bitrate::Forbidden`]). The intermediate indices map to the
/// fixed bitrates listed in the standard's table.
const LAYER1_BITRATE_KBPS_MPEG1: [u16; 14] = [
    // bitrate_index 0b0001 .. 0b1110 (the 14 fixed MPEG-1 Layer I rates).
    32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448,
];

/// The 13818-3 §2.4.2.3 `bitrate_index` → bitrate ladder, **Layer I
/// column only** (`ID == 0`, MPEG-2 LSF), in kbit/s.
///
/// Index `0b0000` is the *free format* condition, `0b1111` is
/// *forbidden*. The Lower-Sampling-Frequencies ladder differs from
/// the MPEG-1 ladder: notably the smaller increments at the low end
/// (32 / 48 / 56 / 64 …) and the top rate of 256 kbit/s.
const LAYER1_BITRATE_KBPS_LSF: [u16; 14] = [
    // bitrate_index 0b0001 .. 0b1110 (the 14 fixed LSF Layer I rates,
    // per ISO/IEC 13818-3 §2.4.2.3 table).
    32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256,
];

/// The 11172-3 §2.4.2.3 `sampling_frequency` table (MPEG-1, `ID == 1`)
/// in Hz.
///
/// Index `0b11` is *reserved* and is represented here by `None`.
const SAMPLING_FREQUENCY_HZ_MPEG1: [Option<u32>; 4] = [
    Some(44_100), // 0b00 -> 44.1 kHz
    Some(48_000), // 0b01 -> 48 kHz
    Some(32_000), // 0b10 -> 32 kHz
    None,         // 0b11 -> reserved
];

/// The 13818-3 §2.4.2.3 `sampling_frequency` table (MPEG-2 LSF,
/// `ID == 0`) in Hz.
///
/// Note the different mapping from the MPEG-1 table: `0b00` selects
/// 22.05 kHz, `0b01` selects 24 kHz, `0b10` selects 16 kHz. Index
/// `0b11` is *reserved*.
const SAMPLING_FREQUENCY_HZ_LSF: [Option<u32>; 4] = [
    Some(22_050), // 0b00 -> 22.05 kHz
    Some(24_000), // 0b01 -> 24 kHz
    Some(16_000), // 0b10 -> 16 kHz
    None,         // 0b11 -> reserved
];

/// Errors that can arise while parsing a Layer I frame header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderError {
    /// Fewer than the four header bytes were available.
    TooShort,
    /// The 12-bit syncword `0xFFF` was not present at the start.
    BadSync,
    /// The 2-bit `layer` field did not select Layer I (`0b11`).
    ///
    /// Carries the raw 2-bit value that was read.
    NotLayer1(u8),
    /// `bitrate_index` was `0b1111`, which the standard marks as
    /// *forbidden*.
    ForbiddenBitrate,
    /// `sampling_frequency` was `0b11`, which the standard marks as
    /// *reserved*.
    ReservedSamplingFrequency,
}

impl core::fmt::Display for HeaderError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            HeaderError::TooShort => write!(f, "fewer than 4 header bytes available"),
            HeaderError::BadSync => write!(f, "syncword 0xFFF not found at start of header"),
            HeaderError::NotLayer1(v) => {
                write!(f, "layer field 0b{v:02b} does not select Layer I")
            }
            HeaderError::ForbiddenBitrate => write!(f, "bitrate_index 0b1111 is forbidden"),
            HeaderError::ReservedSamplingFrequency => {
                write!(f, "sampling_frequency 0b11 is reserved")
            }
        }
    }
}

impl std::error::Error for HeaderError {}

/// The `ID` bit (§2.4.2.3).
///
/// In ISO/IEC 11172-3 alone, `ID == '0'` is reserved. ISO/IEC 13818-3
/// §2.4.2.3 redefines `ID == '0'` as the *Lower Sampling Frequencies*
/// (LSF) extension that adds the three sampling frequencies 16 /
/// 22.05 / 24 kHz and a separate Layer I bitrate ladder. This crate
/// supports both editions, so `ID == '0'` is now [`Id::Mpeg2Lsf`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Id {
    /// `ID == '1'`: MPEG-1 audio (11172-3) — the 32 / 44.1 / 48 kHz
    /// sampling-frequency table and the 32 / 64 / … / 448 kbit/s
    /// Layer I bitrate ladder apply.
    Mpeg,
    /// `ID == '0'`: MPEG-2 LSF audio (13818-3 §2.4.2.3) — the
    /// 16 / 22.05 / 24 kHz sampling-frequency table and the LSF
    /// Layer I bitrate ladder (32 / 48 / 56 / 64 / 80 / 96 / 112 /
    /// 128 / 144 / 160 / 176 / 192 / 224 / 256 kbit/s) apply.
    Mpeg2Lsf,
}

/// The `mode` field (§2.4.2.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// `'00'` — stereo.
    Stereo,
    /// `'01'` — joint_stereo (in Layer I, intensity_stereo).
    JointStereo,
    /// `'10'` — dual_channel.
    DualChannel,
    /// `'11'` — single_channel (mono).
    SingleChannel,
}

impl Mode {
    /// Number of audio channels implied by the mode.
    ///
    /// Every mode but `single_channel` carries two channels.
    pub fn channels(self) -> u8 {
        match self {
            Mode::SingleChannel => 1,
            _ => 2,
        }
    }
}

/// The `emphasis` field (§2.4.2.3): de-emphasis type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Emphasis {
    /// `'00'` — none.
    None,
    /// `'01'` — 50/15 microseconds.
    Ms5015,
    /// `'10'` — reserved.
    Reserved,
    /// `'11'` — CCITT J.17.
    CcittJ17,
}

/// The `bitrate_index` interpretation, **Layer I** (§2.4.2.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bitrate {
    /// `bitrate_index == 0b0000`: free format. A fixed bitrate not in
    /// the ladder is in use; its value is recovered from the distance
    /// between syncwords, not from the header.
    Free,
    /// A fixed bitrate from the Layer I ladder, in kbit/s.
    Fixed(u16),
    /// `bitrate_index == 0b1111`: forbidden.
    Forbidden,
}

/// The `mode_extension` field for joint_stereo (§2.4.2.3).
///
/// In Layer I/II these two bits select which subbands are coded in
/// intensity_stereo and therefore set the stereo `bound`. The variant
/// is only meaningful when [`Mode::JointStereo`] is selected; for the
/// other modes the raw two bits are still preserved so the header
/// round-trips exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeExtension(pub u8);

impl ModeExtension {
    /// The intensity_stereo `bound` (first subband coded in stereo)
    /// selected by this `mode_extension`, per the §2.4.2.3 table:
    ///
    /// | bits | bound |
    /// |------|-------|
    /// | `00` | 4     |
    /// | `01` | 8     |
    /// | `10` | 12    |
    /// | `11` | 16    |
    ///
    /// Only meaningful in joint_stereo mode.
    pub fn bound(self) -> u8 {
        match self.0 & 0b11 {
            0b00 => 4,
            0b01 => 8,
            0b10 => 12,
            _ => 16,
        }
    }
}

/// A fully decoded MPEG-1 Audio Layer I frame header (§2.4.1.3).
///
/// All thirteen header fields are preserved verbatim so a parsed
/// header carries enough state to compute the frame length and (in a
/// later round) drive the audio-data decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameHeader {
    /// `ID` — algorithm identifier.
    pub id: Id,
    /// `protection_bit`: `true` when **no** redundancy (CRC) was
    /// added. When `false`, a 16-bit CRC follows the header
    /// (§2.4.1.4).
    pub protection: bool,
    /// `bitrate_index` interpreted against the Layer I ladder.
    pub bitrate: Bitrate,
    /// `sampling_frequency`, in Hz.
    pub sampling_frequency: u32,
    /// `padding_bit`: when `true` the frame carries one extra slot.
    pub padding: bool,
    /// `private_bit`: reserved for private use, never assigned by ISO.
    pub private: bool,
    /// `mode`.
    pub mode: Mode,
    /// `mode_extension` (raw two bits; meaningful in joint_stereo).
    pub mode_extension: ModeExtension,
    /// `copyright`: `true` if copyright protected.
    pub copyright: bool,
    /// `original/copy`: `true` if this is an original (not a copy).
    pub original: bool,
    /// `emphasis`.
    pub emphasis: Emphasis,
}

impl FrameHeader {
    /// Parse the four header bytes that begin a Layer I frame
    /// (§2.4.1.3). Multi-byte fields are most-significant-byte first
    /// (§2.3, "The byte order of multi-byte words is most significant
    /// byte first").
    ///
    /// Returns an error if the slice is short, the syncword is absent,
    /// the layer is not Layer I, or the bitrate/sampling-frequency
    /// fields hold the forbidden / reserved values.
    pub fn parse(bytes: &[u8]) -> Result<FrameHeader, HeaderError> {
        if bytes.len() < 4 {
            return Err(HeaderError::TooShort);
        }
        // Pack the 32 header bits MSB-first.
        let word = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);

        // syncword: top 12 bits must be all ones (§2.4.2.3).
        if (word >> 20) != 0xFFF {
            return Err(HeaderError::BadSync);
        }

        // Field offsets within the 32-bit big-endian word, counting
        // from the MSB exactly as the §2.4.1.3 syntax lists them.
        let id_bit = (word >> 19) & 0x1;
        let layer = ((word >> 17) & 0x3) as u8;
        let protection_bit = (word >> 16) & 0x1;
        let bitrate_index = ((word >> 12) & 0xF) as u8;
        let sampling_index = ((word >> 10) & 0x3) as usize;
        let padding_bit = (word >> 9) & 0x1;
        let private_bit = (word >> 8) & 0x1;
        let mode_bits = ((word >> 6) & 0x3) as u8;
        let mode_ext_bits = ((word >> 4) & 0x3) as u8;
        let copyright_bit = (word >> 3) & 0x1;
        let original_bit = (word >> 2) & 0x1;
        let emphasis_bits = (word & 0x3) as u8;

        // layer: '11' selects Layer I (§2.4.2.3).
        if layer != 0b11 {
            return Err(HeaderError::NotLayer1(layer));
        }

        // ID selects which Layer I bitrate ladder and which
        // sampling_frequency table apply: 11172-3 §2.4.2.3 for ID==1,
        // 13818-3 §2.4.2.3 (LSF) for ID==0.
        let id = if id_bit == 1 { Id::Mpeg } else { Id::Mpeg2Lsf };
        let bitrate_table = match id {
            Id::Mpeg => &LAYER1_BITRATE_KBPS_MPEG1,
            Id::Mpeg2Lsf => &LAYER1_BITRATE_KBPS_LSF,
        };
        let sampling_table = match id {
            Id::Mpeg => &SAMPLING_FREQUENCY_HZ_MPEG1,
            Id::Mpeg2Lsf => &SAMPLING_FREQUENCY_HZ_LSF,
        };

        // bitrate_index against the selected Layer I ladder.
        let bitrate = match bitrate_index {
            0b0000 => Bitrate::Free,
            0b1111 => return Err(HeaderError::ForbiddenBitrate),
            n => Bitrate::Fixed(bitrate_table[(n - 1) as usize]),
        };

        // sampling_frequency table for the selected ID.
        let sampling_frequency = match sampling_table[sampling_index] {
            Some(hz) => hz,
            None => return Err(HeaderError::ReservedSamplingFrequency),
        };

        // protection_bit == '1' means NO redundancy added (§2.4.2.3).
        let protection = protection_bit == 1;

        let mode = match mode_bits {
            0b00 => Mode::Stereo,
            0b01 => Mode::JointStereo,
            0b10 => Mode::DualChannel,
            _ => Mode::SingleChannel,
        };

        let emphasis = match emphasis_bits {
            0b00 => Emphasis::None,
            0b01 => Emphasis::Ms5015,
            0b10 => Emphasis::Reserved,
            _ => Emphasis::CcittJ17,
        };

        Ok(FrameHeader {
            id,
            protection,
            bitrate,
            sampling_frequency,
            padding: padding_bit == 1,
            private: private_bit == 1,
            mode,
            mode_extension: ModeExtension(mode_ext_bits),
            copyright: copyright_bit == 1,
            original: original_bit == 1,
            emphasis,
        })
    }

    /// Number of channels signalled by `mode`.
    pub fn channels(self) -> u8 {
        self.mode.channels()
    }

    /// Whether a 16-bit CRC `error_check()` word follows this header
    /// (§2.4.1.4): present exactly when `protection_bit == 0`.
    pub fn has_crc(self) -> bool {
        !self.protection
    }

    /// Whether this header is an MPEG-2 LSF (Lower Sampling
    /// Frequencies) header (13818-3 §2.4.2.3, `ID == 0`).
    pub fn is_lsf(self) -> bool {
        matches!(self.id, Id::Mpeg2Lsf)
    }

    /// Number of slots in this frame (§2.4.2.3 / §2.4.3.1).
    ///
    /// For Layer I the slot distance `N` between consecutive syncwords
    /// is `floor(12 * bitrate / sampling_frequency)`, plus one extra
    /// slot when `padding_bit == 1`. This is the integer form of the
    /// §2.4.2.3 padding method, whose Layer I remainder is
    /// `dif = (12 * bitrate) % sampling_frequency`. `bitrate` here is
    /// in bit/s (the ladder stores kbit/s, so we scale by 1000).
    ///
    /// Returns `None` for the *free format* condition, where the
    /// header alone does not determine the slot count (§2.4.3.1: it
    /// must be recovered from the distance between syncwords).
    pub fn slot_count(self) -> Option<u32> {
        let kbps = match self.bitrate {
            Bitrate::Fixed(k) => k as u32,
            Bitrate::Free | Bitrate::Forbidden => return None,
        };
        let bitrate_bps = kbps * 1000;
        let base = (12 * bitrate_bps) / self.sampling_frequency;
        Some(base + u32::from(self.padding))
    }

    /// Total length of this frame in bytes, including the four header
    /// bytes (and, when present, the CRC and audio data — all of which
    /// are accounted for by the slot count, since the slot count spans
    /// the whole inter-syncword distance per §2.4.2.1 / §2.4.3.1).
    ///
    /// Each Layer I slot is four bytes (§2.4.2.1). Returns `None` for
    /// the free-format condition (see [`slot_count`](Self::slot_count)).
    pub fn frame_length_bytes(self) -> Option<u32> {
        self.slot_count().map(|slots| slots * LAYER1_SLOT_BYTES)
    }
}

/// Scan `buf` for the start of a Layer I frame, returning the offset
/// of the first position whose four bytes parse as a valid Layer I
/// header (§2.4.3.1, synchronization).
///
/// The search uses the full 32-bit header validity (syncword, the
/// Layer I selector, and the non-forbidden / non-reserved fields)
/// rather than the 12-bit syncword alone, which §2.4.3.1 notes is the
/// more reliable form when the layer and protection status are known.
pub fn find_sync(buf: &[u8]) -> Option<usize> {
    if buf.len() < 4 {
        return None;
    }
    (0..=buf.len() - 4).find(|&i| FrameHeader::parse(&buf[i..i + 4]).is_ok())
}

/// The §2.4.3.1 CRC-16 generator polynomial, expressed as the 16-bit
/// feedback mask used by the bit-serial shift register.
///
/// §2.4.3.1 specifies `G(X) = X^16 + X^15 + X^2 + 1` (page 36 of the
/// staged ISO/IEC 11172-3 PDF, recovered from the typeset equation
/// image). In a 16-bit register the `X^16` term is the bit shifted out
/// at the top, so the feedback taps below it are `X^15`, `X^2`, `X^0` —
/// i.e. the mask `0b1000_0000_0000_0101 = 0x8005`.
const CRC16_POLY: u16 = 0x8005;

/// The §2.4.3.1 CRC-16 shift-register initial state.
///
/// §2.4.3.1: "The initial state of the shift register is
/// '1111 1111 1111 1111'." — numerically `0xFFFF`.
const CRC16_INIT: u16 = 0xFFFF;

/// Feed `count` bits of `byte` (the most-significant `count` bits,
/// MSB-first) of each byte of `bytes` through the §2.4.3.1 CRC-16
/// shift register, returning the updated register value.
///
/// The full-byte case (`bit_count` a multiple of 8) is the common one;
/// the partial trailing byte handles bit-allocation fields whose bit
/// count is not a whole number of bytes (the Layer I allocation field
/// is always a whole number of nibbles, hence whole bytes when paired,
/// but the helper is bit-granular to stay faithful to §2.4.3.1's
/// bit-serial description).
///
/// `reg` is the running register state; the first call passes
/// [`CRC16_INIT`].
fn crc16_update_bits(mut reg: u16, bytes: &[u8], bit_count: usize) -> u16 {
    debug_assert!(bit_count <= bytes.len() * 8, "bit_count past slice end");
    for i in 0..bit_count {
        let bit = (bytes[i >> 3] >> (7 - (i & 7))) & 1;
        // Shift-register step for G(X) = X^16 + X^15 + X^2 + 1:
        // the bit shifted out of the top (X^16) XORs with the input
        // bit to decide whether the feedback mask is applied
        // (§2.4.3.1, figure A.9).
        let feedback = ((reg >> 15) as u8 & 1) ^ bit;
        reg <<= 1;
        if feedback == 1 {
            reg ^= CRC16_POLY;
        }
    }
    reg
}

/// The number of bits in the Layer I bit-allocation field for a frame
/// with the given header (Table 3-B.5 "bit allocation").
///
/// §2.4.1.5 serialises four allocation bits per subband per channel,
/// except that in joint_stereo the subbands at or above the
/// `bound` (`(mode_extension + 1) × 4`, §2.4.2.3) carry a single
/// shared 4-bit allocation. So the allocation count is:
///
/// * non-joint modes: `channels × 32` allocations;
/// * joint_stereo: `channels × bound + (32 − bound)` allocations.
///
/// Each allocation is four bits, so the bit total is `4 ×`
/// allocations.
fn allocation_field_bits(header: &FrameHeader) -> usize {
    let nch = header.channels() as usize;
    let allocations = match header.mode {
        Mode::JointStereo => {
            let bound = (header.mode_extension.bound() as usize).min(32);
            nch * bound + (32 - bound)
        }
        _ => nch * 32,
    };
    allocations * 4
}

/// Result of a 16-bit `error_check()` CRC verification (§2.4.1.4 /
/// §2.4.3.1).
///
/// §2.4.3.1 fully specifies the check: generator polynomial
/// `G(X) = X^16 + X^15 + X^2 + 1`, initial shift-register state
/// `0xFFFF`, and the protected-field set from Annex B Table 3-B.5
/// (Layer I: header bits 16…31 plus the bit-allocation field). The
/// computed value is compared with the stored CRC word; on a mismatch
/// §2.4.3.1 recommends concealment (muting the frame or repeating the
/// previous one).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcStatus {
    /// `protection_bit == 1`: no CRC word is present, so nothing to
    /// check (§2.4.2.3).
    Absent,
    /// `protection_bit == 0`: a 16-bit CRC word follows the header,
    /// the stored value (`stored`) matched the value computed over the
    /// protected fields. Carries the (equal) CRC word.
    Ok(u16),
    /// `protection_bit == 0`: the stored CRC word did not match the
    /// value computed over the protected fields. Carries the
    /// `stored`/`computed` pair so callers can log a diagnostic and
    /// apply §2.4.3.1 concealment.
    Mismatch {
        /// The CRC word read from the bitstream.
        stored: u16,
        /// The CRC-16 value computed over the protected fields.
        computed: u16,
    },
}

impl CrcStatus {
    /// Whether the frame passed (or had no) CRC protection.
    ///
    /// [`Absent`](CrcStatus::Absent) and [`Ok`](CrcStatus::Ok) are
    /// "good"; only a [`Mismatch`](CrcStatus::Mismatch) is not.
    pub fn is_good(self) -> bool {
        !matches!(self, CrcStatus::Mismatch { .. })
    }
}

impl FrameHeader {
    /// Read **and verify** the optional `error_check()` field that
    /// follows the header (§2.4.1.4 / §2.4.3.1).
    ///
    /// `header_bytes` is the four-byte header this `FrameHeader` was
    /// parsed from (bits 16…31 of it are part of the protected field —
    /// Table 3-B.5). `after_header` is the bitstream slice that begins
    /// immediately after those four header bytes: in a protected frame
    /// its first two bytes are the stored CRC word (MSB-first, §2.3)
    /// and the remaining bytes are the audio data, whose leading
    /// bit-allocation field is the rest of the protected set.
    ///
    /// When [`has_crc`](Self::has_crc) is false this returns
    /// [`CrcStatus::Absent`] without consuming any bytes. When a CRC is
    /// expected the §2.4.3.1 CRC-16 is computed over header bits 16…31
    /// followed by the bit-allocation field and compared with the
    /// stored word, returning [`CrcStatus::Ok`] or
    /// [`CrcStatus::Mismatch`]. Returns `None` if a CRC is expected but
    /// the slice is too short to hold the CRC word plus the allocation
    /// field.
    pub fn verify_crc(self, header_bytes: &[u8], after_header: &[u8]) -> Option<CrcStatus> {
        if !self.has_crc() {
            return Some(CrcStatus::Absent);
        }
        if header_bytes.len() < 4 || after_header.len() < 2 {
            return None;
        }
        let stored = u16::from_be_bytes([after_header[0], after_header[1]]);

        // The audio data (and thus the bit-allocation field) begins
        // after the two CRC bytes.
        let audio = &after_header[2..];
        let alloc_bits = allocation_field_bits(&self);
        if audio.len() * 8 < alloc_bits {
            return None;
        }

        // Protected fields (Table 3-B.5, Layer I):
        //   1. bits 16…31 of the header = header_bytes[2..4];
        //   2. the bit-allocation field at the start of the audio data.
        let mut reg = crc16_update_bits(CRC16_INIT, &header_bytes[2..4], 16);
        reg = crc16_update_bits(reg, audio, alloc_bits);

        Some(if reg == stored {
            CrcStatus::Ok(stored)
        } else {
            CrcStatus::Mismatch {
                stored,
                computed: reg,
            }
        })
    }

    /// The §2.4.3.1 CRC-16 word for a frame this header describes,
    /// computed over the protected fields (Table 3-B.5: header bits
    /// 16…31 + the bit-allocation field).
    ///
    /// `header_bytes` is the four-byte header and `allocation` is the
    /// audio-data slice whose leading bit-allocation field (§2.4.1.5:
    /// four bits per allocation, shared above the joint-stereo `bound`)
    /// is the protected part; any trailing bytes are ignored. This is
    /// the value an encoder writes into the `error_check()` field;
    /// `verify_crc` recomputes it and compares.
    ///
    /// Returns `None` if the slices are too short for the protected
    /// field.
    pub fn compute_crc(self, header_bytes: &[u8], allocation: &[u8]) -> Option<u16> {
        if header_bytes.len() < 4 {
            return None;
        }
        let alloc_bits = allocation_field_bits(&self);
        if allocation.len() * 8 < alloc_bits {
            return None;
        }
        let mut reg = crc16_update_bits(CRC16_INIT, &header_bytes[2..4], 16);
        reg = crc16_update_bits(reg, allocation, alloc_bits);
        Some(reg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 4-byte Layer I header from explicit field values,
    /// packing them MSB-first exactly as §2.4.1.3 lists them. This is
    /// our own constructor (no external bytes), used to drive the
    /// parser tests.
    #[allow(clippy::too_many_arguments)]
    fn build_header(
        id: u32,
        layer: u32,
        protection: u32,
        bitrate_index: u32,
        sampling: u32,
        padding: u32,
        private: u32,
        mode: u32,
        mode_ext: u32,
        copyright: u32,
        original: u32,
        emphasis: u32,
    ) -> [u8; 4] {
        let word: u32 = (0xFFF << 20)
            | (id << 19)
            | (layer << 17)
            | (protection << 16)
            | (bitrate_index << 12)
            | (sampling << 10)
            | (padding << 9)
            | (private << 8)
            | (mode << 6)
            | (mode_ext << 4)
            | (copyright << 3)
            | (original << 2)
            | emphasis;
        word.to_be_bytes()
    }

    fn canonical() -> [u8; 4] {
        // MPEG-1 Layer I, no CRC (protection=1), 256 kbit/s
        // (index 0b1000), 48 kHz (0b01), no padding, stereo, no
        // joint-stereo extension, not copyright, original, no
        // emphasis.
        build_header(1, 0b11, 1, 0b1000, 0b01, 0, 0, 0b00, 0b00, 0, 1, 0b00)
    }

    #[test]
    fn parses_all_canonical_fields() {
        let h = FrameHeader::parse(&canonical()).expect("valid header");
        assert_eq!(h.id, Id::Mpeg);
        assert!(h.protection, "protection_bit==1 means no CRC");
        assert_eq!(h.bitrate, Bitrate::Fixed(256));
        assert_eq!(h.sampling_frequency, 48_000);
        assert!(!h.padding);
        assert!(!h.private);
        assert_eq!(h.mode, Mode::Stereo);
        assert_eq!(h.mode_extension, ModeExtension(0));
        assert!(!h.copyright);
        assert!(h.original);
        assert_eq!(h.emphasis, Emphasis::None);
        assert_eq!(h.channels(), 2);
        assert!(!h.has_crc());
    }

    #[test]
    fn rejects_bad_sync() {
        let mut bytes = canonical();
        bytes[0] = 0xFE; // break the syncword
        assert_eq!(FrameHeader::parse(&bytes), Err(HeaderError::BadSync));
    }

    #[test]
    fn rejects_too_short() {
        assert_eq!(
            FrameHeader::parse(&[0xFF, 0xFF, 0xFF]),
            Err(HeaderError::TooShort)
        );
    }

    #[test]
    fn rejects_non_layer1() {
        // layer '10' = Layer II, '01' = Layer III, '00' = reserved.
        for (bits, _name) in [(0b10u32, "II"), (0b01, "III"), (0b00, "reserved")] {
            let bytes = build_header(1, bits, 1, 0b1000, 0b01, 0, 0, 0, 0, 0, 1, 0);
            assert_eq!(
                FrameHeader::parse(&bytes),
                Err(HeaderError::NotLayer1(bits as u8))
            );
        }
    }

    #[test]
    fn rejects_forbidden_bitrate() {
        let bytes = build_header(1, 0b11, 1, 0b1111, 0b01, 0, 0, 0, 0, 0, 1, 0);
        assert_eq!(
            FrameHeader::parse(&bytes),
            Err(HeaderError::ForbiddenBitrate)
        );
    }

    #[test]
    fn rejects_reserved_sampling_frequency() {
        let bytes = build_header(1, 0b11, 1, 0b1000, 0b11, 0, 0, 0, 0, 0, 1, 0);
        assert_eq!(
            FrameHeader::parse(&bytes),
            Err(HeaderError::ReservedSamplingFrequency)
        );
    }

    #[test]
    fn free_format_bitrate() {
        let bytes = build_header(1, 0b11, 1, 0b0000, 0b01, 0, 0, 0, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert_eq!(h.bitrate, Bitrate::Free);
        // Free format: frame length not determinable from header alone.
        assert_eq!(h.slot_count(), None);
        assert_eq!(h.frame_length_bytes(), None);
    }

    #[test]
    fn whole_bitrate_ladder_layer1() {
        // §2.4.2.3 Layer I column, indices 0b0001..0b1110.
        let expected = [
            32u16, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448,
        ];
        for (i, &kbps) in expected.iter().enumerate() {
            let idx = (i + 1) as u32; // 0b0001..0b1110
            let bytes = build_header(1, 0b11, 1, idx, 0b01, 0, 0, 0, 0, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.bitrate, Bitrate::Fixed(kbps), "index 0b{idx:04b}");
        }
    }

    #[test]
    fn sampling_frequency_table() {
        for (bits, hz) in [(0b00u32, 44_100u32), (0b01, 48_000), (0b10, 32_000)] {
            let bytes = build_header(1, 0b11, 1, 0b1000, bits, 0, 0, 0, 0, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.sampling_frequency, hz, "sampling 0b{bits:02b}");
        }
    }

    #[test]
    fn mode_table_and_channels() {
        let cases = [
            (0b00u32, Mode::Stereo, 2u8),
            (0b01, Mode::JointStereo, 2),
            (0b10, Mode::DualChannel, 2),
            (0b11, Mode::SingleChannel, 1),
        ];
        for (bits, mode, ch) in cases {
            let bytes = build_header(1, 0b11, 1, 0b1000, 0b01, 0, 0, bits, 0, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.mode, mode, "mode 0b{bits:02b}");
            assert_eq!(h.channels(), ch);
        }
    }

    #[test]
    fn mode_extension_bound() {
        // §2.4.2.3: 00->4, 01->8, 10->12, 11->16.
        for (bits, bound) in [(0b00u32, 4u8), (0b01, 8), (0b10, 12), (0b11, 16)] {
            let bytes = build_header(1, 0b11, 1, 0b1000, 0b01, 0, 0, 0b01, bits, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.mode_extension, ModeExtension(bits as u8));
            assert_eq!(h.mode_extension.bound(), bound, "mode_ext 0b{bits:02b}");
        }
    }

    #[test]
    fn emphasis_table() {
        let cases = [
            (0b00u32, Emphasis::None),
            (0b01, Emphasis::Ms5015),
            (0b10, Emphasis::Reserved),
            (0b11, Emphasis::CcittJ17),
        ];
        for (bits, emph) in cases {
            let bytes = build_header(1, 0b11, 1, 0b1000, 0b01, 0, 0, 0, 0, 0, 1, bits);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.emphasis, emph, "emphasis 0b{bits:02b}");
        }
    }

    #[test]
    fn single_bit_flags_round_trip() {
        // Drive a stress combination: ID==0 (MPEG-2 LSF), all single-bit
        // flags set, padding on, original/copy cleared. With ID==0 the
        // bitrate_index 0b1000 maps to 128 kbit/s on the LSF ladder and
        // sampling 0b01 maps to 24 kHz (13818-3 §2.4.2.3).
        let bytes = build_header(0, 0b11, 1, 0b1000, 0b01, 1, 1, 0b00, 0, 1, 0, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert_eq!(h.id, Id::Mpeg2Lsf);
        assert!(h.is_lsf());
        assert_eq!(h.bitrate, Bitrate::Fixed(128));
        assert_eq!(h.sampling_frequency, 24_000);
        assert!(h.padding);
        assert!(h.private);
        assert!(h.copyright);
        assert!(!h.original, "original/copy == 0 means a copy");
    }

    // ---- ISO/IEC 13818-3 §2.4.2.3 LSF (ID==0) -----------------

    #[test]
    fn lsf_sampling_frequency_table() {
        // 13818-3 §2.4.2.3: with ID==0 the sampling_frequency field
        // maps 0b00 -> 22.05 kHz, 0b01 -> 24 kHz, 0b10 -> 16 kHz.
        let cases = [(0b00u32, 22_050u32), (0b01, 24_000), (0b10, 16_000)];
        for (bits, hz) in cases {
            // bitrate_index 0b0001 = 32 kbit/s on the LSF ladder.
            let bytes = build_header(0, 0b11, 1, 0b0001, bits, 0, 0, 0, 0, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.id, Id::Mpeg2Lsf);
            assert_eq!(h.sampling_frequency, hz, "lsf sampling 0b{bits:02b}");
        }
    }

    #[test]
    fn lsf_reserved_sampling_frequency() {
        // ID==0 with sampling_frequency 0b11 must still be rejected.
        let bytes = build_header(0, 0b11, 1, 0b0001, 0b11, 0, 0, 0, 0, 0, 1, 0);
        assert_eq!(
            FrameHeader::parse(&bytes),
            Err(HeaderError::ReservedSamplingFrequency)
        );
    }

    #[test]
    fn lsf_whole_bitrate_ladder_layer1() {
        // 13818-3 §2.4.2.3 Layer I LSF column, indices 0b0001..0b1110.
        // Read directly from the staged ISO/IEC 13818-3 PDF table at
        // §2.4.2.3 ("bitrate specified (kbit/s) for Fs = 16, 22,05, 24
        // kHz", Layer I column).
        let expected = [
            32u16, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256,
        ];
        for (i, &kbps) in expected.iter().enumerate() {
            let idx = (i + 1) as u32; // 0b0001..0b1110
                                      // Use 24 kHz (sampling 0b01 with ID==0) so every ladder
                                      // entry yields a non-zero slot count.
            let bytes = build_header(0, 0b11, 1, idx, 0b01, 0, 0, 0, 0, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.bitrate, Bitrate::Fixed(kbps), "LSF index 0b{idx:04b}");
        }
    }

    #[test]
    fn lsf_does_not_overlap_mpeg1_ladder_at_index_2() {
        // Index 0b0010: MPEG-1 -> 64 kbit/s, LSF -> 48 kbit/s. Same
        // header bits, different ID, different decoded bitrate.
        let m1 = FrameHeader::parse(&build_header(1, 0b11, 1, 0b0010, 0b01, 0, 0, 0, 0, 0, 1, 0))
            .unwrap();
        assert_eq!(m1.bitrate, Bitrate::Fixed(64));
        let lsf = FrameHeader::parse(&build_header(0, 0b11, 1, 0b0010, 0b01, 0, 0, 0, 0, 0, 1, 0))
            .unwrap();
        assert_eq!(lsf.bitrate, Bitrate::Fixed(48));
    }

    #[test]
    fn lsf_frame_length_24k_64kbps() {
        // 24 kHz, 64 kbit/s (LSF index 0b0100), no padding.
        // slots = floor(12 * 64000 / 24000) = 32 -> 128 bytes.
        let bytes = build_header(0, 0b11, 1, 0b0100, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert!(h.is_lsf());
        assert_eq!(h.slot_count(), Some(32));
        assert_eq!(h.frame_length_bytes(), Some(128));
    }

    #[test]
    fn lsf_frame_length_22k05_needs_padding() {
        // 22.05 kHz, 64 kbit/s (LSF index 0b0100), padding bit set.
        // base = floor(12 * 64000 / 22050) = floor(34.83..) = 34;
        // +1 padding slot = 35 slots -> 140 bytes.
        let bytes = build_header(0, 0b11, 1, 0b0100, 0b00, 1, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert_eq!(h.slot_count(), Some(35));
        assert_eq!(h.frame_length_bytes(), Some(140));
    }

    #[test]
    fn lsf_frame_length_16k_lowest_bitrate() {
        // 16 kHz, 32 kbit/s (LSF index 0b0001), no padding.
        // slots = floor(12 * 32000 / 16000) = 24 -> 96 bytes.
        let bytes = build_header(0, 0b11, 1, 0b0001, 0b10, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert_eq!(h.sampling_frequency, 16_000);
        assert_eq!(h.slot_count(), Some(24));
        assert_eq!(h.frame_length_bytes(), Some(96));
    }

    #[test]
    fn frame_length_48k_256kbps_no_padding() {
        // slots = floor(12 * 256000 / 48000) = floor(64) = 64.
        // bytes = 64 * 4 = 256.
        let h = FrameHeader::parse(&canonical()).unwrap();
        assert_eq!(h.slot_count(), Some(64));
        assert_eq!(h.frame_length_bytes(), Some(256));
    }

    #[test]
    fn frame_length_44k1_needs_padding() {
        // 44.1 kHz, 256 kbit/s, padding bit set.
        // base = floor(12 * 256000 / 44100) = floor(69.66..) = 69.
        // +1 padding slot = 70 slots -> 280 bytes.
        let bytes = build_header(1, 0b11, 1, 0b1000, 0b00, 1, 0, 0b00, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert_eq!(h.slot_count(), Some(70));
        assert_eq!(h.frame_length_bytes(), Some(280));
    }

    #[test]
    fn frame_length_44k1_no_padding() {
        let bytes = build_header(1, 0b11, 1, 0b1000, 0b00, 0, 0, 0b00, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        // base = floor(12 * 256000 / 44100) = 69 slots -> 276 bytes.
        assert_eq!(h.slot_count(), Some(69));
        assert_eq!(h.frame_length_bytes(), Some(276));
    }

    #[test]
    fn frame_length_32k_lowest_bitrate() {
        // 32 kHz, 32 kbit/s (index 0b0001), no padding.
        // slots = floor(12 * 32000 / 32000) = 12 -> 48 bytes.
        let bytes = build_header(1, 0b11, 1, 0b0001, 0b10, 0, 0, 0b00, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert_eq!(h.slot_count(), Some(12));
        assert_eq!(h.frame_length_bytes(), Some(48));
    }

    #[test]
    fn find_sync_locates_header_after_garbage() {
        let header = canonical();
        let mut buf = vec![0x00, 0x12, 0x34, 0xFF, 0xE0]; // leading junk
        buf.extend_from_slice(&header);
        let off = find_sync(&buf).expect("should find the header");
        assert_eq!(off, 5);
        // And parsing at that offset succeeds.
        assert!(FrameHeader::parse(&buf[off..]).is_ok());
    }

    #[test]
    fn find_sync_none_when_absent() {
        let buf = [0x00u8, 0x11, 0x22, 0x33, 0x44, 0x55];
        assert_eq!(find_sync(&buf), None);
    }

    #[test]
    fn crc_absent_when_protection_set() {
        let h = FrameHeader::parse(&canonical()).unwrap();
        assert!(!h.has_crc());
        assert_eq!(h.verify_crc(&canonical(), &[]), Some(CrcStatus::Absent));
        assert!(CrcStatus::Absent.is_good());
    }

    // ---- §2.4.3.1 CRC-16: polynomial + protected field --------

    #[test]
    fn crc16_poly_and_init_match_spec() {
        // G(X) = X^16 + X^15 + X^2 + 1 -> feedback mask 0x8005 with the
        // X^16 term as the bit shifted out of the 16-bit register.
        assert_eq!(CRC16_POLY, 0x8005);
        // Initial state '1111 1111 1111 1111' = 0xFFFF (§2.4.3.1).
        assert_eq!(CRC16_INIT, 0xFFFF);
    }

    #[test]
    fn crc16_known_short_vectors() {
        // Self-contained checks of the bit-serial register from the
        // §2.4.3.1 description (init 0xFFFF, poly X^16+X^15+X^2+1).
        // Empty input: register unchanged.
        assert_eq!(crc16_update_bits(CRC16_INIT, &[], 0), 0xFFFF);
        // A single 0 bit: top bit of 0xFFFF is 1, XOR input 0 = 1, so
        // feedback fires: (0xFFFF << 1) ^ 0x8005 = 0xFFFE ^ 0x8005 =
        // 0x7FFB.
        assert_eq!(crc16_update_bits(CRC16_INIT, &[0x00], 1), 0x7FFB);
        // A single 1 bit: feedback = 1 ^ 1 = 0, no mask: 0xFFFF<<1 =
        // 0xFFFE.
        assert_eq!(crc16_update_bits(CRC16_INIT, &[0x80], 1), 0xFFFE);
    }

    #[test]
    fn allocation_field_bits_by_mode() {
        // mono: 1 channel * 32 * 4 = 128 bits.
        let mono = FrameHeader::parse(&build_header(
            1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b11, 0, 0, 1, 0,
        ))
        .unwrap();
        assert_eq!(allocation_field_bits(&mono), 128);
        // stereo: 2 channels * 32 * 4 = 256 bits.
        let stereo = FrameHeader::parse(&build_header(
            1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b00, 0, 0, 1, 0,
        ))
        .unwrap();
        assert_eq!(allocation_field_bits(&stereo), 256);
        // joint_stereo, mode_ext 0b00 -> bound 4: 2*4 + (32-4) = 36
        // allocations * 4 = 144 bits.
        let js = FrameHeader::parse(&build_header(
            1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b01, 0b00, 0, 1, 0,
        ))
        .unwrap();
        assert_eq!(allocation_field_bits(&js), 144);
        // joint_stereo, mode_ext 0b11 -> bound 16: 2*16 + (32-16) = 48
        // allocations * 4 = 192 bits.
        let js2 = FrameHeader::parse(&build_header(
            1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b01, 0b11, 0, 1, 0,
        ))
        .unwrap();
        assert_eq!(allocation_field_bits(&js2), 192);
    }

    #[test]
    fn crc_roundtrip_compute_then_verify_ok() {
        // protection_bit == 0 -> CRC present. Build a mono frame with a
        // chosen allocation field, compute the CRC with `compute_crc`,
        // and confirm `verify_crc` accepts it (Ok) when the same word is
        // stored in the bitstream.
        let header = build_header(1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        assert!(h.has_crc());
        // Mono allocation field = 128 bits = 16 bytes. Use an arbitrary
        // but fixed pattern; trailing audio bytes are ignored by the CRC.
        let mut audio = vec![0u8; 64];
        for (i, b) in audio.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(37).wrapping_add(11);
        }
        let crc = h.compute_crc(&header, &audio).expect("compute_crc");
        // Place the computed CRC word (MSB-first) ahead of the audio.
        let mut after = crc.to_be_bytes().to_vec();
        after.extend_from_slice(&audio);
        assert_eq!(h.verify_crc(&header, &after), Some(CrcStatus::Ok(crc)));
        assert!(h.verify_crc(&header, &after).unwrap().is_good());
    }

    #[test]
    fn crc_detects_corruption_in_protected_field() {
        let header = build_header(1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        let audio = vec![0xA5u8; 64];
        let crc = h.compute_crc(&header, &audio).unwrap();
        let mut after = crc.to_be_bytes().to_vec();
        after.extend_from_slice(&audio);
        // Flip a bit inside the bit-allocation field (first audio byte,
        // which is within the first 128 protected bits): the CRC must
        // no longer match.
        after[2] ^= 0x01;
        match h.verify_crc(&header, &after) {
            Some(CrcStatus::Mismatch { stored, computed }) => {
                assert_eq!(stored, crc);
                assert_ne!(computed, crc);
            }
            other => panic!("expected Mismatch, got {other:?}"),
        }
        // A flip *past* the allocation field (byte 18, well beyond the
        // first 16 bytes) leaves the CRC valid — those bits aren't
        // protected (Table 3-B.5: scalefactors/samples excluded).
        let mut after2 = crc.to_be_bytes().to_vec();
        after2.extend_from_slice(&audio);
        after2[2 + 18] ^= 0xFF;
        assert_eq!(h.verify_crc(&header, &after2), Some(CrcStatus::Ok(crc)));
    }

    #[test]
    fn crc_detects_corruption_in_header_bits_16_31() {
        // Header bits 16..31 (bytes 2,3) are protected. Two headers that
        // differ only in those bits (different sampling/bitrate) produce
        // different CRCs over the same allocation field.
        let h_a = build_header(1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let h_b = build_header(1, 0b11, 0, 0b0100, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let pa = FrameHeader::parse(&h_a).unwrap();
        let pb = FrameHeader::parse(&h_b).unwrap();
        let audio = vec![0x3Cu8; 32];
        let ca = pa.compute_crc(&h_a, &audio).unwrap();
        let cb = pb.compute_crc(&h_b, &audio).unwrap();
        assert_ne!(ca, cb, "CRC must cover header bits 16..31");
    }

    #[test]
    fn crc_none_when_word_truncated() {
        let header = build_header(1, 0b11, 0, 0b1000, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        // Only one byte after the header: not even the CRC word fits.
        assert_eq!(h.verify_crc(&header, &[0xAB]), None);
        // CRC word present but the allocation field is truncated.
        assert_eq!(h.verify_crc(&header, &[0xAB, 0xCD, 0x00]), None);
    }
}
