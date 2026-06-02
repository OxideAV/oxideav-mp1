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
    /// The 2-bit `layer` field selected a layer the crate does not
    /// decode (Layer III `0b01` or the reserved value `0b00`). The
    /// historical variant name (`NotLayer1`) is preserved for API
    /// stability; Layer II is now decoded too, so this only fires on
    /// `0b01` and `0b00`. Carries the raw 2-bit value that was read.
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
            HeaderError::NotLayer1(v) => write!(
                f,
                "layer field 0b{v:02b} is not Layer I or Layer II (not supported by this crate)"
            ),
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

/// The `layer` field, as the variants this crate decodes (§2.4.2.3).
///
/// The crate handles Layer I (its original scope) and Layer II (added
/// for the §2.4.3.3 path). Layer III and the reserved value `0b00` are
/// rejected as [`HeaderError::NotLayer1`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer {
    /// `'11'` — Layer I (§2.4.1.5 / §2.4.3.2).
    I,
    /// `'10'` — Layer II (§2.4.1.6 / §2.4.3.3).
    II,
}

/// The 11172-3 §2.4.2.3 `bitrate_index` → bitrate ladder, **Layer II
/// column** (`ID == 1`, MPEG-1), in kbit/s.
///
/// Index `0b0000` is the *free format* condition; index `0b1111` is
/// *forbidden*. The intermediate indices map to the fixed Layer II
/// bitrates listed in the standard's table.
const LAYER2_BITRATE_KBPS_MPEG1: [u16; 14] = [
    32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384,
];

/// The 13818-3 §2.4.2.3 `bitrate_index` → bitrate ladder, **Layer II
/// column** (`ID == 0`, MPEG-2 LSF), in kbit/s.
///
/// LSF shares the *same* ladder between Layer II and Layer III
/// (per 13818-3 §2.4.2.3 table), distinct from the LSF Layer I ladder.
const LAYER2_BITRATE_KBPS_LSF: [u16; 14] =
    [8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160];

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

/// A fully decoded MPEG-1 / MPEG-2 LSF Audio frame header (§2.4.1.3).
///
/// Carries the thirteen header fields verbatim plus the [`Layer`]
/// selection so a parsed header drives both frame-length computation
/// and the per-layer audio-data decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameHeader {
    /// `ID` — algorithm identifier.
    pub id: Id,
    /// `layer`: which Layer this frame carries. Layer I and Layer II
    /// are supported; other values produce
    /// [`HeaderError::NotLayer1`] at parse time.
    pub layer: Layer,
    /// `protection_bit`: `true` when **no** redundancy (CRC) was
    /// added. When `false`, a 16-bit CRC follows the header
    /// (§2.4.1.4).
    pub protection: bool,
    /// `bitrate_index` interpreted against the appropriate ladder for
    /// the parsed [`Layer`].
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

        // layer: '11' selects Layer I, '10' selects Layer II
        // (§2.4.2.3). '01' (Layer III) and '00' (reserved) are not
        // supported by this crate. The `NotLayer1` variant is retained
        // for compatibility with the original Layer-I-only API; Layer
        // III and the reserved value are reported via the broader
        // `UnsupportedLayer`.
        let parsed_layer = match layer {
            0b11 => Layer::I,
            0b10 => Layer::II,
            // 0b01 (Layer III) / 0b00 (reserved): rejected. The
            // `NotLayer1` variant name is preserved from the
            // Layer-I-only era for API stability.
            _ => return Err(HeaderError::NotLayer1(layer)),
        };

        // ID selects which bitrate ladder and which sampling_frequency
        // table apply: 11172-3 §2.4.2.3 for ID==1, 13818-3 §2.4.2.3
        // (LSF) for ID==0. The ladder column further depends on the
        // selected Layer.
        let id = if id_bit == 1 { Id::Mpeg } else { Id::Mpeg2Lsf };
        let bitrate_table: &[u16; 14] = match (id, parsed_layer) {
            (Id::Mpeg, Layer::I) => &LAYER1_BITRATE_KBPS_MPEG1,
            (Id::Mpeg, Layer::II) => &LAYER2_BITRATE_KBPS_MPEG1,
            (Id::Mpeg2Lsf, Layer::I) => &LAYER1_BITRATE_KBPS_LSF,
            (Id::Mpeg2Lsf, Layer::II) => &LAYER2_BITRATE_KBPS_LSF,
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
            layer: parsed_layer,
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
    /// slot when `padding_bit == 1` (slot size = 4 bytes; 384 samples).
    /// Layer II carries 1152 samples per frame in **1-byte** slots, so
    /// the slot count is `floor(144 * bitrate / sampling_frequency)`
    /// plus an optional padding slot (§2.4.2.1 + §2.4.3.1: a Layer II
    /// frame is 3 Layer-I-sized granules, so 12 × 3 = 36 ÷ 1-byte slot
    /// reflects in the per-slot bit budget — the constant 144 falls out
    /// from `samples_per_frame / 8 = 1152 / 8 = 144`). `bitrate` here
    /// is in bit/s (the ladder stores kbit/s, so we scale by 1000).
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
        let base = match self.layer {
            Layer::I => (12 * bitrate_bps) / self.sampling_frequency,
            Layer::II => (144 * bitrate_bps) / self.sampling_frequency,
        };
        Some(base + u32::from(self.padding))
    }

    /// Total length of this frame in bytes, including the four header
    /// bytes (and, when present, the CRC and audio data — all of which
    /// are accounted for by the slot count, since the slot count spans
    /// the whole inter-syncword distance per §2.4.2.1 / §2.4.3.1).
    ///
    /// Layer I slot = 4 bytes; Layer II slot = 1 byte (§2.4.2.1).
    /// Returns `None` for the free-format condition (see
    /// [`slot_count`](Self::slot_count)).
    pub fn frame_length_bytes(self) -> Option<u32> {
        let slot_bytes = match self.layer {
            Layer::I => LAYER1_SLOT_BYTES,
            Layer::II => 1,
        };
        self.slot_count().map(|slots| slots * slot_bytes)
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

/// Result of a §2.4.3.1 free-format frame-length probe.
///
/// When a header carries `bitrate_index == 0b0000` ([`Bitrate::Free`])
/// the §2.4.2.1 slot-count formula is uninvertible from the header
/// alone — the §2.4.3.1 prose states this verbatim: *"If the bitrate
/// index equals '0000', the exact bitrate is not indicated. N can be
/// determined from the distance between consecutive syncwords and the
/// value of the padding bit."* This struct carries the recovered slot
/// count `N`, the implied frame length in bytes, and the back-derived
/// fixed bitrate in kbit/s (free-format streams hold the bitrate
/// constant across consecutive frames, so `N` for the current frame
/// is what the next syncword's offset measures, less the +1 padding
/// slot when `padding_bit == 1`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FreeFormatFrameLength {
    /// The §2.4.2.1 base slot count `N = floor(L · bitrate / Fs)` for
    /// this stream, with `L = 12` for Layer I and `L = 144` for
    /// Layer II (§2.4.3.1 prose). This is the per-frame slot count
    /// when `padding_bit == 0`; padded frames carry `N + 1` slots.
    pub base_slot_count: u32,
    /// Total length of the current frame in bytes, including the
    /// header (and CRC and padding slot if present). The Layer I slot
    /// is four bytes and the Layer II slot is one byte (§2.4.2.1).
    pub frame_length_bytes: u32,
    /// The fixed-but-unsignalled bitrate in kbit/s, recovered as
    /// `kbps = N · Fs / (L · 1000)` from the §2.4.3.1 inversion of
    /// `N = floor(L · bitrate / Fs)`. May be zero when `Fs / (L · 1000)`
    /// exceeds `N` (the §2.4.3.1 prose does not constrain this case;
    /// it is reported verbatim so the caller can apply its own
    /// minimum-bitrate sanity policy).
    pub bitrate_kbps: u32,
}

/// Errors that can arise while probing a free-format frame's length
/// (§2.4.3.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreeFormatProbeError {
    /// The supplied header does not carry [`Bitrate::Free`]; callers
    /// should use [`FrameHeader::frame_length_bytes`] for non-free
    /// frames.
    NotFreeFormat,
    /// No next-frame syncword was found in `after_header` whose
    /// `(ID, layer, sampling_frequency, mode)` matches the current
    /// frame's. §2.4.3.1 implies the bitrate is held constant across a
    /// free-format stream; a stream-parameter mismatch in the candidate
    /// next-frame header is therefore rejected rather than acted on.
    NoNextSync,
    /// A candidate next syncword was found, but the distance from the
    /// current header to it does not factor cleanly into the
    /// `(N + padding_bit)` slot count §2.4.3.1 prescribes: either the
    /// distance is below the §2.4.2.1 minimum frame size (four bytes
    /// for the header alone, six with a CRC) or the §2.4.3.1 prose's
    /// integer-slot-distance invariant is violated by a Layer I slot
    /// distance that is not a multiple of four bytes.
    InconsistentDistance,
}

impl core::fmt::Display for FreeFormatProbeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FreeFormatProbeError::NotFreeFormat => {
                write!(f, "header bitrate_index is not 0b0000 (free format)")
            }
            FreeFormatProbeError::NoNextSync => {
                write!(
                    f,
                    "no matching next-frame syncword found after the free-format frame"
                )
            }
            FreeFormatProbeError::InconsistentDistance => write!(
                f,
                "next-syncword distance does not factor into integer §2.4.3.1 slots"
            ),
        }
    }
}

impl std::error::Error for FreeFormatProbeError {}

/// Probe the byte-distance to the next consecutive syncword to recover
/// a free-format frame's length (§2.4.3.1).
///
/// The §2.4.3.1 prose states verbatim: *"If the bitrate index equals
/// '0000', the exact bitrate is not indicated. N can be determined
/// from the distance between consecutive syncwords and the value of
/// the padding bit."* This helper performs that inversion:
///
/// 1. Verify the supplied `header` is in free-format mode
///    ([`Bitrate::Free`]); the routine is otherwise inapplicable.
/// 2. Search `after_header` for the next position whose four bytes
///    parse as a valid header for the same stream — same
///    `(ID, layer, sampling_frequency, mode)`. §2.4.3.1 implies the
///    bitrate is held constant across the free-format stream, so a
///    mismatched stream-parameter candidate is rejected rather than
///    acted on. Free-format streams are also free of intervening
///    metadata (no ID3, no Xing tag at this layer of the stack), so
///    the next syncword is at exactly the next-frame offset.
/// 3. Compute the slot distance from the start of the current header
///    to that next syncword (Layer I slot = 4 bytes, Layer II slot = 1
///    byte, §2.4.2.1) and subtract the current frame's
///    `padding_bit` (§2.4.3.1) to recover the base slot count `N`.
/// 4. Report `N`, the current frame's byte length (`(N + padding_bit)
///    · slot_bytes`), and the back-derived bitrate
///    `kbps = N · Fs / (L · 1000)` (§2.4.3.1 inverted, `L = 12` for
///    Layer I, `L = 144` for Layer II).
///
/// `after_header` begins immediately after the four bytes of the
/// current frame's header. The current header's four bytes are NOT
/// part of `after_header`; the distance computation accounts for the
/// header bytes internally.
pub fn detect_free_format_frame_length(
    header: &FrameHeader,
    after_header: &[u8],
) -> Result<FreeFormatFrameLength, FreeFormatProbeError> {
    if !matches!(header.bitrate, Bitrate::Free) {
        return Err(FreeFormatProbeError::NotFreeFormat);
    }
    let slot_bytes: u32 = match header.layer {
        Layer::I => LAYER1_SLOT_BYTES,
        Layer::II => 1,
    };
    // The §2.4.3.1 N formula: N = L · bitrate / Fs.
    let l: u32 = match header.layer {
        Layer::I => 12,
        Layer::II => 144,
    };

    // Scan after_header for the next matching syncword. The offset
    // we want is the byte position OF THAT SYNCWORD relative to the
    // start of the CURRENT header; since `after_header` begins right
    // after the four header bytes we add HEADER_BYTES back in.
    const HEADER_BYTES: u32 = 4;
    let mut cursor: usize = 0;
    while cursor + 4 <= after_header.len() {
        if let Ok(next) = FrameHeader::parse(&after_header[cursor..cursor + 4]) {
            // §2.4.3.1: the bitrate is held constant across the
            // free-format stream, so the next frame must carry the
            // same stream parameters. Reject candidates that differ.
            if next.id == header.id
                && next.layer == header.layer
                && next.sampling_frequency == header.sampling_frequency
                && next.mode == header.mode
            {
                let distance_bytes = HEADER_BYTES as usize + cursor;
                let distance_bytes_u32 = match u32::try_from(distance_bytes) {
                    Ok(v) => v,
                    Err(_) => return Err(FreeFormatProbeError::InconsistentDistance),
                };
                // §2.4.3.1: distance is an integer number of slots.
                if distance_bytes_u32 % slot_bytes != 0 {
                    return Err(FreeFormatProbeError::InconsistentDistance);
                }
                let slots_total = distance_bytes_u32 / slot_bytes;
                let padding = u32::from(header.padding);
                if slots_total <= padding {
                    return Err(FreeFormatProbeError::InconsistentDistance);
                }
                let base_slot_count = slots_total - padding;
                let frame_length_bytes = distance_bytes_u32;
                // §2.4.3.1 inverted: kbps = N · Fs / (L · 1000).
                // Use u64 arithmetic to avoid overflow at the high
                // end of the ladder.
                let bitrate_kbps_u64 =
                    (base_slot_count as u64 * header.sampling_frequency as u64) / (l as u64 * 1000);
                let bitrate_kbps = u32::try_from(bitrate_kbps_u64).unwrap_or(u32::MAX);
                return Ok(FreeFormatFrameLength {
                    base_slot_count,
                    frame_length_bytes,
                    bitrate_kbps,
                });
            }
            // Not a stream-matching candidate; keep scanning. Skip
            // one byte so we can find a candidate that begins one
            // byte later (free-format frame lengths are not required
            // to be byte-aligned to anything other than the slot,
            // and a candidate may share the syncword pattern with
            // payload bytes).
        }
        cursor += 1;
    }
    Err(FreeFormatProbeError::NoNextSync)
}

/// The §2.4.3.1 CRC-16 generator polynomial, expressed as the 16-bit
/// feedback mask used by the bit-serial shift register.
///
/// §2.4.3.1 specifies `G(X) = X^16 + X^15 + X^2 + 1` (page 36 of the
/// staged ISO/IEC 11172-3 PDF, recovered from the typeset equation
/// image). In a 16-bit register the `X^16` term is the bit shifted out
/// at the top, so the feedback taps below it are `X^15`, `X^2`, `X^0` —
/// i.e. the mask `0b1000_0000_0000_0101 = 0x8005`.
pub(crate) const CRC16_POLY: u16 = 0x8005;

/// The §2.4.3.1 CRC-16 shift-register initial state.
///
/// §2.4.3.1: "The initial state of the shift register is
/// '1111 1111 1111 1111'." — numerically `0xFFFF`.
pub(crate) const CRC16_INIT: u16 = 0xFFFF;

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
pub(crate) fn crc16_update_bits(mut reg: u16, bytes: &[u8], bit_count: usize) -> u16 {
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
    fn rejects_layer3_and_reserved() {
        // layer '01' = Layer III (not decoded by this crate), '00' =
        // reserved. Layer II ('10') is now accepted (§2.4.1.6) and is
        // exercised by the Layer II tests; only the two unsupported
        // codepoints are rejected here.
        for (bits, _name) in [(0b01u32, "III"), (0b00, "reserved")] {
            let bytes = build_header(1, bits, 1, 0b1000, 0b01, 0, 0, 0, 0, 0, 1, 0);
            assert_eq!(
                FrameHeader::parse(&bytes),
                Err(HeaderError::NotLayer1(bits as u8))
            );
        }
    }

    #[test]
    fn accepts_layer2_header() {
        // Layer II (layer '10'), MPEG-1, no CRC, 192 kbit/s
        // (index 0b1010 on the MPEG-1 Layer II ladder), 44.1 kHz,
        // stereo. Frame should parse cleanly with `layer == Layer::II`
        // and `bitrate == Fixed(192)`.
        let bytes = build_header(1, 0b10, 1, 0b1010, 0b00, 0, 0, 0b00, 0, 0, 1, 0);
        let h = FrameHeader::parse(&bytes).unwrap();
        assert_eq!(h.layer, Layer::II);
        assert_eq!(h.bitrate, Bitrate::Fixed(192));
        assert_eq!(h.sampling_frequency, 44_100);
        assert_eq!(h.mode, Mode::Stereo);
        // §2.4.2.1 Layer II framing: 1-byte slots, 1152 samples / frame.
        // 144 * 192000 / 44100 = 626.939... -> floor = 626 slots = 626 bytes.
        // (ffmpeg's `-f mp2` muxer emits the same byte count for an
        // unpadded MPEG-1 LII / 192 kbit/s / 44.1 kHz frame.)
        assert_eq!(h.slot_count(), Some(626));
        assert_eq!(h.frame_length_bytes(), Some(626));
    }

    #[test]
    fn layer2_mpeg1_bitrate_ladder() {
        // §2.4.2.3 Layer II column (MPEG-1), indices 0b0001..0b1110.
        let expected = [
            32u16, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384,
        ];
        for (i, &kbps) in expected.iter().enumerate() {
            let idx = (i + 1) as u32;
            let bytes = build_header(1, 0b10, 1, idx, 0b00, 0, 0, 0, 0, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(
                h.bitrate,
                Bitrate::Fixed(kbps),
                "L2 MPEG-1 index 0b{idx:04b}"
            );
        }
    }

    #[test]
    fn layer2_lsf_bitrate_ladder() {
        // 13818-3 §2.4.2.3 Layer II,III column (LSF), indices 0b0001..0b1110.
        let expected = [8u16, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160];
        for (i, &kbps) in expected.iter().enumerate() {
            let idx = (i + 1) as u32;
            // LSF ID=0, 24 kHz (0b01).
            let bytes = build_header(0, 0b10, 1, idx, 0b01, 0, 0, 0, 0, 0, 1, 0);
            let h = FrameHeader::parse(&bytes).unwrap();
            assert_eq!(h.bitrate, Bitrate::Fixed(kbps), "L2 LSF index 0b{idx:04b}");
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

    // ---- §2.4.3.1 free-format frame-length probe -------------------

    #[test]
    fn free_format_probe_rejects_non_free_header() {
        // A non-free-format header is not the §2.4.3.1 prose's domain.
        let header = build_header(1, 0b11, 1, 0b1000, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        assert_eq!(
            detect_free_format_frame_length(&h, &[]),
            Err(FreeFormatProbeError::NotFreeFormat)
        );
    }

    #[test]
    fn free_format_probe_reports_no_next_sync_when_absent() {
        // Free-format frame, 44.1 kHz, mono. No next syncword in the
        // payload — pure noise.
        let header = build_header(1, 0b11, 1, 0b0000, 0b00, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        let payload = vec![0xA5u8; 200];
        assert_eq!(
            detect_free_format_frame_length(&h, &payload),
            Err(FreeFormatProbeError::NoNextSync)
        );
    }

    #[test]
    fn free_format_probe_layer1_recovers_n_and_bitrate() {
        // Layer I, MPEG-1, 32 kHz, mono, no padding, free format.
        //
        // §2.4.3.1: N = 12 · bitrate / Fs. Pick a free-format Layer I
        // stream that would correspond to a 128 kbit/s rate (not on the
        // fixed ladder for this exercise, but the math is the same):
        // N = 12 · 128000 / 32000 = 48 slots = 192 bytes.
        let header = build_header(1, 0b11, 1, 0b0000, 0b10, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        assert_eq!(h.bitrate, Bitrate::Free);
        let frame_bytes: u32 = 192;
        // The payload between two free-format headers is (frame_bytes
        // - 4) bytes; the next header begins exactly at the
        // frame-length offset relative to the start of the current
        // header.
        let mut payload = vec![0x00u8; (frame_bytes - 4) as usize];
        // Stamp a small recognisable pattern that does not collide with
        // the syncword at any byte position.
        for (i, b) in payload.iter_mut().enumerate() {
            *b = ((i as u8) & 0x7F).wrapping_add(0x40);
        }
        // The next-frame header carries identical stream parameters
        // (free-format implies the bitrate is held constant — same
        // (ID, layer, Fs, mode)).
        let next_header = build_header(1, 0b11, 1, 0b0000, 0b10, 0, 0, 0b11, 0, 0, 1, 0);
        payload.extend_from_slice(&next_header);
        // A few extra trailing bytes after the next header should not
        // affect detection.
        payload.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let probe = detect_free_format_frame_length(&h, &payload).expect("probe");
        assert_eq!(probe.base_slot_count, 48);
        assert_eq!(probe.frame_length_bytes, 192);
        assert_eq!(probe.bitrate_kbps, 128);
    }

    #[test]
    fn free_format_probe_layer1_padded_subtracts_one_slot() {
        // Same as above but with padding_bit == 1: the next syncword
        // is one slot (four bytes) further away, but `N` itself stays
        // the same. We construct the next header to begin at offset
        // 196 (192 + 4) so the probe must subtract the padding slot to
        // recover N = 48.
        let header = build_header(1, 0b11, 1, 0b0000, 0b10, 1, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        assert!(h.padding);
        let next_offset: u32 = 196;
        let mut payload = vec![0x00u8; (next_offset - 4) as usize];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = ((i as u8) & 0x7F).wrapping_add(0x40);
        }
        let next_header = build_header(1, 0b11, 1, 0b0000, 0b10, 1, 0, 0b11, 0, 0, 1, 0);
        payload.extend_from_slice(&next_header);
        let probe = detect_free_format_frame_length(&h, &payload).expect("probe");
        assert_eq!(probe.base_slot_count, 48);
        assert_eq!(probe.frame_length_bytes, 196);
        assert_eq!(probe.bitrate_kbps, 128);
    }

    #[test]
    fn free_format_probe_layer2_uses_unit_byte_slot() {
        // Layer II at 32 kHz: §2.4.3.1 says N = 144 · bitrate / Fs.
        // Pick a hypothetical 96 kbit/s free-format frame:
        // N = 144 · 96000 / 32000 = 432 bytes (Layer II slot = 1 byte).
        let header = build_header(1, 0b10, 1, 0b0000, 0b10, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        assert_eq!(h.layer, Layer::II);
        let frame_bytes: u32 = 432;
        let mut payload = vec![0u8; (frame_bytes - 4) as usize];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = ((i as u8) & 0x3F).wrapping_add(0x10);
        }
        let next_header = build_header(1, 0b10, 1, 0b0000, 0b10, 0, 0, 0b11, 0, 0, 1, 0);
        payload.extend_from_slice(&next_header);
        let probe = detect_free_format_frame_length(&h, &payload).expect("probe");
        assert_eq!(probe.base_slot_count, 432);
        assert_eq!(probe.frame_length_bytes, 432);
        assert_eq!(probe.bitrate_kbps, 96);
    }

    #[test]
    fn free_format_probe_rejects_stream_parameter_mismatch() {
        // Free-format Layer I frame at 44.1 kHz, mono. The next header
        // in the buffer is on a DIFFERENT sampling frequency — the
        // probe must walk past it and either find a matching one or
        // report NoNextSync. We arrange no matching follow-up.
        let header = build_header(1, 0b11, 1, 0b0000, 0b00, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        let mut payload = vec![0u8; 100];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = ((i as u8) & 0x7F).wrapping_add(0x40);
        }
        // Inject a 32 kHz header (sampling 0b10) instead of 44.1
        // (0b00) — this candidate must be rejected.
        let wrong_header = build_header(1, 0b11, 1, 0b0000, 0b10, 0, 0, 0b11, 0, 0, 1, 0);
        payload.extend_from_slice(&wrong_header);
        // …and no other syncword in the rest of the payload.
        payload.extend_from_slice(&[0xFEu8, 0xFE, 0xFE, 0xFE]);
        assert_eq!(
            detect_free_format_frame_length(&h, &payload),
            Err(FreeFormatProbeError::NoNextSync)
        );
    }

    #[test]
    fn free_format_probe_layer1_rejects_non_slot_distance() {
        // Layer I slot = 4 bytes (§2.4.2.1). A next syncword whose
        // byte-distance from the current header is not a multiple of
        // four breaks the §2.4.3.1 integer-slot-distance invariant
        // and must be rejected.
        //
        // Build a payload such that the next syncword starts at byte
        // offset 191 from the current header (191 = 4·47 + 3 — not a
        // slot multiple).
        let header = build_header(1, 0b11, 1, 0b0000, 0b00, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        // After-header length: 191 - 4 = 187 bytes of pre-sync filler.
        let mut payload = vec![0x55u8; 187];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = ((i as u8) & 0x3F).wrapping_add(0x10);
        }
        let next_header = build_header(1, 0b11, 1, 0b0000, 0b00, 0, 0, 0b11, 0, 0, 1, 0);
        payload.extend_from_slice(&next_header);
        assert_eq!(
            detect_free_format_frame_length(&h, &payload),
            Err(FreeFormatProbeError::InconsistentDistance)
        );
    }

    #[test]
    fn free_format_probe_layer1_finds_next_syncword_at_min_distance() {
        // Smallest sensible free-format Layer I frame: the four-byte
        // header alone (no audio payload, no CRC). This is degenerate
        // but exercises the boundary of the probe.
        //
        // base_slot_count = 1, frame_length_bytes = 4 (one Layer I
        // slot), bitrate = 1 · Fs / (12 · 1000) — with Fs = 48 000 the
        // bitrate is 4 kbit/s. This is below any conformant Layer I
        // bitrate but the probe reports it verbatim per the spec
        // text (the spec does not constrain a minimum here).
        let header = build_header(1, 0b11, 1, 0b0000, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        let next_header = build_header(1, 0b11, 1, 0b0000, 0b01, 0, 0, 0b11, 0, 0, 1, 0);
        let probe = detect_free_format_frame_length(&h, &next_header).expect("probe");
        assert_eq!(probe.base_slot_count, 1);
        assert_eq!(probe.frame_length_bytes, 4);
        assert_eq!(probe.bitrate_kbps, 4);
    }

    #[test]
    fn free_format_probe_layer2_padded_subtracts_one_byte() {
        // Layer II at 24 kHz LSF, 64 kbit/s would give
        // N = 144 · 64000 / 24000 = 384 bytes. With padding_bit = 1,
        // the next syncword is at 385 bytes — the probe must subtract
        // the one-byte padding slot.
        let header = build_header(0, 0b10, 1, 0b0000, 0b01, 1, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        assert_eq!(h.id, Id::Mpeg2Lsf);
        assert_eq!(h.sampling_frequency, 24_000);
        let next_offset: u32 = 385;
        let mut payload = vec![0u8; (next_offset - 4) as usize];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = ((i as u8) & 0x3F).wrapping_add(0x10);
        }
        let next_header = build_header(0, 0b10, 1, 0b0000, 0b01, 1, 0, 0b11, 0, 0, 1, 0);
        payload.extend_from_slice(&next_header);
        let probe = detect_free_format_frame_length(&h, &payload).expect("probe");
        assert_eq!(probe.base_slot_count, 384);
        assert_eq!(probe.frame_length_bytes, 385);
        assert_eq!(probe.bitrate_kbps, 64);
    }

    #[test]
    fn free_format_probe_skips_past_syncword_pattern_in_payload() {
        // A free-format payload byte may incidentally start a valid
        // 32-bit header by chance. The probe must accept the first
        // *matching* candidate; this test confirms it scans
        // byte-by-byte (not 4-byte-by-4-byte) and lands on the right
        // next-syncword position even when the payload has an
        // intervening non-matching candidate sync earlier on.
        //
        // Build the current frame at 44.1 kHz mono. Plant a candidate
        // 32 kHz header at offset 96 in the payload (rejected as
        // mismatching), then the real next 44.1 kHz header at offset
        // 196 — total frame length 200 bytes.
        let header = build_header(1, 0b11, 1, 0b0000, 0b00, 0, 0, 0b11, 0, 0, 1, 0);
        let h = FrameHeader::parse(&header).unwrap();
        let mut payload = vec![0u8; 92]; // 92 + 4 (header) = 96 offset
        for (i, b) in payload.iter_mut().enumerate() {
            *b = ((i as u8) & 0x3F).wrapping_add(0x10);
        }
        // Mismatching candidate (32 kHz) at offset 96.
        payload.extend_from_slice(&build_header(
            1, 0b11, 1, 0b0000, 0b10, 0, 0, 0b11, 0, 0, 1, 0,
        ));
        // Fill until offset 196 from header start (so after-header
        // offset 192).
        let target = 196usize;
        while 4 + payload.len() < target {
            payload.push(0x33);
        }
        // The real matching next header.
        payload.extend_from_slice(&build_header(
            1, 0b11, 1, 0b0000, 0b00, 0, 0, 0b11, 0, 0, 1, 0,
        ));
        let probe = detect_free_format_frame_length(&h, &payload).expect("probe");
        // 196 bytes / 4-byte slot = 49 slots; padding 0 → N = 49.
        assert_eq!(probe.frame_length_bytes, 196);
        assert_eq!(probe.base_slot_count, 49);
        // kbps = 49 · 44100 / (12 · 1000) = 180 (truncated, the spec
        // formula is an integer division).
        assert_eq!(probe.bitrate_kbps, 180);
    }
}
