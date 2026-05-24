//! MPEG-1 Audio **Layer II** Annex B tables (3-B.2a..d, 3-B.4),
//! transcribed verbatim from the staged ISO/IEC 11172-3 (1993) PDF
//! page renders (PDF pages 52-53 for B.2a/B.2b, 54-55 for B.2c/B.2d,
//! 59 for B.4).
//!
//! ## Table 3-B.2x — "Possible quantization per subband"
//!
//! Four sub-tables differ by the `(sampling_frequency, bitrate per
//! channel)` cell. Each row gives, for one subband, the `nbal` width
//! (2, 3, or 4 bits) of the `audio_data()` allocation field and, at
//! columns indexed by the `nbal`-bit allocation value, the
//! corresponding `nlevels` (`-` = invalid).
//!
//! `nbal == 0` rows are subbands at or above `sblimit`; they carry no
//! bits in the bitstream and read as `nlevels = 0` (the "all-zero
//! samples" condition of §2.4.3.3.5).
//!
//! ## Table 3-B.4 — "Layer II classes of quantization"
//!
//! Indexed by `nlevels`. Gives the requantization constants `C` and
//! `D`, the `grouping` flag (true = three sub-band samples carried by
//! one combined codeword that degroups by repeated `% nlevels` and
//! `/ nlevels`; false = three separable codewords), the number of
//! samples carried per codeword (3 if grouped, 1 if not), and the
//! number of bits per codeword.
//!
//! See [`QuantClass::bits_per_sample`] for the per-sample-after-
//! degrouping bit width that the §2.4.3.3.4 inversion (MSB-flip)
//! operates on — for grouped classes this is `ceil(log2(nlevels))`
//! per sample, *not* `bits_per_codeword`.

use crate::header::{FrameHeader, Id, Layer};

/// One row of a Table 3-B.2x — the bit allocation table for one
/// subband.
///
/// `nbal == 0` means the subband is at or above `sblimit` and carries
/// no bits / no samples in the bitstream.
#[derive(Debug, Clone, Copy)]
struct AllocationRow {
    /// Bit width of the `allocation[ch][sb]` field in the bitstream
    /// for this subband (2, 3, or 4 — `0` for subbands above sblimit).
    nbal: u8,
    /// Array of `nlevels` indexed by the `nbal`-bit allocation value
    /// `1..(1<<nbal)`. Slot `0` of this array always corresponds to
    /// allocation index `1`; allocation `0` is "no samples", handled
    /// outside this array.
    ///
    /// A `None` slot represents the `-` cells in the PDF rendering —
    /// allocation values for which Table 3-B.2x lists nothing. The
    /// decoder rejects them as [`crate::decode::DecodeError::InvalidAllocation`].
    levels: &'static [Option<u16>],
}

/// A complete per-frame bit-allocation table (one of B.2a..d).
#[derive(Debug, Clone, Copy)]
pub struct AllocationTable {
    /// `sblimit` for this table (8, 12, 27, or 30 across B.2a..d).
    sblimit_value: usize,
    rows: &'static [AllocationRow; 32],
}

impl AllocationTable {
    /// `sblimit`: the number of subbands carried in `audio_data()`.
    /// Subbands `[sblimit, 32)` are not present and decode as zero
    /// per §2.4.3.3.5.
    pub fn sblimit(&self) -> usize {
        self.sblimit_value
    }

    /// `nbal(sb)`: number of bits the `allocation[ch][sb]` field
    /// occupies for subband `sb` under this table.
    pub fn nbal(&self, sb: usize) -> u8 {
        self.rows[sb].nbal
    }

    /// Resolve a raw allocation index to a quantization class for
    /// subband `sb` under this table. Returns `None` for invalid (`-`)
    /// allocation values or for allocation `0` (no samples).
    pub fn quant_class(&self, sb: usize, alloc: u8) -> Option<&'static QuantClass> {
        if alloc == 0 {
            return None;
        }
        let levels = self.rows[sb].levels;
        let idx = (alloc as usize).checked_sub(1)?;
        let nlevels = (*levels.get(idx)?)?;
        lookup_class(nlevels)
    }
}

/// One row of Table 3-B.4 — a quantization class indexed by `nlevels`.
///
/// `bits_per_codeword` is the number of bits read from the bitstream
/// per codeword (a `samplecode` for grouped classes, an individual
/// `sample` for non-grouped classes). `samples_per_codeword` is 3 for
/// grouped classes and 1 otherwise.
#[derive(Debug, Clone, Copy)]
pub struct QuantClass {
    /// Number of quantization steps (3, 5, 7, ..., 65535).
    pub nlevels: u16,
    /// Linear-formula multiplier C from §2.4.3.3.4.
    pub c: f64,
    /// Linear-formula offset D from §2.4.3.3.4.
    pub d: f64,
    /// `true` when three sub-band samples are carried by one combined
    /// codeword (degrouped by repeated `% nlevels`).
    pub grouping: bool,
    /// Samples per codeword: 3 if `grouping`, 1 otherwise.
    pub samples_per_codeword: u8,
    /// Number of bits the codeword occupies.
    pub bits_per_codeword: u8,
}

impl QuantClass {
    /// The per-*sample* bit width used by the §2.4.3.3.4 MSB inversion
    /// + two's-complement interpretation.
    ///
    /// For non-grouped classes this is `bits_per_codeword`; for grouped
    /// classes (3 samples in one codeword) it is `ceil(log2(nlevels))`
    /// since each separated sample (after `% nlevels`) is `0..nlevels`
    /// and packs into that many bits.
    pub fn bits_per_sample(&self) -> u8 {
        if self.grouping {
            // ceil(log2(nlevels)).
            let n = self.nlevels as u32;
            // For nlevels in {3,5,9} this evaluates to {2,3,4}.
            32 - (n - 1).leading_zeros() as u8
        } else {
            self.bits_per_codeword
        }
    }
}

// ---- Table 3-B.4 ------------------------------------------------

/// Table 3-B.4 — Layer II classes of quantization (transcribed
/// verbatim from PDF page 59 / spec page 53).
///
/// The visual table lists exactly seventeen rows: `nlevels` of
/// `{3, 5, 7, 9, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191,
/// 16383, 32767, 65535}`. `C` and `D` are decimal fractions in
/// European-comma notation (parsed here as decimal points).
pub const QUANT_CLASSES: &[QuantClass] = &[
    QuantClass {
        nlevels: 3,
        c: 1.33333333333,
        d: 0.50000000000,
        grouping: true,
        samples_per_codeword: 3,
        bits_per_codeword: 5,
    },
    QuantClass {
        nlevels: 5,
        c: 1.60000000000,
        d: 0.50000000000,
        grouping: true,
        samples_per_codeword: 3,
        bits_per_codeword: 7,
    },
    QuantClass {
        nlevels: 7,
        c: 1.14285714286,
        d: 0.25000000000,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 3,
    },
    QuantClass {
        nlevels: 9,
        c: 1.77777777777,
        d: 0.50000000000,
        grouping: true,
        samples_per_codeword: 3,
        bits_per_codeword: 10,
    },
    QuantClass {
        nlevels: 15,
        c: 1.06666666666,
        d: 0.12500000000,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 4,
    },
    QuantClass {
        nlevels: 31,
        c: 1.03225806452,
        d: 0.06250000000,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 5,
    },
    QuantClass {
        nlevels: 63,
        c: 1.01587301587,
        d: 0.03125000000,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 6,
    },
    QuantClass {
        nlevels: 127,
        c: 1.00787401575,
        d: 0.01562500000,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 7,
    },
    QuantClass {
        nlevels: 255,
        c: 1.00392156863,
        d: 0.00781250000,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 8,
    },
    QuantClass {
        nlevels: 511,
        c: 1.00195694716,
        d: 0.00390625000,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 9,
    },
    QuantClass {
        nlevels: 1023,
        c: 1.00097751711,
        d: 0.00195312500,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 10,
    },
    QuantClass {
        nlevels: 2047,
        c: 1.00048851979,
        d: 0.00097656250,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 11,
    },
    QuantClass {
        nlevels: 4095,
        c: 1.00024420024,
        d: 0.00048828125,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 12,
    },
    QuantClass {
        nlevels: 8191,
        c: 1.00012208522,
        d: 0.00024414063,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 13,
    },
    QuantClass {
        nlevels: 16383,
        c: 1.00006103888,
        d: 0.00012207031,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 14,
    },
    QuantClass {
        nlevels: 32767,
        c: 1.00003051851,
        d: 0.00006103516,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 15,
    },
    QuantClass {
        nlevels: 65535,
        c: 1.00001525902,
        d: 0.00003051758,
        grouping: false,
        samples_per_codeword: 1,
        bits_per_codeword: 16,
    },
];

fn lookup_class(nlevels: u16) -> Option<&'static QuantClass> {
    QUANT_CLASSES.iter().find(|c| c.nlevels == nlevels)
}

// ---- Table 3-B.2a (sblimit=27, sum_of_nbal=88) -----------------

/// `Some(n)` for a tabulated nlevels at column position
/// `allocation_index - 1`; `None` for the `-` cells.
const ROW_B2A_0_2: &[Option<u16>] = &[
    Some(3),
    Some(7),
    Some(15),
    Some(31),
    Some(63),
    Some(127),
    Some(255),
    Some(511),
    Some(1023),
    Some(2047),
    Some(4095),
    Some(8191),
    Some(16383),
    Some(32767),
    Some(65535),
];
const ROW_B2A_3_10: &[Option<u16>] = &[
    Some(3),
    Some(5),
    Some(7),
    Some(9),
    Some(15),
    Some(31),
    Some(63),
    Some(127),
    Some(255),
    Some(511),
    Some(1023),
    Some(2047),
    Some(4095),
    Some(8191),
    Some(65535),
];
const ROW_B2A_11_22: &[Option<u16>] = &[
    Some(3),
    Some(5),
    Some(7),
    Some(9),
    Some(15),
    Some(31),
    Some(65535),
];
const ROW_B2A_23_26: &[Option<u16>] = &[Some(3), Some(5), Some(65535)];

const TABLE_B2A_ROWS: [AllocationRow; 32] = {
    let mut rows = [AllocationRow {
        nbal: 0,
        levels: &[],
    }; 32];
    rows[0] = AllocationRow {
        nbal: 4,
        levels: ROW_B2A_0_2,
    };
    rows[1] = AllocationRow {
        nbal: 4,
        levels: ROW_B2A_0_2,
    };
    rows[2] = AllocationRow {
        nbal: 4,
        levels: ROW_B2A_0_2,
    };
    let mut sb = 3;
    while sb <= 10 {
        rows[sb] = AllocationRow {
            nbal: 4,
            levels: ROW_B2A_3_10,
        };
        sb += 1;
    }
    while sb <= 22 {
        rows[sb] = AllocationRow {
            nbal: 3,
            levels: ROW_B2A_11_22,
        };
        sb += 1;
    }
    while sb <= 26 {
        rows[sb] = AllocationRow {
            nbal: 2,
            levels: ROW_B2A_23_26,
        };
        sb += 1;
    }
    rows
};

const TABLE_B2A: AllocationTable = AllocationTable {
    sblimit_value: 27,
    rows: &TABLE_B2A_ROWS,
};

// ---- Table 3-B.2b (sblimit=30, sum_of_nbal=94) -----------------

// B.2b: rows 0..2 nbal=4 with ROW_B2A_0_2-equivalent values; rows
// 3..10 nbal=4 with the same ROW_B2A_3_10 values; rows 11..22 nbal=3
// with ROW_B2A_11_22; rows 23..29 nbal=2 with ROW_B2A_23_26 (PDF page
// 53 confirms identical level vectors to B.2a — only sblimit differs).
const TABLE_B2B_ROWS: [AllocationRow; 32] = {
    let mut rows = [AllocationRow {
        nbal: 0,
        levels: &[],
    }; 32];
    rows[0] = AllocationRow {
        nbal: 4,
        levels: ROW_B2A_0_2,
    };
    rows[1] = AllocationRow {
        nbal: 4,
        levels: ROW_B2A_0_2,
    };
    rows[2] = AllocationRow {
        nbal: 4,
        levels: ROW_B2A_0_2,
    };
    let mut sb = 3;
    while sb <= 10 {
        rows[sb] = AllocationRow {
            nbal: 4,
            levels: ROW_B2A_3_10,
        };
        sb += 1;
    }
    while sb <= 22 {
        rows[sb] = AllocationRow {
            nbal: 3,
            levels: ROW_B2A_11_22,
        };
        sb += 1;
    }
    while sb <= 29 {
        rows[sb] = AllocationRow {
            nbal: 2,
            levels: ROW_B2A_23_26,
        };
        sb += 1;
    }
    rows
};

const TABLE_B2B: AllocationTable = AllocationTable {
    sblimit_value: 30,
    rows: &TABLE_B2B_ROWS,
};

// ---- Table 3-B.2c (sblimit=8, sum_of_nbal=26) ------------------

const ROW_B2C_0_1: &[Option<u16>] = &[
    Some(3),
    Some(5),
    Some(9),
    Some(15),
    Some(31),
    Some(63),
    Some(127),
    Some(255),
    Some(511),
    Some(1023),
    Some(2047),
    Some(4095),
    Some(8191),
    Some(16383),
    Some(32767),
];
const ROW_B2C_2_7: &[Option<u16>] = &[
    Some(3),
    Some(5),
    Some(9),
    Some(15),
    Some(31),
    Some(63),
    Some(127),
];

const TABLE_B2C_ROWS: [AllocationRow; 32] = {
    let mut rows = [AllocationRow {
        nbal: 0,
        levels: &[],
    }; 32];
    rows[0] = AllocationRow {
        nbal: 4,
        levels: ROW_B2C_0_1,
    };
    rows[1] = AllocationRow {
        nbal: 4,
        levels: ROW_B2C_0_1,
    };
    let mut sb = 2;
    while sb <= 7 {
        rows[sb] = AllocationRow {
            nbal: 3,
            levels: ROW_B2C_2_7,
        };
        sb += 1;
    }
    rows
};

const TABLE_B2C: AllocationTable = AllocationTable {
    sblimit_value: 8,
    rows: &TABLE_B2C_ROWS,
};

// ---- Table 3-B.2d (sblimit=12, sum_of_nbal=38) -----------------

const TABLE_B2D_ROWS: [AllocationRow; 32] = {
    let mut rows = [AllocationRow {
        nbal: 0,
        levels: &[],
    }; 32];
    rows[0] = AllocationRow {
        nbal: 4,
        levels: ROW_B2C_0_1,
    };
    rows[1] = AllocationRow {
        nbal: 4,
        levels: ROW_B2C_0_1,
    };
    let mut sb = 2;
    while sb <= 11 {
        rows[sb] = AllocationRow {
            nbal: 3,
            levels: ROW_B2C_2_7,
        };
        sb += 1;
    }
    rows
};

const TABLE_B2D: AllocationTable = AllocationTable {
    sblimit_value: 12,
    rows: &TABLE_B2D_ROWS,
};

/// Select the per-frame bit-allocation table for the given Layer II
/// header (§2.4.3.3.1).
///
/// The decision drivers are `sampling_frequency` and **bitrate per
/// channel** (total bitrate / 2 in any non-mono mode). The per-table
/// "allowed" lists below are read verbatim from the per-table headers
/// in the staged PDF (pages 46-49):
///
/// * **B.2a** (sblimit=27): Fs = 48 kHz at 56/64/80/96/112/128/160/192
///   kbit/s per channel + free format; Fs = 44.1/32 kHz at 56/64/80.
/// * **B.2b** (sblimit=30): Fs = 44.1/32 kHz at 96/112/128/160/192 + free.
/// * **B.2c** (sblimit=8):  Fs = 48/44.1 kHz at 32 and 48.
/// * **B.2d** (sblimit=12): Fs = 32 kHz at 32 and 48.
///
/// For ISO/IEC 13818-3 LSF Fs = 16/22.05/24 kHz the 13818-3 §2.4.3
/// extension specifies one shared table covering all combinations
/// (sblimit=30, structurally identical to B.2b). We map every LSF
/// frame to B.2b accordingly; explicit `Fs ≤ 24 kHz` cases all share
/// the LSF Layer-II "Possible quantization" table reproduced in
/// 13818-3 Annex B which has the same shape as 11172-3 B.2b — see the
/// 13818-3 §B.1 annex page.
///
/// When a `(Fs, kbps-per-channel)` combination is not in any "allowed"
/// list (e.g. an out-of-spec total bitrate × Fs the §2.4.2.3 footnote
/// flags), this falls back to **B.2a** (the broadest 32-kHz/48-kHz
/// table), so malformed streams produce a defined behaviour rather
/// than a panic from a missing-table lookup. Real-world ffmpeg-encoded
/// streams stay within the standard combinations and exercise the
/// explicit branches only.
pub fn layer2_bit_allocation_table(header: &FrameHeader) -> &'static AllocationTable {
    debug_assert!(matches!(header.layer, Layer::II));
    if matches!(header.id, Id::Mpeg2Lsf) {
        // 13818-3 §B.1 LSF Layer II allocation table (sblimit = 30).
        return &TABLE_B2B;
    }
    let kbps_total = match header.bitrate {
        crate::header::Bitrate::Fixed(k) => k,
        crate::header::Bitrate::Free | crate::header::Bitrate::Forbidden => {
            // Free format: §2.4.3.3.1 doesn't specify a fallback;
            // pick B.2a as the broadest 48 kHz table.
            return &TABLE_B2A;
        }
    };
    let kbps_per_ch = if matches!(header.mode, crate::header::Mode::SingleChannel) {
        kbps_total
    } else {
        kbps_total / 2
    };
    let fs = header.sampling_frequency;
    match fs {
        48_000 => match kbps_per_ch {
            32 | 48 => &TABLE_B2C,
            56 | 64 | 80 | 96 | 112 | 128 | 160 | 192 => &TABLE_B2A,
            _ => &TABLE_B2A,
        },
        44_100 => match kbps_per_ch {
            32 | 48 => &TABLE_B2C,
            56 | 64 | 80 => &TABLE_B2A,
            96 | 112 | 128 | 160 | 192 => &TABLE_B2B,
            _ => &TABLE_B2B,
        },
        32_000 => match kbps_per_ch {
            32 | 48 => &TABLE_B2D,
            56 | 64 | 80 => &TABLE_B2A,
            96 | 112 | 128 | 160 | 192 => &TABLE_B2B,
            _ => &TABLE_B2B,
        },
        _ => &TABLE_B2B,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header_mp1(fs: u32, kbps: u16, mode: crate::header::Mode) -> FrameHeader {
        FrameHeader {
            id: Id::Mpeg,
            layer: Layer::II,
            protection: true,
            bitrate: crate::header::Bitrate::Fixed(kbps),
            sampling_frequency: fs,
            padding: false,
            private: false,
            mode,
            mode_extension: crate::header::ModeExtension(0),
            copyright: false,
            original: true,
            emphasis: crate::header::Emphasis::None,
        }
    }

    #[test]
    fn quant_classes_match_b4_formulas() {
        // For nlevels = 2^n - 1 (every row but n=3/5/9 with grouping),
        // the spec's C = 2^n/(2^n-1) and D = 2^(-n+1). Check rows in
        // that family against the formula.
        for class in QUANT_CLASSES.iter() {
            // n is the codeword width for non-grouped classes.
            if !class.grouping {
                let n = class.bits_per_codeword as i32;
                let expected_c = (1u64 << n) as f64 / ((1u64 << n) - 1) as f64;
                let expected_d = 2f64.powi(-(n) + 1);
                assert!(
                    (class.c - expected_c).abs() / expected_c < 1e-10,
                    "C mismatch nlevels={} got {} want {}",
                    class.nlevels,
                    class.c,
                    expected_c,
                );
                assert!(
                    (class.d - expected_d).abs() / expected_d < 1e-4,
                    "D mismatch nlevels={} got {} want {}",
                    class.nlevels,
                    class.d,
                    expected_d,
                );
            }
        }
    }

    #[test]
    fn grouped_classes_bits_per_sample() {
        // nlevels 3 -> 2 bits per sample, 5 -> 3, 9 -> 4.
        let c3 = lookup_class(3).unwrap();
        let c5 = lookup_class(5).unwrap();
        let c9 = lookup_class(9).unwrap();
        assert_eq!(c3.bits_per_sample(), 2);
        assert_eq!(c5.bits_per_sample(), 3);
        assert_eq!(c9.bits_per_sample(), 4);
        // Non-grouped: per-sample width == codeword width.
        let c7 = lookup_class(7).unwrap();
        assert_eq!(c7.bits_per_sample(), 3);
    }

    #[test]
    fn b2a_shape() {
        let t = layer2_bit_allocation_table(&header_mp1(
            44_100,
            128,
            crate::header::Mode::SingleChannel,
        ));
        // 128 kbit/s mono at 44.1 kHz -> B.2b (96..192 column).
        assert_eq!(t.sblimit(), 30);

        let t = layer2_bit_allocation_table(&header_mp1(
            44_100,
            64,
            crate::header::Mode::SingleChannel,
        ));
        // 64 kbit/s mono at 44.1 kHz -> B.2a.
        assert_eq!(t.sblimit(), 27);
        assert_eq!(t.nbal(0), 4);
        assert_eq!(t.nbal(22), 3);
        assert_eq!(t.nbal(26), 2);
        assert_eq!(t.nbal(27), 0);
        // sum-of-nbal sanity (B.2a spec footer: 88).
        let total: u32 = (0..t.sblimit()).map(|sb| t.nbal(sb) as u32).sum();
        assert_eq!(total, 88);
    }

    #[test]
    fn b2c_b2d_shapes() {
        let t = layer2_bit_allocation_table(&header_mp1(
            44_100,
            32,
            crate::header::Mode::SingleChannel,
        ));
        assert_eq!(t.sblimit(), 8);
        let total: u32 = (0..t.sblimit()).map(|sb| t.nbal(sb) as u32).sum();
        assert_eq!(total, 26);

        let t = layer2_bit_allocation_table(&header_mp1(
            32_000,
            48,
            crate::header::Mode::SingleChannel,
        ));
        assert_eq!(t.sblimit(), 12);
        let total: u32 = (0..t.sblimit()).map(|sb| t.nbal(sb) as u32).sum();
        assert_eq!(total, 38);
    }

    #[test]
    fn quant_class_resolution_b2a() {
        let t = layer2_bit_allocation_table(&header_mp1(
            44_100,
            64,
            crate::header::Mode::SingleChannel,
        ));
        // sb 0 alloc 1 -> nlevels 3 (first non-empty cell in row 0..2).
        let cls = t.quant_class(0, 1).expect("class");
        assert_eq!(cls.nlevels, 3);
        // sb 23 alloc 1 -> nlevels 3 (row 23..26 first cell).
        let cls = t.quant_class(23, 1).unwrap();
        assert_eq!(cls.nlevels, 3);
        // sb 23 alloc 3 -> 65535 (the last allowed allocation = 0b11).
        let cls = t.quant_class(23, 3).unwrap();
        assert_eq!(cls.nlevels, 65535);
        // sb 23 alloc 0 -> None (no samples; allocation 0 is not in the table).
        assert!(t.quant_class(23, 0).is_none());
    }

    #[test]
    fn stereo_total_bitrate_per_channel_division() {
        // 192 kbit/s stereo at 44.1 kHz -> per-channel 96 -> B.2b.
        let t = layer2_bit_allocation_table(&header_mp1(44_100, 192, crate::header::Mode::Stereo));
        assert_eq!(t.sblimit(), 30);
        // 64 kbit/s stereo at 44.1 kHz -> per-channel 32 -> B.2c.
        let t = layer2_bit_allocation_table(&header_mp1(44_100, 64, crate::header::Mode::Stereo));
        assert_eq!(t.sblimit(), 8);
    }
}
