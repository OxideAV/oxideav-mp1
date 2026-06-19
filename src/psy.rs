//! Annex D — Psychoacoustic Model 1 building blocks
//! ------------------------------------------------
//!
//! This module stages the text-extractable portions of **ISO/IEC
//! 11172-3:1993 Annex D** (informative, encoder psychoacoustic models).
//! The decoder never consults these tables — they only feed the encoder
//! when a perceptually-driven bit allocator is plugged in.
//!
//! Reading list (clean-room provenance):
//!
//! * [`docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md`](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md)
//!   — verbatim transcription of the §D.1 prose, **Tables D.2a-f**
//!   (critical-band boundaries — fully text-extracted from 200-DPI page
//!   renders), the **Step 6 av / vf masking coefficients** (text-layer
//!   readable), the **Step 7 global-threshold formula**, and **Table
//!   D.5** (Layer I / Layer II coder partition table — text-extracted).
//! * The docs collaborator's Annex-D extraction round (docs `#129`)
//!   completed the previously PNG-only dense pages as text/CSV under
//!   `docs/audio/mp3/annex-d-table-D{1,3,4}{a..f}-*.csv`. The
//!   text-anchored tables land here as complete consts
//!   ([`CALC_PARTITION_32K`] — the 49-row D.3a table;
//!   [`LTQ_L1_32K`] — the full 108-row D.1a threshold-in-quiet
//!   table; [`LTQ_L1_44K1`] — the 106-row D.1b threshold-in-quiet
//!   table for Layer I at 44,1 kHz). The remaining siblings
//!   (D.1c (48 kHz), D.1d–f (Layer II), D.3b–c, D.4a–c) are staged
//!   as CSVs but not yet transcribed into Rust consts.
//!
//! ### What this module provides
//!
//! 1. [`CriticalBand`] — typed row of Table D.2x.
//! 2. [`critical_band_table`] — runtime lookup of D.2a / D.2b / …
//!    D.2f keyed by `(Layer, sampling_frequency_hz)`.
//! 3. [`masking_index_tonal`] / [`masking_index_non_tonal`] — closed-form
//!    `av_tm` / `av_nm` (clause D.1 Step 6).
//! 4. [`masking_function`] — closed-form `vf(dz, X)` with the
//!    four-piece spreading-function definition.
//! 5. [`CODER_PARTITIONS`] — Table D.5 (33 rows × `(boundary, width)`).
//! 6. [`model2_spreading_matrix_32k`] / [`model2_spread_weight_32k`] /
//!    [`model2_spread_normalization_32k`] — the clause D.2.3 Model 2
//!    partition-domain spreading operator composed over the complete
//!    Table D.3a (`bval` column), at Fs = 32 kHz only (D.3b / D.3c
//!    stay DOCS-GAP).
//!
//! No allocator wiring is changed in this round; the existing
//! [`crate::encode::allocate_bits`] path still drives subband bits
//! from signal energy. The 32 / 44,1 kHz Layer I threshold-in-quiet
//! tables ([`LTQ_L1_32K`] / [`LTQ_L1_44K1`]) and Model 2 partition
//! table ([`CALC_PARTITION_32K`]) are the building blocks an Annex D
//! allocator would consume; the 48 kHz LTq, Layer II LTq, and D.4
//! per-line tables remain to be transcribed before the full model is
//! wired.

use crate::header::Layer;

/// One row of Annex D Table D.2x ("Critical band boundaries").
///
/// Tables D.2a / D.2b / D.2c list the band layouts for Layer I at
/// 32 / 44,1 / 48 kHz (24 / 25 / 26 bands respectively). Tables D.2d /
/// D.2e / D.2f list the Layer II layouts (25 / 27 / 27 bands). The
/// `index_fcb` column points at a row of the corresponding Table
/// D.1x ("Frequencies, critical band rates and absolute threshold")
/// and is 1-based per the printed spec; this struct preserves that
/// 1-based numbering verbatim. `top_freq_hz` is the **upper** edge of
/// the critical band; `bark_z` is the Bark (Zwicker) value at that
/// edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CriticalBand {
    /// Index *i* into the matching Table D.1x (1-based, per the spec).
    pub index_fcb: u16,
    /// Upper-edge frequency of the band, in Hz.
    pub top_freq_hz: f64,
    /// Bark value `z` at the upper edge.
    pub bark_z: f64,
}

// -----------------------------------------------------------------
// Table D.2a — Layer I, Fs = 32 kHz (24 bands, no 0..23)
// docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md
// -----------------------------------------------------------------
const D2A_L1_32K: &[CriticalBand] = &[
    CriticalBand {
        index_fcb: 1,
        top_freq_hz: 62.500,
        bark_z: 0.617,
    },
    CriticalBand {
        index_fcb: 3,
        top_freq_hz: 187.500,
        bark_z: 1.842,
    },
    CriticalBand {
        index_fcb: 5,
        top_freq_hz: 312.500,
        bark_z: 3.037,
    },
    CriticalBand {
        index_fcb: 7,
        top_freq_hz: 437.500,
        bark_z: 4.185,
    },
    CriticalBand {
        index_fcb: 9,
        top_freq_hz: 562.500,
        bark_z: 5.272,
    },
    CriticalBand {
        index_fcb: 11,
        top_freq_hz: 687.500,
        bark_z: 6.289,
    },
    CriticalBand {
        index_fcb: 13,
        top_freq_hz: 812.500,
        bark_z: 7.233,
    },
    CriticalBand {
        index_fcb: 15,
        top_freq_hz: 937.500,
        bark_z: 8.103,
    },
    CriticalBand {
        index_fcb: 18,
        top_freq_hz: 1125.000,
        bark_z: 9.275,
    },
    CriticalBand {
        index_fcb: 21,
        top_freq_hz: 1312.500,
        bark_z: 10.301,
    },
    CriticalBand {
        index_fcb: 24,
        top_freq_hz: 1500.000,
        bark_z: 11.199,
    },
    CriticalBand {
        index_fcb: 27,
        top_freq_hz: 1687.500,
        bark_z: 11.988,
    },
    CriticalBand {
        index_fcb: 32,
        top_freq_hz: 2000.000,
        bark_z: 13.104,
    },
    CriticalBand {
        index_fcb: 37,
        top_freq_hz: 2312.500,
        bark_z: 14.027,
    },
    CriticalBand {
        index_fcb: 44,
        top_freq_hz: 2750.000,
        bark_z: 15.087,
    },
    CriticalBand {
        index_fcb: 50,
        top_freq_hz: 3250.000,
        bark_z: 16.069,
    },
    CriticalBand {
        index_fcb: 55,
        top_freq_hz: 3875.000,
        bark_z: 17.078,
    },
    CriticalBand {
        index_fcb: 61,
        top_freq_hz: 4625.000,
        bark_z: 18.089,
    },
    CriticalBand {
        index_fcb: 68,
        top_freq_hz: 5500.000,
        bark_z: 19.095,
    },
    CriticalBand {
        index_fcb: 74,
        top_freq_hz: 6500.000,
        bark_z: 20.079,
    },
    CriticalBand {
        index_fcb: 79,
        top_freq_hz: 7750.000,
        bark_z: 21.098,
    },
    CriticalBand {
        index_fcb: 85,
        top_freq_hz: 9250.000,
        bark_z: 22.046,
    },
    CriticalBand {
        index_fcb: 94,
        top_freq_hz: 11500.000,
        bark_z: 23.030,
    },
    CriticalBand {
        index_fcb: 108,
        top_freq_hz: 15000.000,
        bark_z: 23.923,
    },
];

// -----------------------------------------------------------------
// Table D.2b — Layer I, Fs = 44,1 kHz (25 bands, no 0..24)
// -----------------------------------------------------------------
const D2B_L1_44K1: &[CriticalBand] = &[
    CriticalBand {
        index_fcb: 1,
        top_freq_hz: 86.133,
        bark_z: 0.850,
    },
    CriticalBand {
        index_fcb: 2,
        top_freq_hz: 172.266,
        bark_z: 1.694,
    },
    CriticalBand {
        index_fcb: 3,
        top_freq_hz: 258.398,
        bark_z: 2.525,
    },
    CriticalBand {
        index_fcb: 5,
        top_freq_hz: 430.664,
        bark_z: 4.124,
    },
    CriticalBand {
        index_fcb: 6,
        top_freq_hz: 516.797,
        bark_z: 4.882,
    },
    CriticalBand {
        index_fcb: 8,
        top_freq_hz: 689.063,
        bark_z: 6.301,
    },
    CriticalBand {
        index_fcb: 9,
        top_freq_hz: 775.195,
        bark_z: 6.959,
    },
    CriticalBand {
        index_fcb: 11,
        top_freq_hz: 947.461,
        bark_z: 8.169,
    },
    CriticalBand {
        index_fcb: 13,
        top_freq_hz: 1119.727,
        bark_z: 9.244,
    },
    CriticalBand {
        index_fcb: 15,
        top_freq_hz: 1291.992,
        bark_z: 10.195,
    },
    CriticalBand {
        index_fcb: 17,
        top_freq_hz: 1464.258,
        bark_z: 11.037,
    },
    CriticalBand {
        index_fcb: 20,
        top_freq_hz: 1722.656,
        bark_z: 12.125,
    },
    CriticalBand {
        index_fcb: 23,
        top_freq_hz: 1981.055,
        bark_z: 13.042,
    },
    CriticalBand {
        index_fcb: 27,
        top_freq_hz: 2325.586,
        bark_z: 14.062,
    },
    CriticalBand {
        index_fcb: 32,
        top_freq_hz: 2756.250,
        bark_z: 15.100,
    },
    CriticalBand {
        index_fcb: 37,
        top_freq_hz: 3186.914,
        bark_z: 15.955,
    },
    CriticalBand {
        index_fcb: 45,
        top_freq_hz: 3875.977,
        bark_z: 17.079,
    },
    CriticalBand {
        index_fcb: 50,
        top_freq_hz: 4478.906,
        bark_z: 17.904,
    },
    CriticalBand {
        index_fcb: 55,
        top_freq_hz: 5340.234,
        bark_z: 18.922,
    },
    CriticalBand {
        index_fcb: 61,
        top_freq_hz: 6373.828,
        bark_z: 19.963,
    },
    CriticalBand {
        index_fcb: 68,
        top_freq_hz: 7579.688,
        bark_z: 20.971,
    },
    CriticalBand {
        index_fcb: 75,
        top_freq_hz: 9302.344,
        bark_z: 22.074,
    },
    CriticalBand {
        index_fcb: 81,
        top_freq_hz: 11369.531,
        bark_z: 22.984,
    },
    CriticalBand {
        index_fcb: 93,
        top_freq_hz: 15503.906,
        bark_z: 24.013,
    },
    CriticalBand {
        index_fcb: 106,
        top_freq_hz: 19982.813,
        bark_z: 24.573,
    },
];

// -----------------------------------------------------------------
// Table D.2c — Layer I, Fs = 48 kHz (26 bands, no 0..25)
// -----------------------------------------------------------------
const D2C_L1_48K: &[CriticalBand] = &[
    CriticalBand {
        index_fcb: 1,
        top_freq_hz: 93.750,
        bark_z: 0.925,
    },
    CriticalBand {
        index_fcb: 2,
        top_freq_hz: 187.500,
        bark_z: 1.842,
    },
    CriticalBand {
        index_fcb: 3,
        top_freq_hz: 281.250,
        bark_z: 2.742,
    },
    CriticalBand {
        index_fcb: 4,
        top_freq_hz: 375.000,
        bark_z: 3.618,
    },
    CriticalBand {
        index_fcb: 5,
        top_freq_hz: 468.750,
        bark_z: 4.463,
    },
    CriticalBand {
        index_fcb: 6,
        top_freq_hz: 562.500,
        bark_z: 5.272,
    },
    CriticalBand {
        index_fcb: 7,
        top_freq_hz: 656.250,
        bark_z: 6.041,
    },
    CriticalBand {
        index_fcb: 9,
        top_freq_hz: 843.750,
        bark_z: 7.457,
    },
    CriticalBand {
        index_fcb: 10,
        top_freq_hz: 937.500,
        bark_z: 8.103,
    },
    CriticalBand {
        index_fcb: 12,
        top_freq_hz: 1125.000,
        bark_z: 9.275,
    },
    CriticalBand {
        index_fcb: 14,
        top_freq_hz: 1312.500,
        bark_z: 10.301,
    },
    CriticalBand {
        index_fcb: 16,
        top_freq_hz: 1500.000,
        bark_z: 11.199,
    },
    CriticalBand {
        index_fcb: 19,
        top_freq_hz: 1781.250,
        bark_z: 12.347,
    },
    CriticalBand {
        index_fcb: 21,
        top_freq_hz: 1968.750,
        bark_z: 13.002,
    },
    CriticalBand {
        index_fcb: 25,
        top_freq_hz: 2343.750,
        bark_z: 14.111,
    },
    CriticalBand {
        index_fcb: 29,
        top_freq_hz: 2718.750,
        bark_z: 15.018,
    },
    CriticalBand {
        index_fcb: 35,
        top_freq_hz: 3281.250,
        bark_z: 16.124,
    },
    CriticalBand {
        index_fcb: 41,
        top_freq_hz: 3843.750,
        bark_z: 17.032,
    },
    CriticalBand {
        index_fcb: 49,
        top_freq_hz: 4687.500,
        bark_z: 18.166,
    },
    CriticalBand {
        index_fcb: 53,
        top_freq_hz: 5437.500,
        bark_z: 19.028,
    },
    CriticalBand {
        index_fcb: 58,
        top_freq_hz: 6375.000,
        bark_z: 19.964,
    },
    CriticalBand {
        index_fcb: 65,
        top_freq_hz: 7687.500,
        bark_z: 21.052,
    },
    CriticalBand {
        index_fcb: 73,
        top_freq_hz: 9375.000,
        bark_z: 22.113,
    },
    CriticalBand {
        index_fcb: 79,
        top_freq_hz: 11625.000,
        bark_z: 23.072,
    },
    CriticalBand {
        index_fcb: 89,
        top_freq_hz: 15375.000,
        bark_z: 23.991,
    },
    CriticalBand {
        index_fcb: 102,
        top_freq_hz: 20250.000,
        bark_z: 24.597,
    },
];

// -----------------------------------------------------------------
// Table D.2d — Layer II, Fs = 32 kHz (25 bands, no 0..24)
// -----------------------------------------------------------------
const D2D_L2_32K: &[CriticalBand] = &[
    CriticalBand {
        index_fcb: 1,
        top_freq_hz: 31.250,
        bark_z: 0.309,
    },
    CriticalBand {
        index_fcb: 3,
        top_freq_hz: 93.750,
        bark_z: 0.925,
    },
    CriticalBand {
        index_fcb: 6,
        top_freq_hz: 187.500,
        bark_z: 1.842,
    },
    CriticalBand {
        index_fcb: 10,
        top_freq_hz: 312.500,
        bark_z: 3.037,
    },
    CriticalBand {
        index_fcb: 13,
        top_freq_hz: 406.250,
        bark_z: 3.903,
    },
    CriticalBand {
        index_fcb: 17,
        top_freq_hz: 531.250,
        bark_z: 5.006,
    },
    CriticalBand {
        index_fcb: 21,
        top_freq_hz: 656.250,
        bark_z: 6.041,
    },
    CriticalBand {
        index_fcb: 25,
        top_freq_hz: 781.250,
        bark_z: 7.004,
    },
    CriticalBand {
        index_fcb: 30,
        top_freq_hz: 937.500,
        bark_z: 8.103,
    },
    CriticalBand {
        index_fcb: 35,
        top_freq_hz: 1093.750,
        bark_z: 9.090,
    },
    CriticalBand {
        index_fcb: 41,
        top_freq_hz: 1281.250,
        bark_z: 10.139,
    },
    CriticalBand {
        index_fcb: 47,
        top_freq_hz: 1468.750,
        bark_z: 11.058,
    },
    CriticalBand {
        index_fcb: 51,
        top_freq_hz: 1687.500,
        bark_z: 11.988,
    },
    CriticalBand {
        index_fcb: 56,
        top_freq_hz: 2000.000,
        bark_z: 13.104,
    },
    CriticalBand {
        index_fcb: 61,
        top_freq_hz: 2312.500,
        bark_z: 14.027,
    },
    CriticalBand {
        index_fcb: 68,
        top_freq_hz: 2750.000,
        bark_z: 15.087,
    },
    CriticalBand {
        index_fcb: 74,
        top_freq_hz: 3250.000,
        bark_z: 16.069,
    },
    CriticalBand {
        index_fcb: 79,
        top_freq_hz: 3875.000,
        bark_z: 17.078,
    },
    CriticalBand {
        index_fcb: 85,
        top_freq_hz: 4625.000,
        bark_z: 18.089,
    },
    CriticalBand {
        index_fcb: 92,
        top_freq_hz: 5500.000,
        bark_z: 19.095,
    },
    CriticalBand {
        index_fcb: 98,
        top_freq_hz: 6500.000,
        bark_z: 20.079,
    },
    CriticalBand {
        index_fcb: 103,
        top_freq_hz: 7750.000,
        bark_z: 21.098,
    },
    CriticalBand {
        index_fcb: 109,
        top_freq_hz: 9250.000,
        bark_z: 22.046,
    },
    CriticalBand {
        index_fcb: 118,
        top_freq_hz: 11500.000,
        bark_z: 23.030,
    },
    CriticalBand {
        index_fcb: 132,
        top_freq_hz: 15000.000,
        bark_z: 23.923,
    },
];

// -----------------------------------------------------------------
// Table D.2e — Layer II, Fs = 44,1 kHz (27 bands, no 0..26)
//
// Spec note: row 17 prints `bark_z = 16,11[illegible]` in the staged
// PDF. The companion docs note documents this as `[illegible]` with a
// likely value of 16,116 by inter-row spacing. We carry **16.116**
// here — the closest 3-decimal-place value consistent with the row's
// `index_fcb = 62` and the surrounding band Bark spacings — and flag
// it in the row's surrounding comment. Implementers that need the
// exact spec digit should treat this as a DOCS-GAP until the source
// PDF text is rendered with a higher-DPI / different OCR pass.
// -----------------------------------------------------------------
const D2E_L2_44K1: &[CriticalBand] = &[
    CriticalBand {
        index_fcb: 1,
        top_freq_hz: 43.066,
        bark_z: 0.425,
    },
    CriticalBand {
        index_fcb: 2,
        top_freq_hz: 86.133,
        bark_z: 0.850,
    },
    CriticalBand {
        index_fcb: 3,
        top_freq_hz: 129.199,
        bark_z: 1.273,
    },
    CriticalBand {
        index_fcb: 5,
        top_freq_hz: 215.332,
        bark_z: 2.112,
    },
    CriticalBand {
        index_fcb: 7,
        top_freq_hz: 301.465,
        bark_z: 2.934,
    },
    CriticalBand {
        index_fcb: 10,
        top_freq_hz: 430.664,
        bark_z: 4.124,
    },
    CriticalBand {
        index_fcb: 13,
        top_freq_hz: 559.863,
        bark_z: 5.249,
    },
    CriticalBand {
        index_fcb: 16,
        top_freq_hz: 689.063,
        bark_z: 6.301,
    },
    CriticalBand {
        index_fcb: 19,
        top_freq_hz: 818.262,
        bark_z: 7.274,
    },
    CriticalBand {
        index_fcb: 22,
        top_freq_hz: 947.461,
        bark_z: 8.169,
    },
    CriticalBand {
        index_fcb: 26,
        top_freq_hz: 1119.727,
        bark_z: 9.244,
    },
    CriticalBand {
        index_fcb: 30,
        top_freq_hz: 1291.992,
        bark_z: 10.195,
    },
    CriticalBand {
        index_fcb: 35,
        top_freq_hz: 1507.324,
        bark_z: 11.232,
    },
    CriticalBand {
        index_fcb: 40,
        top_freq_hz: 1722.656,
        bark_z: 12.125,
    },
    CriticalBand {
        index_fcb: 46,
        top_freq_hz: 1981.055,
        bark_z: 13.042,
    },
    CriticalBand {
        index_fcb: 51,
        top_freq_hz: 2325.586,
        bark_z: 14.062,
    },
    CriticalBand {
        index_fcb: 56,
        top_freq_hz: 2756.250,
        bark_z: 15.100,
    },
    // Row 17: bark_z prints as `16,11[illegible]` in the staged PDF;
    // best-fit value `16.116` per the docs note (DOCS-GAP for the
    // exact final digit).
    CriticalBand {
        index_fcb: 62,
        top_freq_hz: 3273.047,
        bark_z: 16.116,
    },
    CriticalBand {
        index_fcb: 69,
        top_freq_hz: 3875.977,
        bark_z: 17.079,
    },
    CriticalBand {
        index_fcb: 74,
        top_freq_hz: 4478.906,
        bark_z: 17.904,
    },
    CriticalBand {
        index_fcb: 79,
        top_freq_hz: 5340.234,
        bark_z: 18.922,
    },
    CriticalBand {
        index_fcb: 85,
        top_freq_hz: 6373.828,
        bark_z: 19.963,
    },
    CriticalBand {
        index_fcb: 92,
        top_freq_hz: 7579.688,
        bark_z: 20.971,
    },
    CriticalBand {
        index_fcb: 99,
        top_freq_hz: 9302.344,
        bark_z: 22.074,
    },
    CriticalBand {
        index_fcb: 105,
        top_freq_hz: 11369.531,
        bark_z: 22.984,
    },
    CriticalBand {
        index_fcb: 117,
        top_freq_hz: 15503.906,
        bark_z: 24.013,
    },
    CriticalBand {
        index_fcb: 130,
        top_freq_hz: 19982.813,
        bark_z: 24.573,
    },
];

// -----------------------------------------------------------------
// Table D.2f — Layer II, Fs = 48 kHz (27 bands, no 0..26)
// -----------------------------------------------------------------
const D2F_L2_48K: &[CriticalBand] = &[
    CriticalBand {
        index_fcb: 1,
        top_freq_hz: 46.875,
        bark_z: 0.463,
    },
    CriticalBand {
        index_fcb: 2,
        top_freq_hz: 93.750,
        bark_z: 0.925,
    },
    CriticalBand {
        index_fcb: 3,
        top_freq_hz: 140.625,
        bark_z: 1.385,
    },
    CriticalBand {
        index_fcb: 5,
        top_freq_hz: 234.375,
        bark_z: 2.295,
    },
    CriticalBand {
        index_fcb: 7,
        top_freq_hz: 328.125,
        bark_z: 3.184,
    },
    CriticalBand {
        index_fcb: 9,
        top_freq_hz: 421.875,
        bark_z: 4.045,
    },
    CriticalBand {
        index_fcb: 12,
        top_freq_hz: 562.500,
        bark_z: 5.272,
    },
    CriticalBand {
        index_fcb: 14,
        top_freq_hz: 656.250,
        bark_z: 6.041,
    },
    CriticalBand {
        index_fcb: 17,
        top_freq_hz: 796.875,
        bark_z: 7.119,
    },
    CriticalBand {
        index_fcb: 20,
        top_freq_hz: 937.500,
        bark_z: 8.103,
    },
    CriticalBand {
        index_fcb: 24,
        top_freq_hz: 1125.000,
        bark_z: 9.275,
    },
    CriticalBand {
        index_fcb: 27,
        top_freq_hz: 1265.625,
        bark_z: 10.057,
    },
    CriticalBand {
        index_fcb: 32,
        top_freq_hz: 1500.000,
        bark_z: 11.199,
    },
    CriticalBand {
        index_fcb: 37,
        top_freq_hz: 1734.375,
        bark_z: 12.170,
    },
    CriticalBand {
        index_fcb: 42,
        top_freq_hz: 1968.750,
        bark_z: 13.002,
    },
    CriticalBand {
        index_fcb: 49,
        top_freq_hz: 2343.750,
        bark_z: 14.111,
    },
    CriticalBand {
        index_fcb: 53,
        top_freq_hz: 2718.750,
        bark_z: 15.018,
    },
    CriticalBand {
        index_fcb: 59,
        top_freq_hz: 3281.250,
        bark_z: 16.124,
    },
    CriticalBand {
        index_fcb: 65,
        top_freq_hz: 3843.750,
        bark_z: 17.032,
    },
    CriticalBand {
        index_fcb: 73,
        top_freq_hz: 4687.500,
        bark_z: 18.166,
    },
    CriticalBand {
        index_fcb: 77,
        top_freq_hz: 5437.500,
        bark_z: 19.028,
    },
    CriticalBand {
        index_fcb: 82,
        top_freq_hz: 6375.000,
        bark_z: 19.964,
    },
    CriticalBand {
        index_fcb: 89,
        top_freq_hz: 7687.500,
        bark_z: 21.052,
    },
    CriticalBand {
        index_fcb: 97,
        top_freq_hz: 9375.000,
        bark_z: 22.113,
    },
    CriticalBand {
        index_fcb: 103,
        top_freq_hz: 11625.000,
        bark_z: 23.072,
    },
    CriticalBand {
        index_fcb: 113,
        top_freq_hz: 15375.000,
        bark_z: 23.991,
    },
    CriticalBand {
        index_fcb: 126,
        top_freq_hz: 20250.000,
        bark_z: 24.597,
    },
];

/// Look up the §D.1 critical-band-boundary table for the given Layer
/// and sampling frequency. Returns `None` for sampling frequencies
/// outside the {32 000, 44 100, 48 000} Hz set covered by Annex D
/// (i.e. for the MPEG-2 LSF rates 16 / 22.05 / 24 kHz — Annex D is
/// MPEG-1-only and the §D.1 / §D.2 prose makes no LSF reference).
/// The crate's `Layer` enum is `{I, II}` only; Layer III has its own
/// §C.1.5.3 model that is not part of this MPEG-1 Layer I/II crate.
pub fn critical_band_table(
    layer: Layer,
    sampling_frequency_hz: u32,
) -> Option<&'static [CriticalBand]> {
    match (layer, sampling_frequency_hz) {
        (Layer::I, 32_000) => Some(D2A_L1_32K),
        (Layer::I, 44_100) => Some(D2B_L1_44K1),
        (Layer::I, 48_000) => Some(D2C_L1_48K),
        (Layer::II, 32_000) => Some(D2D_L2_32K),
        (Layer::II, 44_100) => Some(D2E_L2_44K1),
        (Layer::II, 48_000) => Some(D2F_L2_48K),
        _ => None,
    }
}

// -----------------------------------------------------------------
// Annex D Step 6 — Masking-index `av` (closed-form)
//
// Quoted verbatim from the staged docs extract (text-extractable PDF
// region):
//
//   tonal     : av_tm = -1,525 - 0,275 * z(j) - 4,5   dB
//   non-tonal : av_nm = -1,525 - 0,175 * z(j) - 0,5   dB
//
// (`z(j)` is the Bark value of the masker j.)
// -----------------------------------------------------------------

/// Annex D Step 6 — tonal-masker index `av_tm(z)` in dB.
///
/// Returns `-1.525 - 0.275 · z - 4.5`.
#[inline]
pub fn masking_index_tonal(z: f64) -> f64 {
    -1.525 - 0.275 * z - 4.5
}

/// Annex D Step 6 — non-tonal-masker index `av_nm(z)` in dB.
///
/// Returns `-1.525 - 0.175 · z - 0.5`.
#[inline]
pub fn masking_index_non_tonal(z: f64) -> f64 {
    -1.525 - 0.175 * z - 0.5
}

// -----------------------------------------------------------------
// Annex D Step 6 — Masking-function `vf(dz, X)` (closed-form)
//
// Quoted verbatim from the staged docs extract:
//
//   vf = 17 * (dz + 1) - (0,4 * X[z(j)] + 6)   dB  for -3 <= dz < -1 Bark
//   vf = (0,4 * X[z(j)] + 6) * dz              dB  for -1 <= dz <  0 Bark
//   vf = -17 * dz                              dB  for  0 <= dz <  1 Bark
//   vf = -(dz - 1) * (17 - 0,15 * X[z(j)]) - 17 dB for  1 <= dz <  8 Bark
//
// Outside -3 <= dz < 8 the masker is ignored (LT set to -inf dB);
// we return `None` to model that. Inside the range we return a
// concrete dB value.
//
// `dz = z(i) - z(j)` is the Bark distance from masker j to spectral
// line i; `x_db` is the SPL of the masker (dB).
// -----------------------------------------------------------------

/// Annex D Step 6 — spreading-function piece selector. Exposed for
/// tests so each branch can be exercised independently.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaskingBranch {
    /// `-3 <= dz < -1`: rising-edge low-side branch.
    LowFar,
    /// `-1 <= dz < 0`: rising-edge near-side branch.
    LowNear,
    /// `0 <= dz < 1`: high-side near branch.
    HighNear,
    /// `1 <= dz < 8`: high-side far (level-dependent) branch.
    HighFar,
}

/// Annex D Step 6 — pick the spreading-function branch for `dz`, or
/// `None` if the masker is outside the `-3 <= dz < 8` Bark window
/// (i.e. ignored).
#[inline]
pub fn masking_branch(dz: f64) -> Option<MaskingBranch> {
    if !(-3.0..8.0).contains(&dz) {
        None
    } else if dz < -1.0 {
        Some(MaskingBranch::LowFar)
    } else if dz < 0.0 {
        Some(MaskingBranch::LowNear)
    } else if dz < 1.0 {
        Some(MaskingBranch::HighNear)
    } else {
        Some(MaskingBranch::HighFar)
    }
}

/// Annex D Step 6 — masking function `vf(dz, X)` in dB.
///
/// `dz = z(i) - z(j)` is the Bark distance from masker j to spectral
/// line i; `x_db` is the SPL of the masker (dB). Returns `None` when
/// the masker is ignored (outside `-3 <= dz < 8` Bark).
pub fn masking_function(dz: f64, x_db: f64) -> Option<f64> {
    let branch = masking_branch(dz)?;
    let v = match branch {
        MaskingBranch::LowFar => 17.0 * (dz + 1.0) - (0.4 * x_db + 6.0),
        MaskingBranch::LowNear => (0.4 * x_db + 6.0) * dz,
        MaskingBranch::HighNear => -17.0 * dz,
        MaskingBranch::HighFar => -(dz - 1.0) * (17.0 - 0.15 * x_db) - 17.0,
    };
    Some(v)
}

/// Annex D Step 6 — composite individual masking threshold
/// `LT_tm[z(j), z(i)]` of a **tonal** masker at Bark `z_j` with SPL
/// `x_db`, evaluated at spectral-line Bark `z_i`. Returns `None` when
/// the masker is outside the Bark window.
///
/// `LT_tm = X + av_tm(z_j) + vf(z_i - z_j, X)`.
pub fn individual_threshold_tonal(z_j: f64, z_i: f64, x_db: f64) -> Option<f64> {
    let vf = masking_function(z_i - z_j, x_db)?;
    Some(x_db + masking_index_tonal(z_j) + vf)
}

/// Annex D Step 6 — composite individual masking threshold
/// `LT_nm[z(j), z(i)]` of a **non-tonal** masker. Symmetric to
/// [`individual_threshold_tonal`].
pub fn individual_threshold_non_tonal(z_j: f64, z_i: f64, x_db: f64) -> Option<f64> {
    let vf = masking_function(z_i - z_j, x_db)?;
    Some(x_db + masking_index_non_tonal(z_j) + vf)
}

/// Annex D Step 7 — combine `LTq(i)` with a set of tonal and
/// non-tonal individual thresholds into the global masking threshold
/// `LTg(i)` (dB). Inputs are dB; the function performs the
/// power-domain sum the spec prints verbatim:
///
/// `LTg = 10·log10( 10^(LTq/10) + Σ 10^(LT_tm/10) + Σ 10^(LT_nm/10) )`
///
/// Maskers that should be "ignored" (dz outside the Bark window) must
/// be omitted from the slices before calling this — the function does
/// no filtering of its own.
pub fn global_threshold_db(ltq_db: f64, tonal: &[f64], non_tonal: &[f64]) -> f64 {
    let mut acc = 10f64.powf(ltq_db / 10.0);
    for &lt in tonal {
        acc += 10f64.powf(lt / 10.0);
    }
    for &lt in non_tonal {
        acc += 10f64.powf(lt / 10.0);
    }
    10.0 * acc.log10()
}

// -----------------------------------------------------------------
// Table D.5 — Layer I / Layer II coder partition table
//
// Text-extracted verbatim from the staged docs note (33 rows, the
// per-row `ωlow_{n+1} / ωhigh_n` boundary FFT line + the partition
// width). Same partition layout is shared by Layer I and Layer II.
// -----------------------------------------------------------------

/// One row of Table D.5. `boundary` is the partition-edge FFT line
/// number (1-based, per the printed table — `ωlow_{n+1} / ωhigh_n`);
/// `width` is the partition width column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoderPartition {
    /// Partition-boundary FFT line number (1-based).
    pub boundary: u16,
    /// Partition width column (0 or 1 per the printed spec).
    pub width: u8,
}

/// Annex D Table D.5 — Layer I and Layer II coder partition table.
///
/// 33 rows (n = 0..32). Partitions 0..12 use width 0; partitions
/// 13..32 use width 1.
pub const CODER_PARTITIONS: [CoderPartition; 33] = [
    CoderPartition {
        boundary: 1,
        width: 0,
    },
    CoderPartition {
        boundary: 17,
        width: 0,
    },
    CoderPartition {
        boundary: 33,
        width: 0,
    },
    CoderPartition {
        boundary: 49,
        width: 0,
    },
    CoderPartition {
        boundary: 65,
        width: 0,
    },
    CoderPartition {
        boundary: 81,
        width: 0,
    },
    CoderPartition {
        boundary: 97,
        width: 0,
    },
    CoderPartition {
        boundary: 113,
        width: 0,
    },
    CoderPartition {
        boundary: 129,
        width: 0,
    },
    CoderPartition {
        boundary: 145,
        width: 0,
    },
    CoderPartition {
        boundary: 161,
        width: 0,
    },
    CoderPartition {
        boundary: 177,
        width: 0,
    },
    CoderPartition {
        boundary: 193,
        width: 0,
    },
    CoderPartition {
        boundary: 209,
        width: 1,
    },
    CoderPartition {
        boundary: 225,
        width: 1,
    },
    CoderPartition {
        boundary: 241,
        width: 1,
    },
    CoderPartition {
        boundary: 257,
        width: 1,
    },
    CoderPartition {
        boundary: 273,
        width: 1,
    },
    CoderPartition {
        boundary: 289,
        width: 1,
    },
    CoderPartition {
        boundary: 305,
        width: 1,
    },
    CoderPartition {
        boundary: 321,
        width: 1,
    },
    CoderPartition {
        boundary: 337,
        width: 1,
    },
    CoderPartition {
        boundary: 353,
        width: 1,
    },
    CoderPartition {
        boundary: 369,
        width: 1,
    },
    CoderPartition {
        boundary: 385,
        width: 1,
    },
    CoderPartition {
        boundary: 401,
        width: 1,
    },
    CoderPartition {
        boundary: 417,
        width: 1,
    },
    CoderPartition {
        boundary: 433,
        width: 1,
    },
    CoderPartition {
        boundary: 449,
        width: 1,
    },
    CoderPartition {
        boundary: 465,
        width: 1,
    },
    CoderPartition {
        boundary: 481,
        width: 1,
    },
    CoderPartition {
        boundary: 497,
        width: 1,
    },
    CoderPartition {
        boundary: 513,
        width: 1,
    },
];

// -----------------------------------------------------------------
// Annex D Table D.3a — Psychoacoustic Model 2 calculation-partition
// table at Fs = 32 kHz (partial anchor: partitions 1..=20 of 63).
//
// Spec context: clause D.2 (Psychoacoustic Model 2). Each row of
// Table D.3x describes one Model-2 threshold-calculation partition
// at the given sampling frequency and carries five columns: the
// 1-based partition index `n`, the inclusive FFT-line span
// `[ωlow_n, ωhigh_n]` (also 1-based), the median Bark value `bval`
// of the partition, the minimum masking-spread floor `minval` (dB),
// and the tone-masking-noise offset `TMN` (dB).
//
// The staged docs extract
// (`docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md`,
// "Table D.3a–c — Model 2 calculation partition table") now
// transcribes the **complete** 49-partition 32 kHz row (D.3a) as
// text, cross-checked against the authoritative PNG render under
// `docs/audio/mp3/annex-d-renders/`. The docs collaborator's
// `docs` #129 correction fixed the earlier "63 partitions"
// miscount to the printed 49 (`bmax` per spec page 130 is 49 at
// 32 kHz, 57 at 44,1 kHz, 58 at 48 kHz). The D.3b (44,1 kHz) /
// D.3c (48 kHz) tables remain staged as renders.
//
// This transcription parallels the prior `CODER_PARTITIONS`
// (Table D.5) staging: the table lands as a typed `const` with
// explicit 1-based row numbering. The Annex D allocator wiring
// proper still waits on D.3b/c, the D.1a LTq body, and D.4a–c,
// per the README spec gap list.
// -----------------------------------------------------------------

/// One row of Annex D Table D.3x ("Psychoacoustic Model 2
/// calculation partition table").
///
/// Carries the five spec columns verbatim: 1-based partition index
/// `n` (`index`), the inclusive FFT-line span `[omega_low,
/// omega_high]` (also 1-based), the median Bark value `bval`, the
/// minimum masking-spread floor `minval` in dB, and the
/// tone-masking-noise offset `tmn` in dB.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalcPartition {
    /// Partition number `n` (1-based, per the printed spec).
    pub index: u16,
    /// Lower FFT-line bound `ωlow_n` (1-based, inclusive).
    pub omega_low: u16,
    /// Upper FFT-line bound `ωhigh_n` (1-based, inclusive).
    pub omega_high: u16,
    /// Median Bark value of the partition.
    pub bval: f64,
    /// Minimum masking-spread floor (dB).
    pub minval: f64,
    /// Tone-masking-noise offset (dB).
    pub tmn: f64,
}

impl CalcPartition {
    /// Width of the partition in FFT lines, `ωhigh − ωlow + 1`.
    ///
    /// Convenience accessor for the (otherwise implicit) per-row
    /// count of FFT lines the partition spans.
    pub fn width(self) -> u16 {
        self.omega_high - self.omega_low + 1
    }
}

/// Annex D Table D.3a — **complete** calculation-partition table at
/// **Fs = 32 kHz**, all **49** partitions printed in ISO/IEC
/// 11172-3 (1993) PDF page 139 (printed 133).
///
/// The full table is now text-transcribed in the staged docs
/// extract (`docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md`,
/// "Table D.3a–c") cross-checked against the authoritative render
/// `docs/audio/mp3/annex-d-renders/Table-D.3a-calc-partition-32kHz-p133.png`.
/// The docs collaborator's correction (`docs` #129) fixed the
/// earlier "63 partitions" miscount to the printed **49** — the
/// last partition `[497, 513]` reaches FFT line 513, the Nyquist
/// line of the 1024-point Model 2 analysis FFT, with `bval`
/// rising to 24,07 Bark, `minval` settled at 4,5 dB and `TMN`
/// climbing to 38,6 dB. The 44,1 kHz (`bmax = 57`) and 48 kHz
/// (`bmax = 58`) tables D.3b / D.3c remain staged as PNG renders.
pub const CALC_PARTITION_32K: [CalcPartition; 49] = [
    CalcPartition {
        index: 1,
        omega_low: 1,
        omega_high: 1,
        bval: 0.00,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 2,
        omega_low: 2,
        omega_high: 4,
        bval: 0.63,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 3,
        omega_low: 5,
        omega_high: 7,
        bval: 1.56,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 4,
        omega_low: 8,
        omega_high: 10,
        bval: 2.50,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 5,
        omega_low: 11,
        omega_high: 13,
        bval: 3.44,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 6,
        omega_low: 14,
        omega_high: 16,
        bval: 4.34,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 7,
        omega_low: 17,
        omega_high: 19,
        bval: 5.17,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 8,
        omega_low: 20,
        omega_high: 22,
        bval: 5.94,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 9,
        omega_low: 23,
        omega_high: 25,
        bval: 6.63,
        minval: 17.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 10,
        omega_low: 26,
        omega_high: 28,
        bval: 7.28,
        minval: 15.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 11,
        omega_low: 29,
        omega_high: 31,
        bval: 7.90,
        minval: 15.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 12,
        omega_low: 32,
        omega_high: 34,
        bval: 8.50,
        minval: 10.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 13,
        omega_low: 35,
        omega_high: 37,
        bval: 9.06,
        minval: 7.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 14,
        omega_low: 38,
        omega_high: 41,
        bval: 9.65,
        minval: 7.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 15,
        omega_low: 42,
        omega_high: 45,
        bval: 10.28,
        minval: 4.4,
        tmn: 24.8,
    },
    CalcPartition {
        index: 16,
        omega_low: 46,
        omega_high: 49,
        bval: 10.87,
        minval: 4.4,
        tmn: 25.4,
    },
    CalcPartition {
        index: 17,
        omega_low: 50,
        omega_high: 53,
        bval: 11.41,
        minval: 4.5,
        tmn: 25.9,
    },
    CalcPartition {
        index: 18,
        omega_low: 54,
        omega_high: 57,
        bval: 11.92,
        minval: 4.5,
        tmn: 26.4,
    },
    CalcPartition {
        index: 19,
        omega_low: 58,
        omega_high: 61,
        bval: 12.39,
        minval: 4.5,
        tmn: 26.9,
    },
    CalcPartition {
        index: 20,
        omega_low: 62,
        omega_high: 65,
        bval: 12.83,
        minval: 4.5,
        tmn: 27.3,
    },
    CalcPartition {
        index: 21,
        omega_low: 66,
        omega_high: 70,
        bval: 13.29,
        minval: 4.5,
        tmn: 27.8,
    },
    CalcPartition {
        index: 22,
        omega_low: 71,
        omega_high: 75,
        bval: 13.78,
        minval: 4.5,
        tmn: 28.3,
    },
    CalcPartition {
        index: 23,
        omega_low: 76,
        omega_high: 81,
        bval: 14.27,
        minval: 4.5,
        tmn: 28.8,
    },
    CalcPartition {
        index: 24,
        omega_low: 82,
        omega_high: 87,
        bval: 14.76,
        minval: 4.5,
        tmn: 29.3,
    },
    CalcPartition {
        index: 25,
        omega_low: 88,
        omega_high: 93,
        bval: 15.22,
        minval: 4.5,
        tmn: 29.7,
    },
    CalcPartition {
        index: 26,
        omega_low: 94,
        omega_high: 99,
        bval: 15.63,
        minval: 4.5,
        tmn: 30.1,
    },
    CalcPartition {
        index: 27,
        omega_low: 100,
        omega_high: 106,
        bval: 16.06,
        minval: 4.5,
        tmn: 30.6,
    },
    CalcPartition {
        index: 28,
        omega_low: 107,
        omega_high: 113,
        bval: 16.47,
        minval: 4.5,
        tmn: 31.0,
    },
    CalcPartition {
        index: 29,
        omega_low: 114,
        omega_high: 120,
        bval: 16.86,
        minval: 4.5,
        tmn: 31.4,
    },
    CalcPartition {
        index: 30,
        omega_low: 121,
        omega_high: 129,
        bval: 17.25,
        minval: 4.5,
        tmn: 31.8,
    },
    CalcPartition {
        index: 31,
        omega_low: 130,
        omega_high: 138,
        bval: 17.65,
        minval: 4.5,
        tmn: 32.2,
    },
    CalcPartition {
        index: 32,
        omega_low: 139,
        omega_high: 148,
        bval: 18.05,
        minval: 4.5,
        tmn: 32.5,
    },
    CalcPartition {
        index: 33,
        omega_low: 149,
        omega_high: 159,
        bval: 18.42,
        minval: 4.5,
        tmn: 32.9,
    },
    CalcPartition {
        index: 34,
        omega_low: 160,
        omega_high: 170,
        bval: 18.81,
        minval: 4.5,
        tmn: 33.3,
    },
    CalcPartition {
        index: 35,
        omega_low: 171,
        omega_high: 183,
        bval: 19.18,
        minval: 4.5,
        tmn: 33.7,
    },
    CalcPartition {
        index: 36,
        omega_low: 184,
        omega_high: 196,
        bval: 19.55,
        minval: 4.5,
        tmn: 34.1,
    },
    CalcPartition {
        index: 37,
        omega_low: 197,
        omega_high: 210,
        bval: 19.93,
        minval: 4.5,
        tmn: 34.4,
    },
    CalcPartition {
        index: 38,
        omega_low: 211,
        omega_high: 225,
        bval: 20.29,
        minval: 4.5,
        tmn: 34.8,
    },
    CalcPartition {
        index: 39,
        omega_low: 226,
        omega_high: 240,
        bval: 20.65,
        minval: 4.5,
        tmn: 35.2,
    },
    CalcPartition {
        index: 40,
        omega_low: 241,
        omega_high: 258,
        bval: 21.02,
        minval: 4.5,
        tmn: 35.5,
    },
    CalcPartition {
        index: 41,
        omega_low: 259,
        omega_high: 279,
        bval: 21.38,
        minval: 4.5,
        tmn: 35.9,
    },
    CalcPartition {
        index: 42,
        omega_low: 280,
        omega_high: 300,
        bval: 21.74,
        minval: 4.5,
        tmn: 36.2,
    },
    CalcPartition {
        index: 43,
        omega_low: 301,
        omega_high: 326,
        bval: 22.10,
        minval: 4.5,
        tmn: 36.6,
    },
    CalcPartition {
        index: 44,
        omega_low: 327,
        omega_high: 354,
        bval: 22.44,
        minval: 4.5,
        tmn: 36.9,
    },
    CalcPartition {
        index: 45,
        omega_low: 355,
        omega_high: 382,
        bval: 22.79,
        minval: 4.5,
        tmn: 37.3,
    },
    CalcPartition {
        index: 46,
        omega_low: 383,
        omega_high: 420,
        bval: 23.14,
        minval: 4.5,
        tmn: 37.6,
    },
    CalcPartition {
        index: 47,
        omega_low: 421,
        omega_high: 458,
        bval: 23.49,
        minval: 4.5,
        tmn: 38.0,
    },
    CalcPartition {
        index: 48,
        omega_low: 459,
        omega_high: 496,
        bval: 23.83,
        minval: 4.5,
        tmn: 38.3,
    },
    CalcPartition {
        index: 49,
        omega_low: 497,
        omega_high: 513,
        bval: 24.07,
        minval: 4.5,
        tmn: 38.6,
    },
];

/// Total number of partitions (`bmax`) in the Annex D Table D.3a
/// (Fs = 32 kHz). Now the printed **49**, per the `docs` #129
/// correction (the earlier 63 was an OCR miscount).
/// [`CALC_PARTITION_32K`] carries all 49 rows.
pub const CALC_PARTITION_32K_FULL_LEN: usize = 49;

/// Look up the calculation-partition row for partition index `n`
/// (1-based, per the printed spec) at Fs = 32 kHz. Returns
/// `Some(row)` for every `n ∈ 1..=49` (the table is now complete)
/// and `None` for the `n == 0` 1-based underflow and `n > 49`
/// out-of-range.
pub fn calc_partition_32k(n: u16) -> Option<CalcPartition> {
    let idx = usize::from(n).checked_sub(1)?;
    CALC_PARTITION_32K.get(idx).copied()
}

/// Annex D Table D.3b — **complete** calculation-partition table at
/// **Fs = 44,1 kHz**, all **57** partitions printed in ISO/IEC
/// 11172-3 (1993) PDF page 140 (printed 134).
///
/// Transcribed from the docs collaborator's text extraction
/// `docs/audio/mp3/annex-d-table-D3b-calc-partition-44k1Hz.csv`,
/// cross-checked against the authoritative render
/// `docs/audio/mp3/annex-d-renders/Table-D.3b-calc-partition-44k1Hz-p134.png`.
/// Each row carries the partition number, the inclusive lower/upper
/// FFT-line bounds (`ωlow`, `ωhigh`), the median Bark value `bval`,
/// the masking-spread floor `minval`, and the tone-masking-noise
/// offset `TMN`. The partitions tile the FFT lines contiguously
/// (`ωlow_{n+1} = ωhigh_n + 1`) up to the Nyquist line 513 of the
/// 1024-point Model 2 analysis FFT. The 44,1 kHz grid packs `bmax = 57`
/// partitions (eight more than the 32 kHz D.3a `bmax = 49`); `bval`
/// rises to 25,33 Bark, and the final partition `[470, 513]` drops its
/// `minval` to 3,5 dB (vs the 4,5 dB the body carries) with `TMN`
/// climbing to 39,8 dB.
pub const CALC_PARTITION_44K1: [CalcPartition; 57] = [
    CalcPartition {
        index: 1,
        omega_low: 1,
        omega_high: 1,
        bval: 0.00,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 2,
        omega_low: 2,
        omega_high: 2,
        bval: 0.43,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 3,
        omega_low: 3,
        omega_high: 3,
        bval: 0.86,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 4,
        omega_low: 4,
        omega_high: 4,
        bval: 1.29,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 5,
        omega_low: 5,
        omega_high: 5,
        bval: 1.72,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 6,
        omega_low: 6,
        omega_high: 6,
        bval: 2.15,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 7,
        omega_low: 7,
        omega_high: 7,
        bval: 2.58,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 8,
        omega_low: 8,
        omega_high: 8,
        bval: 3.01,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 9,
        omega_low: 9,
        omega_high: 9,
        bval: 3.45,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 10,
        omega_low: 10,
        omega_high: 10,
        bval: 3.88,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 11,
        omega_low: 11,
        omega_high: 11,
        bval: 4.28,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 12,
        omega_low: 12,
        omega_high: 12,
        bval: 4.67,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 13,
        omega_low: 13,
        omega_high: 13,
        bval: 5.06,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 14,
        omega_low: 14,
        omega_high: 14,
        bval: 5.42,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 15,
        omega_low: 15,
        omega_high: 15,
        bval: 5.77,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 16,
        omega_low: 16,
        omega_high: 16,
        bval: 6.11,
        minval: 17.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 17,
        omega_low: 17,
        omega_high: 19,
        bval: 6.73,
        minval: 17.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 18,
        omega_low: 20,
        omega_high: 22,
        bval: 7.61,
        minval: 15.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 19,
        omega_low: 23,
        omega_high: 25,
        bval: 8.44,
        minval: 10.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 20,
        omega_low: 26,
        omega_high: 28,
        bval: 9.21,
        minval: 7.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 21,
        omega_low: 29,
        omega_high: 31,
        bval: 9.88,
        minval: 7.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 22,
        omega_low: 32,
        omega_high: 34,
        bval: 10.51,
        minval: 4.4,
        tmn: 25.0,
    },
    CalcPartition {
        index: 23,
        omega_low: 35,
        omega_high: 37,
        bval: 11.11,
        minval: 4.5,
        tmn: 25.6,
    },
    CalcPartition {
        index: 24,
        omega_low: 38,
        omega_high: 40,
        bval: 11.65,
        minval: 4.5,
        tmn: 26.2,
    },
    CalcPartition {
        index: 25,
        omega_low: 41,
        omega_high: 44,
        bval: 12.24,
        minval: 4.5,
        tmn: 26.7,
    },
    CalcPartition {
        index: 26,
        omega_low: 45,
        omega_high: 48,
        bval: 12.85,
        minval: 4.5,
        tmn: 27.4,
    },
    CalcPartition {
        index: 27,
        omega_low: 49,
        omega_high: 52,
        bval: 13.41,
        minval: 4.5,
        tmn: 27.9,
    },
    CalcPartition {
        index: 28,
        omega_low: 53,
        omega_high: 56,
        bval: 13.94,
        minval: 4.5,
        tmn: 28.4,
    },
    CalcPartition {
        index: 29,
        omega_low: 57,
        omega_high: 60,
        bval: 14.42,
        minval: 4.5,
        tmn: 28.9,
    },
    CalcPartition {
        index: 30,
        omega_low: 61,
        omega_high: 64,
        bval: 14.86,
        minval: 4.5,
        tmn: 29.4,
    },
    CalcPartition {
        index: 31,
        omega_low: 65,
        omega_high: 69,
        bval: 15.32,
        minval: 4.5,
        tmn: 29.8,
    },
    CalcPartition {
        index: 32,
        omega_low: 70,
        omega_high: 74,
        bval: 15.79,
        minval: 4.5,
        tmn: 30.3,
    },
    CalcPartition {
        index: 33,
        omega_low: 75,
        omega_high: 80,
        bval: 16.26,
        minval: 4.5,
        tmn: 30.8,
    },
    CalcPartition {
        index: 34,
        omega_low: 81,
        omega_high: 86,
        bval: 16.73,
        minval: 4.5,
        tmn: 31.2,
    },
    CalcPartition {
        index: 35,
        omega_low: 87,
        omega_high: 93,
        bval: 17.19,
        minval: 4.5,
        tmn: 31.7,
    },
    CalcPartition {
        index: 36,
        omega_low: 94,
        omega_high: 100,
        bval: 17.62,
        minval: 4.5,
        tmn: 32.1,
    },
    CalcPartition {
        index: 37,
        omega_low: 101,
        omega_high: 108,
        bval: 18.05,
        minval: 4.5,
        tmn: 32.5,
    },
    CalcPartition {
        index: 38,
        omega_low: 109,
        omega_high: 116,
        bval: 18.45,
        minval: 4.5,
        tmn: 32.9,
    },
    CalcPartition {
        index: 39,
        omega_low: 117,
        omega_high: 124,
        bval: 18.83,
        minval: 4.5,
        tmn: 33.3,
    },
    CalcPartition {
        index: 40,
        omega_low: 125,
        omega_high: 134,
        bval: 19.21,
        minval: 4.5,
        tmn: 33.7,
    },
    CalcPartition {
        index: 41,
        omega_low: 135,
        omega_high: 144,
        bval: 19.60,
        minval: 4.5,
        tmn: 34.1,
    },
    CalcPartition {
        index: 42,
        omega_low: 145,
        omega_high: 155,
        bval: 20.00,
        minval: 4.5,
        tmn: 34.5,
    },
    CalcPartition {
        index: 43,
        omega_low: 156,
        omega_high: 166,
        bval: 20.38,
        minval: 4.5,
        tmn: 34.9,
    },
    CalcPartition {
        index: 44,
        omega_low: 167,
        omega_high: 177,
        bval: 20.74,
        minval: 4.5,
        tmn: 35.2,
    },
    CalcPartition {
        index: 45,
        omega_low: 178,
        omega_high: 192,
        bval: 21.12,
        minval: 4.5,
        tmn: 35.6,
    },
    CalcPartition {
        index: 46,
        omega_low: 193,
        omega_high: 207,
        bval: 21.48,
        minval: 4.5,
        tmn: 36.0,
    },
    CalcPartition {
        index: 47,
        omega_low: 208,
        omega_high: 222,
        bval: 21.84,
        minval: 4.5,
        tmn: 36.3,
    },
    CalcPartition {
        index: 48,
        omega_low: 223,
        omega_high: 243,
        bval: 22.20,
        minval: 4.5,
        tmn: 36.7,
    },
    CalcPartition {
        index: 49,
        omega_low: 244,
        omega_high: 264,
        bval: 22.56,
        minval: 4.5,
        tmn: 37.1,
    },
    CalcPartition {
        index: 50,
        omega_low: 265,
        omega_high: 286,
        bval: 22.91,
        minval: 4.5,
        tmn: 37.4,
    },
    CalcPartition {
        index: 51,
        omega_low: 287,
        omega_high: 314,
        bval: 23.26,
        minval: 4.5,
        tmn: 37.8,
    },
    CalcPartition {
        index: 52,
        omega_low: 315,
        omega_high: 342,
        bval: 23.60,
        minval: 4.5,
        tmn: 38.1,
    },
    CalcPartition {
        index: 53,
        omega_low: 343,
        omega_high: 371,
        bval: 23.95,
        minval: 4.5,
        tmn: 38.4,
    },
    CalcPartition {
        index: 54,
        omega_low: 372,
        omega_high: 401,
        bval: 24.30,
        minval: 4.5,
        tmn: 38.8,
    },
    CalcPartition {
        index: 55,
        omega_low: 402,
        omega_high: 431,
        bval: 24.65,
        minval: 4.5,
        tmn: 39.1,
    },
    CalcPartition {
        index: 56,
        omega_low: 432,
        omega_high: 469,
        bval: 25.00,
        minval: 4.5,
        tmn: 39.5,
    },
    CalcPartition {
        index: 57,
        omega_low: 470,
        omega_high: 513,
        bval: 25.33,
        minval: 3.5,
        tmn: 39.8,
    },
];

/// Annex D Table D.3c — **complete** calculation-partition table at
/// **Fs = 48 kHz**, all **58** partitions printed in ISO/IEC
/// 11172-3 (1993) PDF page 141 (printed 135).
///
/// Transcribed from the docs collaborator's text extraction
/// `docs/audio/mp3/annex-d-table-D3c-calc-partition-48kHz.csv`,
/// cross-checked against the authoritative render
/// `docs/audio/mp3/annex-d-renders/Table-D.3c-calc-partition-48kHz-p135.png`.
/// Layout matches D.3a/D.3b. The 48 kHz grid packs `bmax = 58`
/// partitions (the most of the three rates); the partitions tile the
/// FFT lines contiguously up to the Nyquist line 513. `bval` rises to
/// 25,81 Bark, and the last two partitions (`57: [466, 507]`,
/// `58: [508, 513]`) drop `minval` to 3,5 dB with `TMN` reaching
/// 40,3 dB.
pub const CALC_PARTITION_48K: [CalcPartition; 58] = [
    CalcPartition {
        index: 1,
        omega_low: 1,
        omega_high: 1,
        bval: 0.00,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 2,
        omega_low: 2,
        omega_high: 2,
        bval: 0.47,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 3,
        omega_low: 3,
        omega_high: 3,
        bval: 0.94,
        minval: 0.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 4,
        omega_low: 4,
        omega_high: 4,
        bval: 1.41,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 5,
        omega_low: 5,
        omega_high: 5,
        bval: 1.88,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 6,
        omega_low: 6,
        omega_high: 6,
        bval: 2.34,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 7,
        omega_low: 7,
        omega_high: 7,
        bval: 2.81,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 8,
        omega_low: 8,
        omega_high: 8,
        bval: 3.28,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 9,
        omega_low: 9,
        omega_high: 9,
        bval: 3.75,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 10,
        omega_low: 10,
        omega_high: 10,
        bval: 4.20,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 11,
        omega_low: 11,
        omega_high: 11,
        bval: 4.63,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 12,
        omega_low: 12,
        omega_high: 12,
        bval: 5.05,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 13,
        omega_low: 13,
        omega_high: 13,
        bval: 5.44,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 14,
        omega_low: 14,
        omega_high: 14,
        bval: 5.83,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 15,
        omega_low: 15,
        omega_high: 15,
        bval: 6.19,
        minval: 20.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 16,
        omega_low: 16,
        omega_high: 16,
        bval: 6.52,
        minval: 17.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 17,
        omega_low: 17,
        omega_high: 17,
        bval: 6.86,
        minval: 17.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 18,
        omega_low: 18,
        omega_high: 20,
        bval: 7.49,
        minval: 15.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 19,
        omega_low: 21,
        omega_high: 23,
        bval: 8.40,
        minval: 10.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 20,
        omega_low: 24,
        omega_high: 26,
        bval: 9.24,
        minval: 7.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 21,
        omega_low: 27,
        omega_high: 29,
        bval: 9.97,
        minval: 7.0,
        tmn: 24.5,
    },
    CalcPartition {
        index: 22,
        omega_low: 30,
        omega_high: 32,
        bval: 10.65,
        minval: 4.4,
        tmn: 25.1,
    },
    CalcPartition {
        index: 23,
        omega_low: 33,
        omega_high: 35,
        bval: 11.28,
        minval: 4.5,
        tmn: 25.8,
    },
    CalcPartition {
        index: 24,
        omega_low: 36,
        omega_high: 38,
        bval: 11.86,
        minval: 4.5,
        tmn: 26.4,
    },
    CalcPartition {
        index: 25,
        omega_low: 39,
        omega_high: 41,
        bval: 12.39,
        minval: 4.5,
        tmn: 26.9,
    },
    CalcPartition {
        index: 26,
        omega_low: 42,
        omega_high: 45,
        bval: 12.96,
        minval: 4.5,
        tmn: 27.5,
    },
    CalcPartition {
        index: 27,
        omega_low: 46,
        omega_high: 49,
        bval: 13.56,
        minval: 4.5,
        tmn: 28.1,
    },
    CalcPartition {
        index: 28,
        omega_low: 50,
        omega_high: 53,
        bval: 14.12,
        minval: 4.5,
        tmn: 28.6,
    },
    CalcPartition {
        index: 29,
        omega_low: 54,
        omega_high: 57,
        bval: 14.62,
        minval: 4.5,
        tmn: 29.1,
    },
    CalcPartition {
        index: 30,
        omega_low: 58,
        omega_high: 62,
        bval: 15.14,
        minval: 4.5,
        tmn: 29.6,
    },
    CalcPartition {
        index: 31,
        omega_low: 63,
        omega_high: 67,
        bval: 15.67,
        minval: 4.5,
        tmn: 30.2,
    },
    CalcPartition {
        index: 32,
        omega_low: 68,
        omega_high: 72,
        bval: 16.15,
        minval: 4.5,
        tmn: 30.7,
    },
    CalcPartition {
        index: 33,
        omega_low: 73,
        omega_high: 77,
        bval: 16.58,
        minval: 4.5,
        tmn: 31.1,
    },
    CalcPartition {
        index: 34,
        omega_low: 78,
        omega_high: 83,
        bval: 17.02,
        minval: 4.5,
        tmn: 31.5,
    },
    CalcPartition {
        index: 35,
        omega_low: 84,
        omega_high: 89,
        bval: 17.44,
        minval: 4.5,
        tmn: 31.9,
    },
    CalcPartition {
        index: 36,
        omega_low: 90,
        omega_high: 95,
        bval: 17.84,
        minval: 4.5,
        tmn: 32.3,
    },
    CalcPartition {
        index: 37,
        omega_low: 96,
        omega_high: 103,
        bval: 18.24,
        minval: 4.5,
        tmn: 32.7,
    },
    CalcPartition {
        index: 38,
        omega_low: 104,
        omega_high: 111,
        bval: 18.66,
        minval: 4.5,
        tmn: 33.2,
    },
    CalcPartition {
        index: 39,
        omega_low: 112,
        omega_high: 120,
        bval: 19.07,
        minval: 4.5,
        tmn: 33.6,
    },
    CalcPartition {
        index: 40,
        omega_low: 121,
        omega_high: 129,
        bval: 19.47,
        minval: 4.5,
        tmn: 34.0,
    },
    CalcPartition {
        index: 41,
        omega_low: 130,
        omega_high: 138,
        bval: 19.85,
        minval: 4.5,
        tmn: 34.3,
    },
    CalcPartition {
        index: 42,
        omega_low: 139,
        omega_high: 149,
        bval: 20.23,
        minval: 4.5,
        tmn: 34.7,
    },
    CalcPartition {
        index: 43,
        omega_low: 150,
        omega_high: 160,
        bval: 20.63,
        minval: 4.5,
        tmn: 35.1,
    },
    CalcPartition {
        index: 44,
        omega_low: 161,
        omega_high: 173,
        bval: 21.02,
        minval: 4.5,
        tmn: 35.5,
    },
    CalcPartition {
        index: 45,
        omega_low: 174,
        omega_high: 187,
        bval: 21.40,
        minval: 4.5,
        tmn: 35.9,
    },
    CalcPartition {
        index: 46,
        omega_low: 188,
        omega_high: 201,
        bval: 21.76,
        minval: 4.5,
        tmn: 36.3,
    },
    CalcPartition {
        index: 47,
        omega_low: 202,
        omega_high: 219,
        bval: 22.12,
        minval: 4.5,
        tmn: 36.6,
    },
    CalcPartition {
        index: 48,
        omega_low: 220,
        omega_high: 238,
        bval: 22.47,
        minval: 4.5,
        tmn: 37.0,
    },
    CalcPartition {
        index: 49,
        omega_low: 239,
        omega_high: 257,
        bval: 22.83,
        minval: 4.5,
        tmn: 37.3,
    },
    CalcPartition {
        index: 50,
        omega_low: 258,
        omega_high: 283,
        bval: 23.18,
        minval: 4.5,
        tmn: 37.7,
    },
    CalcPartition {
        index: 51,
        omega_low: 284,
        omega_high: 309,
        bval: 23.53,
        minval: 4.5,
        tmn: 38.0,
    },
    CalcPartition {
        index: 52,
        omega_low: 310,
        omega_high: 335,
        bval: 23.88,
        minval: 4.5,
        tmn: 38.4,
    },
    CalcPartition {
        index: 53,
        omega_low: 336,
        omega_high: 363,
        bval: 24.23,
        minval: 4.5,
        tmn: 38.7,
    },
    CalcPartition {
        index: 54,
        omega_low: 364,
        omega_high: 391,
        bval: 24.58,
        minval: 4.5,
        tmn: 39.1,
    },
    CalcPartition {
        index: 55,
        omega_low: 392,
        omega_high: 423,
        bval: 24.93,
        minval: 4.5,
        tmn: 39.4,
    },
    CalcPartition {
        index: 56,
        omega_low: 424,
        omega_high: 465,
        bval: 25.27,
        minval: 4.5,
        tmn: 39.8,
    },
    CalcPartition {
        index: 57,
        omega_low: 466,
        omega_high: 507,
        bval: 25.61,
        minval: 3.5,
        tmn: 40.1,
    },
    CalcPartition {
        index: 58,
        omega_low: 508,
        omega_high: 513,
        bval: 25.81,
        minval: 3.5,
        tmn: 40.3,
    },
];

/// Look up a row of Annex D **Table D.3b** (Fs = 44,1 kHz
/// calculation-partition table) by its 1-based partition number `n`.
///
/// Returns `Some(row)` for `n ∈ 1..=57` and `None` for `n == 0` (the
/// table is 1-based) and `n > 57` (above the printed table). Mirrors
/// [`calc_partition_32k`] for the 44,1 kHz rate.
pub fn calc_partition_44k1(n: u16) -> Option<CalcPartition> {
    let idx = usize::from(n).checked_sub(1)?;
    CALC_PARTITION_44K1.get(idx).copied()
}

/// Look up a row of Annex D **Table D.3c** (Fs = 48 kHz
/// calculation-partition table) by its 1-based partition number `n`.
///
/// Returns `Some(row)` for `n ∈ 1..=58` and `None` for `n == 0` and
/// `n > 58`. Mirrors [`calc_partition_32k`] for the 48 kHz rate.
pub fn calc_partition_48k(n: u16) -> Option<CalcPartition> {
    let idx = usize::from(n).checked_sub(1)?;
    CALC_PARTITION_48K.get(idx).copied()
}

// -----------------------------------------------------------------
// Annex D Step 3 — threshold-in-quiet `LTq` bit-rate-dependent offset
//
// Quoted verbatim from the staged docs extract (text-layer readable
// in the PDF):
//
//   "An offset depending on the overall bit rate is used for the
//    absolute threshold. This offset is -12 dB for bit rates >=
//    96 kbits/s and 0 dB for bit rates < 96 kbits/s per channel."
//
// The rule's wording "per channel" controls the comparison threshold:
// it is the per-channel bit rate that gates which offset applies,
// matching how the Annex D allocator consumes the budget. The Step 3
// reading of the §D.1 Table D.1x absolute-threshold column is then
//
//   LTq_used(i) = LTq_table(i) + ltq_offset_db(bit_rate_per_channel)
// -----------------------------------------------------------------

/// Annex D Step 3 — bit-rate-dependent offset applied to the §D.1
/// threshold-in-quiet `LTq(i)` column.
///
/// Returns `-12.0` dB when `bit_rate_per_channel_kbps >= 96`, and
/// `0.0` dB otherwise. The argument is the **per-channel** bit rate
/// in kbit/s, as the spec wording specifies. For a stereo / dual /
/// joint stream the caller divides the overall bit rate by the active
/// channel count before calling; for mono the overall rate is the
/// per-channel rate.
#[inline]
pub fn ltq_offset_db(bit_rate_per_channel_kbps: u32) -> f64 {
    if bit_rate_per_channel_kbps >= 96 {
        -12.0
    } else {
        0.0
    }
}

// -----------------------------------------------------------------
// Annex D clause D.2.3 (Psychoacoustic Model 2) — the "spreading
// function", complete
//
// The `tmpx` / `x` lines and the `sprdngf` cutoff condition extract
// from the PDF text layer; the `tmpy` line and the `sprdngf`
// exponent are typeset as equation images and were read from a
// 150-DPI render of the staged ISO/IEC 11172-3:1993 PDF page 135
// (printed p.129), where they are fully legible:
//
//   tmpx = 1,05 (j - i)           ; i = Bark of signal spread,
//                                   j = Bark of band spread into
//   x    = 8 minimum( (tmpx - 0,5)^2 - 2(tmpx - 0,5), 0 )
//   tmpy = 15,811389 + 7,5(tmpx + 0,474)
//          - 17,5(1,0 + (tmpx + 0,474)^2)^0,5
//   if (tmpy < -100) then sprdngf(i,j) = 0
//                    else sprdngf(i,j) = 10^((x + tmpy)/10)
//
// NOTE: the PDF *text layer* drops the `x +` term from the
// `sprdngf` exponent (pdftotext yields `10^(tmpy/10)`); the printed
// equation image carries `(x + tmpy)/10` and is the authoritative
// reading. `sprdngf_from_tmpy` below keeps the one-argument
// reduction (exact whenever `x == 0`, i.e. outside the
// `model2_x_is_active` window); `model2_sprdngf` is the full
// printed-form per-pair composition.
// -----------------------------------------------------------------

/// Model 2 spreading-function `tmpx` term (clause D.2, text-extracted
/// verbatim): `tmpx = 1.05 · (j - i)`.
///
/// `j_bark` is the Bark of the band the masker is spread into; `i_bark`
/// is the Bark of the signal being spread. The sign convention follows
/// the spec literally — negative `tmpx` for `j < i`.
#[inline]
pub fn model2_tmpx(j_bark: f64, i_bark: f64) -> f64 {
    1.05 * (j_bark - i_bark)
}

/// Model 2 spreading-function `x` term (clause D.2, text-extracted
/// verbatim): `x = 8 · min((tmpx − 0.5)^2 − 2·(tmpx − 0.5), 0)`.
///
/// The `min(_, 0)` clamps the dB attenuation to never go positive — `x`
/// is always `<= 0`. The function peaks at `tmpx = 0.5` (return value
/// `0`) and is `0` again at `tmpx = 2.5` (`(2.0)^2 − 2·2.0 = 0`).
#[inline]
pub fn model2_x(tmpx: f64) -> f64 {
    let s = tmpx - 0.5;
    let v = s * s - 2.0 * s;
    8.0 * v.min(0.0)
}

/// Model 2 spreading-function post-step (clause D.2.3), **`x = 0`
/// reduction**: given the intermediate `tmpy` (dB), returns
/// `0` when `tmpy < -100`, else `10^(tmpy / 10)`.
///
/// The full printed post-step is `sprdngf(i, j) = 10^((x + tmpy)/10)`
/// (cutoff still on `tmpy` alone) — see [`model2_sprdngf`] for the
/// complete per-pair composition. This one-argument form equals the
/// printed form exactly whenever the `x` term is zero, i.e. for every
/// pair **outside** the [`model2_x_is_active`] window (`tmpx ∉
/// (0.5, 2.5)`), where the [`model2_x`] `min(_, 0)` clamp engages.
#[inline]
pub fn sprdngf_from_tmpy(tmpy_db: f64) -> f64 {
    if tmpy_db < -100.0 {
        0.0
    } else {
        10f64.powf(tmpy_db / 10.0)
    }
}

// -----------------------------------------------------------------
// Step 4 helper — FFT-line → critical-band lookup
//
// Tables D.2a..f give critical-band boundaries as the **top** FFT-line
// index (column `index F&CB`, 1-based into the matching Table D.1x).
// Step 4 of the Annex D Model 1 prose (clause D.1) walks every FFT
// line of the analysis window and asks "which critical band does this
// line fall into?". The mapping is a strict closed-form over the
// already-staged D.2x boundary list: the band containing line `i` is
// the lowest band `k` whose `index_fcb >= i`.
//
// Returning the 0-based band index makes this composable with `.len()`
// / iteration over the band slice the caller obtained from
// `critical_band_table()`.
// -----------------------------------------------------------------

/// Map an FFT-line index (1-based into the Table D.1x
/// `index F&CB` column) to the 0-based critical-band number that
/// contains it, per the Table D.2x boundaries staged in
/// [`critical_band_table`].
///
/// Returns `None` when:
///
/// * `(layer, sampling_frequency_hz)` is outside the
///   `{32 000, 44 100, 48 000}` × `{Layer::I, Layer::II}` set
///   Annex D covers (mirrors [`critical_band_table`]).
/// * `line_index_fcb == 0` — the spec is 1-based.
/// * `line_index_fcb` is above the table's highest band-top
///   (`bands.last().index_fcb`) — the line lies above the top of the
///   audio band the D.1x/D.2x tables cover and is therefore not
///   masked by any critical band.
///
/// The band containing line `i` is the smallest band `k` such that
/// `bands[k].index_fcb >= i`. The Table D.2x boundary column is the
/// **upper** edge of each band, so this is the correct comparator
/// (a line with `index_fcb == bands[k].index_fcb` belongs to band
/// `k`, not `k+1`).
pub fn critical_band_for_line(
    layer: Layer,
    sampling_frequency_hz: u32,
    line_index_fcb: u16,
) -> Option<usize> {
    if line_index_fcb == 0 {
        return None;
    }
    let bands = critical_band_table(layer, sampling_frequency_hz)?;
    bands.iter().position(|b| b.index_fcb >= line_index_fcb)
}

// -----------------------------------------------------------------
// Step 3 — apply `LTq` bit-rate-dependent offset to a per-line value
//
// The offset itself is staged as [`ltq_offset_db`]. This is the
// trivial adapter the allocator wires at the per-line site:
//
//   LTq_used(i) = LTq_table(i) + ltq_offset_db(per_channel_rate)
//
// Splitting the two functions keeps the bit-rate predicate
// independently testable (it is the part the encoder may evaluate
// **once** per frame) while the per-line apply is a pure addition.
// -----------------------------------------------------------------

/// Annex D Step 3 — apply the bit-rate offset to a single tabulated
/// threshold-in-quiet value.
///
/// `ltq_table_db` is the Table D.1x absolute-threshold column for an
/// FFT line; `bit_rate_per_channel_kbps` selects the offset per
/// [`ltq_offset_db`]. Returns the per-line `LTq_used(i) = LTq +
/// offset` in dB.
///
/// This is a one-line helper; its purpose is to document the Step 3
/// composition site so callers can reach for a single function rather
/// than re-deriving the addition. The D.1x table values are still
/// DOCS-GAP (PNG-only), so this helper exists ahead of its primary
/// data source.
#[inline]
pub fn step3_apply_ltq_offset(ltq_table_db: f64, bit_rate_per_channel_kbps: u32) -> f64 {
    ltq_table_db + ltq_offset_db(bit_rate_per_channel_kbps)
}

// -----------------------------------------------------------------
// Step 7 — filtered global-threshold variant
//
// The base [`global_threshold_db`] takes pre-filtered slices of
// individual masker thresholds (the Bark-window predicate is left to
// the caller). The Step 7 prose also says, verbatim:
//
//   "For a given i the range of j may be reduced to maskers within
//    −8…+3 Bark of i."
//
// That secondary window is wider than the `vf` Bark window
// (`-3 <= dz < 8`, applied per masker by [`masking_function`] /
// [`individual_threshold_tonal`] / [`individual_threshold_non_tonal`]).
// In practice the `vf` `None` is the stricter filter — a masker that
// `vf` ignores will never contribute. The convenience entry point
// here takes raw `(masker_bark, masker_spl_db)` pairs, evaluates each
// masker through the matching `individual_threshold_*` helper at the
// evaluation-line Bark `z_i`, drops the `None` cases, and feeds the
// rest into the same power-domain sum [`global_threshold_db`]
// performs — surfacing the composite Step-6+Step-7 evaluation as one
// call instead of asking the caller to build the slices.
// -----------------------------------------------------------------

/// Annex D Step 6 + Step 7 — composite per-line global masking
/// threshold from raw masker lists, evaluated at FFT-line Bark `z_i`.
///
/// `tonal_maskers` and `non_tonal_maskers` each carry `(z_j, X_db)`
/// pairs — masker Bark `z_j` and masker SPL `X` in dB. Maskers whose
/// Bark distance `z_i − z_j` lies outside the `[-3, 8)` spreading-
/// function window are ignored automatically (the matching
/// `individual_threshold_*` helper returns `None` and the contribution
/// is dropped). The remaining individual thresholds are accumulated
/// into the power-domain Step 7 sum
///
/// `LTg(i) = 10·log10( 10^(LTq/10) + Σ 10^(LT_tm/10) + Σ 10^(LT_nm/10) )`
///
/// alongside `ltq_used_db` (the **already-offset** Step 3 threshold —
/// callers can compose [`step3_apply_ltq_offset`] for the per-line
/// term).
pub fn global_threshold_db_from_maskers(
    z_i: f64,
    ltq_used_db: f64,
    tonal_maskers: &[(f64, f64)],
    non_tonal_maskers: &[(f64, f64)],
) -> f64 {
    let mut acc = 10f64.powf(ltq_used_db / 10.0);
    for &(z_j, x_db) in tonal_maskers {
        if let Some(lt) = individual_threshold_tonal(z_j, z_i, x_db) {
            acc += 10f64.powf(lt / 10.0);
        }
    }
    for &(z_j, x_db) in non_tonal_maskers {
        if let Some(lt) = individual_threshold_non_tonal(z_j, z_i, x_db) {
            acc += 10f64.powf(lt / 10.0);
        }
    }
    10.0 * acc.log10()
}

// -----------------------------------------------------------------
// Model 2 spreading function — `x` term per (masker-Bark, line-Bark)
//
// Composes the two text-extractable pieces already staged
// ([`model2_tmpx`] and [`model2_x`]) into the per-pair site the
// allocator wires when walking calculation-partition pairs. Mirrors
// the [`step3_apply_ltq_offset`] adapter style for [`ltq_offset_db`].
//
// Spec composition (clause D.2):
//
//   tmpx = 1,05 * (j - i)
//   x    = 8 * minimum( (tmpx - 0,5)^2 - 2*(tmpx - 0,5), 0 )
//
// The `model2_x` clamp guarantees `x <= 0` everywhere; the open
// interval where the inner term is strictly negative is
// `tmpx ∈ (0.5, 2.5)`, i.e. `j - i ∈ (0.5/1.05, 2.5/1.05) ≈
// (0.476, 2.381)`. Outside that open interval the `x` term is
// exactly zero, so the spreading-function contribution from `x`
// vanishes — the still-DOCS-GAP `tmpy` line will reduce to its
// `minval` term alone for those pairs (per the prose convention).
// `model2_x_is_active` exposes the predicate so callers can
// short-circuit the `tmpy` evaluation site for inactive pairs.
// -----------------------------------------------------------------

/// Model 2 spreading-function `x` term evaluated for a `(j_bark,
/// i_bark)` pair (clause D.2).
///
/// Composes [`model2_tmpx`] with [`model2_x`]:
///
/// `x(i, j) = model2_x(model2_tmpx(j_bark, i_bark))`
///
/// `j_bark` is the Bark of the band the masker is spread into;
/// `i_bark` is the Bark of the signal being spread. The function is
/// non-positive everywhere (the `model2_x` `min(_, 0)` clamp) and
/// strictly negative only when `j_bark - i_bark ∈ (0.5/1.05,
/// 2.5/1.05)` — see [`model2_x_is_active`] for the same predicate
/// exposed as a `bool`.
#[inline]
pub fn model2_x_for_pair(j_bark: f64, i_bark: f64) -> f64 {
    model2_x(model2_tmpx(j_bark, i_bark))
}

/// Predicate: is the Model 2 spreading-function `x` term strictly
/// negative for this `(j_bark, i_bark)` pair?
///
/// Returns `true` iff `tmpx ∈ (0.5, 2.5)` (the open interval where
/// `(tmpx - 0.5)^2 - 2·(tmpx - 0.5)` is negative and the
/// [`model2_x`] `min(_, 0)` clamp is *not* engaged). Equivalently in
/// Bark coordinates: `j_bark - i_bark ∈ (0.5/1.05, 2.5/1.05) ≈
/// (0.47619…, 2.38095…)`.
///
/// Outside this open interval `model2_x_for_pair(j, i) == 0.0`
/// exactly, which lets callers walking calculation-partition pairs
/// skip the per-pair `tmpy` evaluation site entirely (the
/// `x`-contribution is zero, and the post-step
/// [`sprdngf_from_tmpy`] only depends on the remaining `tmpy`
/// terms).
#[inline]
pub fn model2_x_is_active(j_bark: f64, i_bark: f64) -> bool {
    let t = model2_tmpx(j_bark, i_bark);
    t > 0.5 && t < 2.5
}

/// Model 2 spreading-function `tmpy` term (clause D.2.3, read from
/// the 150-DPI render of the staged ISO PDF page 135 / printed
/// p.129):
///
/// `tmpy = 15.811389 + 7.5·(tmpx + 0.474) − 17.5·√(1.0 + (tmpx + 0.474)²)`
///
/// This is the dB backbone of the spreading function. With
/// `u = tmpx + 0.474` it is `15.811389 + 7.5·u − 17.5·√(1 + u²)` — a
/// hyperbola whose two asymptote slopes are `7.5 + 17.5 = 25` dB per
/// `tmpx` unit on the `tmpx → −∞` side and `7.5 − 17.5 = −10` dB per
/// `tmpx` unit on the `tmpx → +∞` side (i.e. `26.25` dB/Bark below
/// the masker and `10.5` dB/Bark above it after the `tmpx = 1.05·(j −
/// i)` scaling). The constant `15.811389` equals
/// `17.5·√(1 + u₀²) − 7.5·u₀` at the stationary point `u₀ = 3/√40`
/// to within `10⁻⁶`, so the curve's maximum is `0 dB` (within
/// `~10⁻⁶`), reached at `tmpx ≈ 0` — the spreading backbone never
/// amplifies.
#[inline]
pub fn model2_tmpy(tmpx: f64) -> f64 {
    let u = tmpx + 0.474;
    15.811_389 + 7.5 * u - 17.5 * (1.0 + u * u).sqrt()
}

/// Model 2 spreading function `sprdngf(i, j)` for a `(j_bark,
/// i_bark)` pair (clause D.2.3, complete printed form):
///
/// * `tmpx = 1.05·(j − i)` — [`model2_tmpx`]
/// * `x = 8·min((tmpx − 0.5)² − 2·(tmpx − 0.5), 0)` — [`model2_x`]
/// * `tmpy = 15.811389 + 7.5·(tmpx + 0.474) − 17.5·√(1 + (tmpx +
///   0.474)²)` — [`model2_tmpy`]
/// * `if tmpy < −100 then sprdngf(i, j) = 0 else sprdngf(i, j) =
///   10^((x + tmpy)/10)`
///
/// `i_bark` is the Bark value of the signal being spread; `j_bark`
/// is the Bark value of the band being spread into. Note the
/// **cutoff tests `tmpy` alone** while the surviving branch raises
/// `10` to `(x + tmpy)/10` — the `x` term sharpens the near-field
/// skirt (it is non-zero only for `tmpx ∈ (0.5, 2.5)`) but does not
/// participate in the kill test. The value is a power-domain weight:
/// `≈ 1` at `j == i`, decaying toward `0` in both directions, with
/// the `tmpy < −100` cutoff zeroing pairs beyond `j − i ≈ −4.79`
/// Bark on the steep side and `j − i ≈ +10.51` Bark on the shallow
/// side.
#[inline]
pub fn model2_sprdngf(j_bark: f64, i_bark: f64) -> f64 {
    let tmpx = model2_tmpx(j_bark, i_bark);
    let tmpy = model2_tmpy(tmpx);
    if tmpy < -100.0 {
        0.0
    } else {
        let x = model2_x(tmpx);
        10f64.powf((x + tmpy) / 10.0)
    }
}

// -----------------------------------------------------------------
// Annex D Table D.1a — "Frequencies, critical band rates and
// absolute threshold" (threshold in quiet, LTq) for Layer I at
// Fs = 32 kHz (partial anchor: rows i = 1..=5 plus the final row
// i = 108 of 108).
//
// Spec context: clause D.1 Steps 3 and 4 (and D.2). Each row of
// Table D.1x carries four columns: the 1-based index number `i`
// (the `index F&CB` namespace Tables D.2x point into), the FFT-line
// frequency in Hz, the critical-band rate `z` in Bark, and the
// absolute threshold (threshold in quiet) `LTq` in dB. Layer I
// tables print 108 entries; Layer II tables print 132.
//
// The staged docs extract
// (`docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md`,
// "Table D.1a–f — Threshold in quiet (absolute threshold) LTq")
// transcribes only the first five rows and the final row of D.1a
// as a cross-checked text anchor — the six dense table pages are
// staged as authoritative PNG renders under
// `docs/audio/mp3/annex-d-renders/` because the PDF text layer
// corrupts the comma/period and several digits in the tight
// two-column-per-page layout. The body of D.1a (i = 6..=107) and
// the full D.1b–f tables therefore remain DOCS-GAP awaiting the
// same render → text transcription cycle that unblocked Tables
// B.1 / B.3.
//
// This partial transcription parallels the `CALC_PARTITION_32K`
// (Table D.3a) staging: the legible
// portion lands as a typed `const` with explicit row numbering,
// the lookup helper surfaces the DOCS-GAP boundary as `None`, and
// downstream consumers can branch on it to fall back through the
// energy-driven allocator path until the remainder is transcribed.
// -----------------------------------------------------------------

/// One row of Annex D Table D.1x ("Frequencies, critical band rates
/// and absolute threshold").
///
/// Carries the four spec columns verbatim: the 1-based index number
/// `i` (`index`, the `index F&CB` namespace the Table D.2x
/// `index_fcb` column points into), the FFT-line frequency in Hz
/// (`freq_hz`), the critical-band rate `z` in Bark (`bark_z`), and
/// the absolute threshold — threshold in quiet — `LTq` in dB
/// (`ltq_db`). The tabulated `ltq_db` is the **pre-offset** Step 3
/// value; apply [`step3_apply_ltq_offset`] (or use
/// [`ltq_layer1_32k_used`]) to obtain the per-frame `LTq_used`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LtqRow {
    /// Index number `i` (1-based, per the printed spec; shared with
    /// the Table D.2x `index F&CB` column).
    pub index: u16,
    /// FFT-line frequency in Hz.
    pub freq_hz: f64,
    /// Critical-band rate `z` in Bark.
    pub bark_z: f64,
    /// Absolute threshold (threshold in quiet) in dB, pre-offset.
    pub ltq_db: f64,
}

/// Number of rows in each **Layer I** Table D.1x (D.1a–c) per the
/// clause D.1 prose: `i = 1…108`.
pub const LTQ_LAYER1_FULL_LEN: usize = 108;

/// Number of rows in each **Layer II** Table D.1x (D.1d–f) per the
/// clause D.1 prose: 132 entries.
pub const LTQ_LAYER2_FULL_LEN: usize = 132;

/// Annex D Table D.1a — threshold in quiet (absolute threshold) for
/// **Layer I** at **Fs = 32 kHz**, the **complete** 108-row table
/// printed in ISO/IEC 11172-3 (1993) PDF page 122 (printed 116).
///
/// Transcribed from the docs collaborator's text extraction
/// `docs/audio/mp3/annex-d-table-D1a-threshold-32kHz.csv`, which
/// captures all four printed columns (1-based index `i`, FFT-line
/// frequency in Hz, critical-band rate `z` in Bark, and the
/// absolute-threshold `ltq_db`) cross-checked against the render
/// `docs/audio/mp3/annex-d-renders/Table-D.1a-threshold-in-quiet-LayerI-32kHz-p116.png`.
/// The column is **non-monotonic**: it falls to a minimum of
/// `-4.97 dB` at `i = 51` (≈ 3,375 kHz), then climbs steeply at both
/// ends (33,44 dB at `i = 1`, 51,04 dB at `i = 108`).
///
/// `i = 6..=107` were previously a DOCS-GAP carried as a 6-row
/// partial anchor; the docs `#129`/Annex-D extraction round filled
/// them. The companion Layer I tables D.1b (44,1 kHz) / D.1c
/// (48 kHz) and the Layer II tables D.1d–f are likewise now staged
/// as CSVs but are not transcribed here (32 kHz Layer I only this
/// round).
// The row i = 8 threshold `6.28 dB` is a spec table value, not an
// approximation of `TAU` (6.283…); silence clippy's approx_constant.
#[allow(clippy::approx_constant)]
pub const LTQ_L1_32K: [LtqRow; LTQ_LAYER1_FULL_LEN] = [
    LtqRow {
        index: 1,
        freq_hz: 62.50,
        bark_z: 0.617,
        ltq_db: 33.44,
    },
    LtqRow {
        index: 2,
        freq_hz: 125.00,
        bark_z: 1.232,
        ltq_db: 19.20,
    },
    LtqRow {
        index: 3,
        freq_hz: 187.50,
        bark_z: 1.842,
        ltq_db: 13.87,
    },
    LtqRow {
        index: 4,
        freq_hz: 250.00,
        bark_z: 2.445,
        ltq_db: 11.01,
    },
    LtqRow {
        index: 5,
        freq_hz: 312.50,
        bark_z: 3.037,
        ltq_db: 9.20,
    },
    LtqRow {
        index: 6,
        freq_hz: 375.00,
        bark_z: 3.618,
        ltq_db: 7.94,
    },
    LtqRow {
        index: 7,
        freq_hz: 437.50,
        bark_z: 4.185,
        ltq_db: 7.00,
    },
    LtqRow {
        index: 8,
        freq_hz: 500.00,
        bark_z: 4.736,
        ltq_db: 6.28,
    },
    LtqRow {
        index: 9,
        freq_hz: 562.50,
        bark_z: 5.272,
        ltq_db: 5.70,
    },
    LtqRow {
        index: 10,
        freq_hz: 625.00,
        bark_z: 5.789,
        ltq_db: 5.21,
    },
    LtqRow {
        index: 11,
        freq_hz: 687.50,
        bark_z: 6.289,
        ltq_db: 4.80,
    },
    LtqRow {
        index: 12,
        freq_hz: 750.00,
        bark_z: 6.770,
        ltq_db: 4.45,
    },
    LtqRow {
        index: 13,
        freq_hz: 812.50,
        bark_z: 7.233,
        ltq_db: 4.14,
    },
    LtqRow {
        index: 14,
        freq_hz: 875.00,
        bark_z: 7.677,
        ltq_db: 3.86,
    },
    LtqRow {
        index: 15,
        freq_hz: 937.50,
        bark_z: 8.103,
        ltq_db: 3.61,
    },
    LtqRow {
        index: 16,
        freq_hz: 1000.00,
        bark_z: 8.511,
        ltq_db: 3.37,
    },
    LtqRow {
        index: 17,
        freq_hz: 1062.50,
        bark_z: 8.901,
        ltq_db: 3.15,
    },
    LtqRow {
        index: 18,
        freq_hz: 1125.00,
        bark_z: 9.275,
        ltq_db: 2.93,
    },
    LtqRow {
        index: 19,
        freq_hz: 1187.50,
        bark_z: 9.632,
        ltq_db: 2.73,
    },
    LtqRow {
        index: 20,
        freq_hz: 1250.00,
        bark_z: 9.974,
        ltq_db: 2.53,
    },
    LtqRow {
        index: 21,
        freq_hz: 1312.50,
        bark_z: 10.301,
        ltq_db: 2.32,
    },
    LtqRow {
        index: 22,
        freq_hz: 1375.00,
        bark_z: 10.614,
        ltq_db: 2.12,
    },
    LtqRow {
        index: 23,
        freq_hz: 1437.50,
        bark_z: 10.913,
        ltq_db: 1.92,
    },
    LtqRow {
        index: 24,
        freq_hz: 1500.00,
        bark_z: 11.199,
        ltq_db: 1.71,
    },
    LtqRow {
        index: 25,
        freq_hz: 1562.50,
        bark_z: 11.474,
        ltq_db: 1.49,
    },
    LtqRow {
        index: 26,
        freq_hz: 1625.00,
        bark_z: 11.736,
        ltq_db: 1.27,
    },
    LtqRow {
        index: 27,
        freq_hz: 1687.50,
        bark_z: 11.988,
        ltq_db: 1.04,
    },
    LtqRow {
        index: 28,
        freq_hz: 1750.00,
        bark_z: 12.230,
        ltq_db: 0.80,
    },
    LtqRow {
        index: 29,
        freq_hz: 1812.50,
        bark_z: 12.461,
        ltq_db: 0.55,
    },
    LtqRow {
        index: 30,
        freq_hz: 1875.00,
        bark_z: 12.684,
        ltq_db: 0.29,
    },
    LtqRow {
        index: 31,
        freq_hz: 1937.50,
        bark_z: 12.898,
        ltq_db: 0.02,
    },
    LtqRow {
        index: 32,
        freq_hz: 2000.00,
        bark_z: 13.104,
        ltq_db: -0.25,
    },
    LtqRow {
        index: 33,
        freq_hz: 2062.50,
        bark_z: 13.302,
        ltq_db: -0.54,
    },
    LtqRow {
        index: 34,
        freq_hz: 2125.00,
        bark_z: 13.493,
        ltq_db: -0.83,
    },
    LtqRow {
        index: 35,
        freq_hz: 2187.50,
        bark_z: 13.678,
        ltq_db: -1.12,
    },
    LtqRow {
        index: 36,
        freq_hz: 2250.00,
        bark_z: 13.855,
        ltq_db: -1.43,
    },
    LtqRow {
        index: 37,
        freq_hz: 2312.50,
        bark_z: 14.027,
        ltq_db: -1.73,
    },
    LtqRow {
        index: 38,
        freq_hz: 2375.00,
        bark_z: 14.193,
        ltq_db: -2.04,
    },
    LtqRow {
        index: 39,
        freq_hz: 2437.50,
        bark_z: 14.354,
        ltq_db: -2.34,
    },
    LtqRow {
        index: 40,
        freq_hz: 2500.00,
        bark_z: 14.509,
        ltq_db: -2.64,
    },
    LtqRow {
        index: 41,
        freq_hz: 2562.50,
        bark_z: 14.660,
        ltq_db: -2.93,
    },
    LtqRow {
        index: 42,
        freq_hz: 2625.00,
        bark_z: 14.807,
        ltq_db: -3.22,
    },
    LtqRow {
        index: 43,
        freq_hz: 2687.50,
        bark_z: 14.949,
        ltq_db: -3.49,
    },
    LtqRow {
        index: 44,
        freq_hz: 2750.00,
        bark_z: 15.087,
        ltq_db: -3.74,
    },
    LtqRow {
        index: 45,
        freq_hz: 2812.50,
        bark_z: 15.221,
        ltq_db: -3.98,
    },
    LtqRow {
        index: 46,
        freq_hz: 2875.00,
        bark_z: 15.351,
        ltq_db: -4.20,
    },
    LtqRow {
        index: 47,
        freq_hz: 2937.50,
        bark_z: 15.478,
        ltq_db: -4.40,
    },
    LtqRow {
        index: 48,
        freq_hz: 3000.00,
        bark_z: 15.602,
        ltq_db: -4.57,
    },
    LtqRow {
        index: 49,
        freq_hz: 3125.00,
        bark_z: 15.841,
        ltq_db: -4.82,
    },
    LtqRow {
        index: 50,
        freq_hz: 3250.00,
        bark_z: 16.069,
        ltq_db: -4.96,
    },
    LtqRow {
        index: 51,
        freq_hz: 3375.00,
        bark_z: 16.287,
        ltq_db: -4.97,
    },
    LtqRow {
        index: 52,
        freq_hz: 3500.00,
        bark_z: 16.496,
        ltq_db: -4.86,
    },
    LtqRow {
        index: 53,
        freq_hz: 3625.00,
        bark_z: 16.697,
        ltq_db: -4.63,
    },
    LtqRow {
        index: 54,
        freq_hz: 3750.00,
        bark_z: 16.891,
        ltq_db: -4.29,
    },
    LtqRow {
        index: 55,
        freq_hz: 3875.00,
        bark_z: 17.078,
        ltq_db: -3.87,
    },
    LtqRow {
        index: 56,
        freq_hz: 4000.00,
        bark_z: 17.259,
        ltq_db: -3.39,
    },
    LtqRow {
        index: 57,
        freq_hz: 4125.00,
        bark_z: 17.434,
        ltq_db: -2.86,
    },
    LtqRow {
        index: 58,
        freq_hz: 4250.00,
        bark_z: 17.605,
        ltq_db: -2.31,
    },
    LtqRow {
        index: 59,
        freq_hz: 4375.00,
        bark_z: 17.770,
        ltq_db: -1.77,
    },
    LtqRow {
        index: 60,
        freq_hz: 4500.00,
        bark_z: 17.932,
        ltq_db: -1.24,
    },
    LtqRow {
        index: 61,
        freq_hz: 4625.00,
        bark_z: 18.089,
        ltq_db: -0.74,
    },
    LtqRow {
        index: 62,
        freq_hz: 4750.00,
        bark_z: 18.242,
        ltq_db: -0.29,
    },
    LtqRow {
        index: 63,
        freq_hz: 4875.00,
        bark_z: 18.392,
        ltq_db: 0.12,
    },
    LtqRow {
        index: 64,
        freq_hz: 5000.00,
        bark_z: 18.539,
        ltq_db: 0.48,
    },
    LtqRow {
        index: 65,
        freq_hz: 5125.00,
        bark_z: 18.682,
        ltq_db: 0.79,
    },
    LtqRow {
        index: 66,
        freq_hz: 5250.00,
        bark_z: 18.823,
        ltq_db: 1.06,
    },
    LtqRow {
        index: 67,
        freq_hz: 5375.00,
        bark_z: 18.960,
        ltq_db: 1.29,
    },
    LtqRow {
        index: 68,
        freq_hz: 5500.00,
        bark_z: 19.095,
        ltq_db: 1.49,
    },
    LtqRow {
        index: 69,
        freq_hz: 5625.00,
        bark_z: 19.226,
        ltq_db: 1.66,
    },
    LtqRow {
        index: 70,
        freq_hz: 5750.00,
        bark_z: 19.356,
        ltq_db: 1.81,
    },
    LtqRow {
        index: 71,
        freq_hz: 5875.00,
        bark_z: 19.482,
        ltq_db: 1.95,
    },
    LtqRow {
        index: 72,
        freq_hz: 6000.00,
        bark_z: 19.606,
        ltq_db: 2.08,
    },
    LtqRow {
        index: 73,
        freq_hz: 6250.00,
        bark_z: 19.847,
        ltq_db: 2.33,
    },
    LtqRow {
        index: 74,
        freq_hz: 6500.00,
        bark_z: 20.079,
        ltq_db: 2.59,
    },
    LtqRow {
        index: 75,
        freq_hz: 6750.00,
        bark_z: 20.300,
        ltq_db: 2.86,
    },
    LtqRow {
        index: 76,
        freq_hz: 7000.00,
        bark_z: 20.513,
        ltq_db: 3.17,
    },
    LtqRow {
        index: 77,
        freq_hz: 7250.00,
        bark_z: 20.717,
        ltq_db: 3.51,
    },
    LtqRow {
        index: 78,
        freq_hz: 7500.00,
        bark_z: 20.912,
        ltq_db: 3.89,
    },
    LtqRow {
        index: 79,
        freq_hz: 7750.00,
        bark_z: 21.098,
        ltq_db: 4.31,
    },
    LtqRow {
        index: 80,
        freq_hz: 8000.00,
        bark_z: 21.275,
        ltq_db: 4.79,
    },
    LtqRow {
        index: 81,
        freq_hz: 8250.00,
        bark_z: 21.445,
        ltq_db: 5.31,
    },
    LtqRow {
        index: 82,
        freq_hz: 8500.00,
        bark_z: 21.606,
        ltq_db: 5.88,
    },
    LtqRow {
        index: 83,
        freq_hz: 8750.00,
        bark_z: 21.760,
        ltq_db: 6.50,
    },
    LtqRow {
        index: 84,
        freq_hz: 9000.00,
        bark_z: 21.906,
        ltq_db: 7.19,
    },
    LtqRow {
        index: 85,
        freq_hz: 9250.00,
        bark_z: 22.046,
        ltq_db: 7.93,
    },
    LtqRow {
        index: 86,
        freq_hz: 9500.00,
        bark_z: 22.178,
        ltq_db: 8.75,
    },
    LtqRow {
        index: 87,
        freq_hz: 9750.00,
        bark_z: 22.304,
        ltq_db: 9.63,
    },
    LtqRow {
        index: 88,
        freq_hz: 10000.00,
        bark_z: 22.424,
        ltq_db: 10.58,
    },
    LtqRow {
        index: 89,
        freq_hz: 10250.00,
        bark_z: 22.538,
        ltq_db: 11.60,
    },
    LtqRow {
        index: 90,
        freq_hz: 10500.00,
        bark_z: 22.646,
        ltq_db: 12.71,
    },
    LtqRow {
        index: 91,
        freq_hz: 10750.00,
        bark_z: 22.749,
        ltq_db: 13.90,
    },
    LtqRow {
        index: 92,
        freq_hz: 11000.00,
        bark_z: 22.847,
        ltq_db: 15.18,
    },
    LtqRow {
        index: 93,
        freq_hz: 11250.00,
        bark_z: 22.941,
        ltq_db: 16.54,
    },
    LtqRow {
        index: 94,
        freq_hz: 11500.00,
        bark_z: 23.030,
        ltq_db: 18.01,
    },
    LtqRow {
        index: 95,
        freq_hz: 11750.00,
        bark_z: 23.114,
        ltq_db: 19.57,
    },
    LtqRow {
        index: 96,
        freq_hz: 12000.00,
        bark_z: 23.195,
        ltq_db: 21.23,
    },
    LtqRow {
        index: 97,
        freq_hz: 12250.00,
        bark_z: 23.272,
        ltq_db: 23.01,
    },
    LtqRow {
        index: 98,
        freq_hz: 12500.00,
        bark_z: 23.345,
        ltq_db: 24.90,
    },
    LtqRow {
        index: 99,
        freq_hz: 12750.00,
        bark_z: 23.415,
        ltq_db: 26.90,
    },
    LtqRow {
        index: 100,
        freq_hz: 13000.00,
        bark_z: 23.482,
        ltq_db: 29.03,
    },
    LtqRow {
        index: 101,
        freq_hz: 13250.00,
        bark_z: 23.546,
        ltq_db: 31.28,
    },
    LtqRow {
        index: 102,
        freq_hz: 13500.00,
        bark_z: 23.607,
        ltq_db: 33.67,
    },
    LtqRow {
        index: 103,
        freq_hz: 13750.00,
        bark_z: 23.666,
        ltq_db: 36.19,
    },
    LtqRow {
        index: 104,
        freq_hz: 14000.00,
        bark_z: 23.722,
        ltq_db: 38.86,
    },
    LtqRow {
        index: 105,
        freq_hz: 14250.00,
        bark_z: 23.775,
        ltq_db: 41.67,
    },
    LtqRow {
        index: 106,
        freq_hz: 14500.00,
        bark_z: 23.827,
        ltq_db: 44.63,
    },
    LtqRow {
        index: 107,
        freq_hz: 14750.00,
        bark_z: 23.876,
        ltq_db: 47.76,
    },
    LtqRow {
        index: 108,
        freq_hz: 15000.00,
        bark_z: 23.923,
        ltq_db: 51.04,
    },
];

/// Look up a row of Annex D **Table D.1a** (Layer I, Fs = 32 kHz)
/// by its 1-based index number `i`.
///
/// Returns `Some(row)` for every printed index `i ∈ 1..=108` (the
/// table is now transcribed in full from
/// `docs/audio/mp3/annex-d-table-D1a-threshold-32kHz.csv`) and
/// `None` for `i == 0` (the spec is 1-based) and for `i > 108`
/// (above the printed table). The lookup indexes directly: row `i`
/// lives at `LTQ_L1_32K[i - 1]`.
pub fn ltq_layer1_32k(i: u16) -> Option<LtqRow> {
    if i == 0 {
        return None;
    }
    LTQ_L1_32K.get((i - 1) as usize).copied()
}

/// Annex D Step 3 — per-line `LTq_used` for **Table D.1a** (Layer I,
/// Fs = 32 kHz): the tabulated threshold-in-quiet with the bit-rate
/// offset applied.
///
/// Composes [`ltq_layer1_32k`] with [`step3_apply_ltq_offset`]:
/// `LTq_used(i) = LTq_table(i) + ltq_offset_db(per-channel rate)` —
/// `−12 dB` for per-channel rates `>= 96` kbit/s, `0 dB` below.
/// Returns `None` exactly when [`ltq_layer1_32k`] does (1-based
/// underflow `i == 0`, or above the printed table `i > 108`).
pub fn ltq_layer1_32k_used(i: u16, bit_rate_per_channel_kbps: u32) -> Option<f64> {
    ltq_layer1_32k(i).map(|r| step3_apply_ltq_offset(r.ltq_db, bit_rate_per_channel_kbps))
}

/// Number of rows in the **Layer I** Table D.1b (Fs = 44,1 kHz)
/// printed in ISO/IEC 11172-3 (1993): `i = 1…106`. The 44,1 kHz
/// FFT-line grid is coarser per Bark than the 32 kHz grid, so the
/// printed table ends two rows short of the 108-row D.1a/D.1c length.
pub const LTQ_LAYER1_44K1_LEN: usize = 106;

/// Annex D Table D.1b — threshold in quiet (absolute threshold) for
/// **Layer I** at **Fs = 44,1 kHz**, the complete 106-row table
/// printed in ISO/IEC 11172-3 (1993).
///
/// Transcribed from the docs collaborator's text extraction
/// `docs/audio/mp3/annex-d-table-D1b-threshold-44k1Hz.csv`, capturing
/// all four printed columns (1-based index `i`, FFT-line frequency in
/// Hz, critical-band rate `z` in Bark, and the absolute-threshold
/// `ltq_db`). As with D.1a the threshold column is **non-monotonic**:
/// it falls to a minimum of `-4.98 dB` at `i = 39` (≈ 3,36 kHz),
/// then climbs steeply at both ends (25,87 dB at `i = 1`, saturating
/// at the 68,00 dB ceiling from `i = 95` onward). The companion
/// Layer I table D.1c (48 kHz) and the Layer II tables D.1d–f are
/// staged as CSVs but not transcribed here (44,1 kHz Layer I this
/// round).
pub const LTQ_L1_44K1: [LtqRow; LTQ_LAYER1_44K1_LEN] = [
    LtqRow {
        index: 1,
        freq_hz: 86.13,
        bark_z: 0.850,
        ltq_db: 25.87,
    },
    LtqRow {
        index: 2,
        freq_hz: 172.27,
        bark_z: 1.694,
        ltq_db: 14.85,
    },
    LtqRow {
        index: 3,
        freq_hz: 258.40,
        bark_z: 2.525,
        ltq_db: 10.72,
    },
    LtqRow {
        index: 4,
        freq_hz: 344.53,
        bark_z: 3.337,
        ltq_db: 8.50,
    },
    LtqRow {
        index: 5,
        freq_hz: 430.66,
        bark_z: 4.124,
        ltq_db: 7.10,
    },
    LtqRow {
        index: 6,
        freq_hz: 516.80,
        bark_z: 4.882,
        ltq_db: 6.11,
    },
    LtqRow {
        index: 7,
        freq_hz: 602.93,
        bark_z: 5.608,
        ltq_db: 5.37,
    },
    LtqRow {
        index: 8,
        freq_hz: 689.06,
        bark_z: 6.301,
        ltq_db: 4.79,
    },
    LtqRow {
        index: 9,
        freq_hz: 775.20,
        bark_z: 6.959,
        ltq_db: 4.32,
    },
    LtqRow {
        index: 10,
        freq_hz: 861.33,
        bark_z: 7.581,
        ltq_db: 3.92,
    },
    LtqRow {
        index: 11,
        freq_hz: 947.46,
        bark_z: 8.169,
        ltq_db: 3.57,
    },
    LtqRow {
        index: 12,
        freq_hz: 1033.59,
        bark_z: 8.723,
        ltq_db: 3.25,
    },
    LtqRow {
        index: 13,
        freq_hz: 1119.73,
        bark_z: 9.244,
        ltq_db: 2.95,
    },
    LtqRow {
        index: 14,
        freq_hz: 1205.86,
        bark_z: 9.734,
        ltq_db: 2.67,
    },
    LtqRow {
        index: 15,
        freq_hz: 1291.99,
        bark_z: 10.195,
        ltq_db: 2.39,
    },
    LtqRow {
        index: 16,
        freq_hz: 1378.13,
        bark_z: 10.629,
        ltq_db: 2.11,
    },
    LtqRow {
        index: 17,
        freq_hz: 1464.26,
        bark_z: 11.037,
        ltq_db: 1.83,
    },
    LtqRow {
        index: 18,
        freq_hz: 1550.39,
        bark_z: 11.421,
        ltq_db: 1.53,
    },
    LtqRow {
        index: 19,
        freq_hz: 1636.52,
        bark_z: 11.783,
        ltq_db: 1.23,
    },
    LtqRow {
        index: 20,
        freq_hz: 1722.66,
        bark_z: 12.125,
        ltq_db: 0.90,
    },
    LtqRow {
        index: 21,
        freq_hz: 1808.79,
        bark_z: 12.448,
        ltq_db: 0.56,
    },
    LtqRow {
        index: 22,
        freq_hz: 1894.92,
        bark_z: 12.753,
        ltq_db: 0.21,
    },
    LtqRow {
        index: 23,
        freq_hz: 1981.05,
        bark_z: 13.042,
        ltq_db: -0.17,
    },
    LtqRow {
        index: 24,
        freq_hz: 2067.19,
        bark_z: 13.317,
        ltq_db: -0.56,
    },
    LtqRow {
        index: 25,
        freq_hz: 2153.32,
        bark_z: 13.578,
        ltq_db: -0.96,
    },
    LtqRow {
        index: 26,
        freq_hz: 2239.45,
        bark_z: 13.826,
        ltq_db: -1.38,
    },
    LtqRow {
        index: 27,
        freq_hz: 2325.59,
        bark_z: 14.062,
        ltq_db: -1.79,
    },
    LtqRow {
        index: 28,
        freq_hz: 2411.72,
        bark_z: 14.288,
        ltq_db: -2.21,
    },
    LtqRow {
        index: 29,
        freq_hz: 2497.85,
        bark_z: 14.504,
        ltq_db: -2.63,
    },
    LtqRow {
        index: 30,
        freq_hz: 2583.98,
        bark_z: 14.711,
        ltq_db: -3.03,
    },
    LtqRow {
        index: 31,
        freq_hz: 2670.12,
        bark_z: 14.909,
        ltq_db: -3.41,
    },
    LtqRow {
        index: 32,
        freq_hz: 2756.25,
        bark_z: 15.100,
        ltq_db: -3.77,
    },
    LtqRow {
        index: 33,
        freq_hz: 2842.38,
        bark_z: 15.284,
        ltq_db: -4.09,
    },
    LtqRow {
        index: 34,
        freq_hz: 2928.52,
        bark_z: 15.460,
        ltq_db: -4.37,
    },
    LtqRow {
        index: 35,
        freq_hz: 3014.65,
        bark_z: 15.631,
        ltq_db: -4.60,
    },
    LtqRow {
        index: 36,
        freq_hz: 3100.78,
        bark_z: 15.796,
        ltq_db: -4.78,
    },
    LtqRow {
        index: 37,
        freq_hz: 3186.91,
        bark_z: 15.955,
        ltq_db: -4.91,
    },
    LtqRow {
        index: 38,
        freq_hz: 3273.05,
        bark_z: 16.110,
        ltq_db: -4.97,
    },
    LtqRow {
        index: 39,
        freq_hz: 3359.18,
        bark_z: 16.260,
        ltq_db: -4.98,
    },
    LtqRow {
        index: 40,
        freq_hz: 3445.31,
        bark_z: 16.406,
        ltq_db: -4.92,
    },
    LtqRow {
        index: 41,
        freq_hz: 3531.45,
        bark_z: 16.547,
        ltq_db: -4.81,
    },
    LtqRow {
        index: 42,
        freq_hz: 3617.58,
        bark_z: 16.685,
        ltq_db: -4.65,
    },
    LtqRow {
        index: 43,
        freq_hz: 3703.71,
        bark_z: 16.820,
        ltq_db: -4.43,
    },
    LtqRow {
        index: 44,
        freq_hz: 3789.84,
        bark_z: 16.951,
        ltq_db: -4.17,
    },
    LtqRow {
        index: 45,
        freq_hz: 3875.98,
        bark_z: 17.079,
        ltq_db: -3.87,
    },
    LtqRow {
        index: 46,
        freq_hz: 3962.11,
        bark_z: 17.205,
        ltq_db: -3.54,
    },
    LtqRow {
        index: 47,
        freq_hz: 4048.24,
        bark_z: 17.327,
        ltq_db: -3.19,
    },
    LtqRow {
        index: 48,
        freq_hz: 4134.38,
        bark_z: 17.447,
        ltq_db: -2.82,
    },
    LtqRow {
        index: 49,
        freq_hz: 4306.64,
        bark_z: 17.680,
        ltq_db: -2.06,
    },
    LtqRow {
        index: 50,
        freq_hz: 4478.91,
        bark_z: 17.905,
        ltq_db: -1.32,
    },
    LtqRow {
        index: 51,
        freq_hz: 4651.17,
        bark_z: 18.121,
        ltq_db: -0.64,
    },
    LtqRow {
        index: 52,
        freq_hz: 4823.44,
        bark_z: 18.331,
        ltq_db: -0.04,
    },
    LtqRow {
        index: 53,
        freq_hz: 4995.70,
        bark_z: 18.534,
        ltq_db: 0.47,
    },
    LtqRow {
        index: 54,
        freq_hz: 5167.97,
        bark_z: 18.731,
        ltq_db: 0.89,
    },
    LtqRow {
        index: 55,
        freq_hz: 5340.23,
        bark_z: 18.922,
        ltq_db: 1.23,
    },
    LtqRow {
        index: 56,
        freq_hz: 5512.50,
        bark_z: 19.108,
        ltq_db: 1.51,
    },
    LtqRow {
        index: 57,
        freq_hz: 5684.77,
        bark_z: 19.289,
        ltq_db: 1.74,
    },
    LtqRow {
        index: 58,
        freq_hz: 5857.03,
        bark_z: 19.464,
        ltq_db: 1.93,
    },
    LtqRow {
        index: 59,
        freq_hz: 6029.30,
        bark_z: 19.635,
        ltq_db: 2.11,
    },
    LtqRow {
        index: 60,
        freq_hz: 6201.56,
        bark_z: 19.801,
        ltq_db: 2.28,
    },
    LtqRow {
        index: 61,
        freq_hz: 6373.83,
        bark_z: 19.963,
        ltq_db: 2.46,
    },
    LtqRow {
        index: 62,
        freq_hz: 6546.09,
        bark_z: 20.120,
        ltq_db: 2.63,
    },
    LtqRow {
        index: 63,
        freq_hz: 6718.36,
        bark_z: 20.273,
        ltq_db: 2.82,
    },
    LtqRow {
        index: 64,
        freq_hz: 6890.63,
        bark_z: 20.421,
        ltq_db: 3.03,
    },
    LtqRow {
        index: 65,
        freq_hz: 7062.89,
        bark_z: 20.565,
        ltq_db: 3.25,
    },
    LtqRow {
        index: 66,
        freq_hz: 7235.16,
        bark_z: 20.705,
        ltq_db: 3.49,
    },
    LtqRow {
        index: 67,
        freq_hz: 7407.42,
        bark_z: 20.840,
        ltq_db: 3.74,
    },
    LtqRow {
        index: 68,
        freq_hz: 7579.69,
        bark_z: 20.972,
        ltq_db: 4.02,
    },
    LtqRow {
        index: 69,
        freq_hz: 7751.95,
        bark_z: 21.099,
        ltq_db: 4.32,
    },
    LtqRow {
        index: 70,
        freq_hz: 7924.22,
        bark_z: 21.222,
        ltq_db: 4.64,
    },
    LtqRow {
        index: 71,
        freq_hz: 8096.48,
        bark_z: 21.342,
        ltq_db: 4.98,
    },
    LtqRow {
        index: 72,
        freq_hz: 8268.75,
        bark_z: 21.457,
        ltq_db: 5.35,
    },
    LtqRow {
        index: 73,
        freq_hz: 8613.28,
        bark_z: 21.677,
        ltq_db: 6.15,
    },
    LtqRow {
        index: 74,
        freq_hz: 8957.81,
        bark_z: 21.882,
        ltq_db: 7.07,
    },
    LtqRow {
        index: 75,
        freq_hz: 9302.34,
        bark_z: 22.074,
        ltq_db: 8.10,
    },
    LtqRow {
        index: 76,
        freq_hz: 9646.88,
        bark_z: 22.253,
        ltq_db: 9.25,
    },
    LtqRow {
        index: 77,
        freq_hz: 9991.41,
        bark_z: 22.420,
        ltq_db: 10.54,
    },
    LtqRow {
        index: 78,
        freq_hz: 10335.94,
        bark_z: 22.576,
        ltq_db: 11.97,
    },
    LtqRow {
        index: 79,
        freq_hz: 10680.47,
        bark_z: 22.721,
        ltq_db: 13.56,
    },
    LtqRow {
        index: 80,
        freq_hz: 11025.00,
        bark_z: 22.857,
        ltq_db: 15.31,
    },
    LtqRow {
        index: 81,
        freq_hz: 11369.53,
        bark_z: 22.984,
        ltq_db: 17.23,
    },
    LtqRow {
        index: 82,
        freq_hz: 11714.06,
        bark_z: 23.102,
        ltq_db: 19.34,
    },
    LtqRow {
        index: 83,
        freq_hz: 12058.59,
        bark_z: 23.213,
        ltq_db: 21.64,
    },
    LtqRow {
        index: 84,
        freq_hz: 12403.13,
        bark_z: 23.317,
        ltq_db: 24.15,
    },
    LtqRow {
        index: 85,
        freq_hz: 12747.66,
        bark_z: 23.415,
        ltq_db: 26.88,
    },
    LtqRow {
        index: 86,
        freq_hz: 13092.19,
        bark_z: 23.506,
        ltq_db: 29.84,
    },
    LtqRow {
        index: 87,
        freq_hz: 13436.72,
        bark_z: 23.592,
        ltq_db: 33.05,
    },
    LtqRow {
        index: 88,
        freq_hz: 13781.25,
        bark_z: 23.673,
        ltq_db: 36.52,
    },
    LtqRow {
        index: 89,
        freq_hz: 14125.78,
        bark_z: 23.749,
        ltq_db: 40.25,
    },
    LtqRow {
        index: 90,
        freq_hz: 14470.31,
        bark_z: 23.821,
        ltq_db: 44.27,
    },
    LtqRow {
        index: 91,
        freq_hz: 14814.84,
        bark_z: 23.888,
        ltq_db: 48.59,
    },
    LtqRow {
        index: 92,
        freq_hz: 15159.38,
        bark_z: 23.952,
        ltq_db: 53.22,
    },
    LtqRow {
        index: 93,
        freq_hz: 15503.91,
        bark_z: 24.013,
        ltq_db: 58.18,
    },
    LtqRow {
        index: 94,
        freq_hz: 15848.44,
        bark_z: 24.070,
        ltq_db: 63.49,
    },
    LtqRow {
        index: 95,
        freq_hz: 16192.97,
        bark_z: 24.125,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 96,
        freq_hz: 16537.50,
        bark_z: 24.176,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 97,
        freq_hz: 16882.03,
        bark_z: 24.225,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 98,
        freq_hz: 17226.56,
        bark_z: 24.271,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 99,
        freq_hz: 17571.09,
        bark_z: 24.316,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 100,
        freq_hz: 17915.63,
        bark_z: 24.358,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 101,
        freq_hz: 18260.16,
        bark_z: 24.398,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 102,
        freq_hz: 18604.69,
        bark_z: 24.436,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 103,
        freq_hz: 18949.22,
        bark_z: 24.473,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 104,
        freq_hz: 19293.75,
        bark_z: 24.508,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 105,
        freq_hz: 19638.28,
        bark_z: 24.542,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 106,
        freq_hz: 19982.81,
        bark_z: 24.574,
        ltq_db: 68.00,
    },
];

/// Look up a row of Annex D **Table D.1b** (Layer I, Fs = 44,1 kHz)
/// by its 1-based index number `i`.
///
/// Returns `Some(row)` for every printed index `i ∈ 1..=106`
/// (transcribed in full from
/// `docs/audio/mp3/annex-d-table-D1b-threshold-44k1Hz.csv`) and
/// `None` for `i == 0` (the spec is 1-based) and for `i > 106`
/// (above the printed table). The lookup indexes directly: row `i`
/// lives at `LTQ_L1_44K1[i - 1]`.
pub fn ltq_layer1_44k1(i: u16) -> Option<LtqRow> {
    if i == 0 {
        return None;
    }
    LTQ_L1_44K1.get((i - 1) as usize).copied()
}

/// Annex D Step 3 — per-line `LTq_used` for **Table D.1b** (Layer I,
/// Fs = 44,1 kHz): the tabulated threshold-in-quiet with the bit-rate
/// offset applied.
///
/// Composes [`ltq_layer1_44k1`] with [`step3_apply_ltq_offset`]:
/// `LTq_used(i) = LTq_table(i) + ltq_offset_db(per-channel rate)` —
/// `−12 dB` for per-channel rates `>= 96` kbit/s, `0 dB` below.
/// Returns `None` exactly when [`ltq_layer1_44k1`] does (1-based
/// underflow `i == 0`, or above the printed table `i > 106`).
pub fn ltq_layer1_44k1_used(i: u16, bit_rate_per_channel_kbps: u32) -> Option<f64> {
    ltq_layer1_44k1(i).map(|r| step3_apply_ltq_offset(r.ltq_db, bit_rate_per_channel_kbps))
}

/// Number of rows in the **Layer I** Table D.1c (Fs = 48 kHz)
/// printed in ISO/IEC 11172-3 (1993): `i = 1…102`. The 48 kHz
/// FFT-line grid is the coarsest per Bark of the three Layer I rates,
/// so the printed table ends six rows short of the 108-row D.1a length
/// and four short of the 106-row D.1b length — the same 102-entry
/// length as the Layer I 48 kHz critical-band Table D.2c.
pub const LTQ_LAYER1_48K_LEN: usize = 102;

/// Annex D Table D.1c — threshold in quiet (absolute threshold) for
/// **Layer I** at **Fs = 48 kHz**, the complete 102-row table printed
/// in ISO/IEC 11172-3 (1993) PDF page 124 (printed 118).
///
/// Transcribed from the docs collaborator's text extraction
/// `docs/audio/mp3/annex-d-table-D1c-threshold-48kHz.csv`, capturing
/// all four printed columns (1-based index `i`, FFT-line frequency in
/// Hz, critical-band rate `z` in Bark, and the absolute-threshold
/// `ltq_db`), cross-checked against the render
/// `docs/audio/mp3/annex-d-renders/Table-D.1c-threshold-in-quiet-LayerI-48kHz-p118.png`.
/// As with D.1a/D.1b the threshold column is **non-monotonic**: it
/// falls to a minimum of `-4.98 dB` at `i = 35` (≈ 3,28 kHz), then
/// climbs steeply at both ends (24,17 dB at `i = 1`, saturating at the
/// 68,00 dB ceiling from `i = 91` onward). This completes the Layer I
/// D.1 family (D.1a 32 kHz / D.1b 44,1 kHz / D.1c 48 kHz); the Layer II
/// tables D.1d–f remain staged as CSVs.
pub const LTQ_L1_48K: [LtqRow; LTQ_LAYER1_48K_LEN] = [
    LtqRow {
        index: 1,
        freq_hz: 93.75,
        bark_z: 0.925,
        ltq_db: 24.17,
    },
    LtqRow {
        index: 2,
        freq_hz: 187.50,
        bark_z: 1.842,
        ltq_db: 13.87,
    },
    LtqRow {
        index: 3,
        freq_hz: 281.25,
        bark_z: 2.742,
        ltq_db: 10.01,
    },
    LtqRow {
        index: 4,
        freq_hz: 375.00,
        bark_z: 3.618,
        ltq_db: 7.94,
    },
    LtqRow {
        index: 5,
        freq_hz: 468.75,
        bark_z: 4.463,
        ltq_db: 6.62,
    },
    LtqRow {
        index: 6,
        freq_hz: 562.50,
        bark_z: 5.272,
        ltq_db: 5.70,
    },
    LtqRow {
        index: 7,
        freq_hz: 656.25,
        bark_z: 6.041,
        ltq_db: 5.00,
    },
    LtqRow {
        index: 8,
        freq_hz: 750.00,
        bark_z: 6.770,
        ltq_db: 4.45,
    },
    LtqRow {
        index: 9,
        freq_hz: 843.75,
        bark_z: 7.457,
        ltq_db: 4.00,
    },
    LtqRow {
        index: 10,
        freq_hz: 937.50,
        bark_z: 8.103,
        ltq_db: 3.61,
    },
    LtqRow {
        index: 11,
        freq_hz: 1031.25,
        bark_z: 8.708,
        ltq_db: 3.26,
    },
    LtqRow {
        index: 12,
        freq_hz: 1125.00,
        bark_z: 9.275,
        ltq_db: 2.93,
    },
    LtqRow {
        index: 13,
        freq_hz: 1218.75,
        bark_z: 9.805,
        ltq_db: 2.63,
    },
    LtqRow {
        index: 14,
        freq_hz: 1312.50,
        bark_z: 10.301,
        ltq_db: 2.32,
    },
    LtqRow {
        index: 15,
        freq_hz: 1406.25,
        bark_z: 10.765,
        ltq_db: 2.02,
    },
    LtqRow {
        index: 16,
        freq_hz: 1500.00,
        bark_z: 11.199,
        ltq_db: 1.71,
    },
    LtqRow {
        index: 17,
        freq_hz: 1593.75,
        bark_z: 11.606,
        ltq_db: 1.38,
    },
    LtqRow {
        index: 18,
        freq_hz: 1687.50,
        bark_z: 11.988,
        ltq_db: 1.04,
    },
    LtqRow {
        index: 19,
        freq_hz: 1781.25,
        bark_z: 12.347,
        ltq_db: 0.67,
    },
    LtqRow {
        index: 20,
        freq_hz: 1875.00,
        bark_z: 12.684,
        ltq_db: 0.29,
    },
    LtqRow {
        index: 21,
        freq_hz: 1968.75,
        bark_z: 13.002,
        ltq_db: -0.11,
    },
    LtqRow {
        index: 22,
        freq_hz: 2062.50,
        bark_z: 13.302,
        ltq_db: -0.54,
    },
    LtqRow {
        index: 23,
        freq_hz: 2156.25,
        bark_z: 13.586,
        ltq_db: -0.97,
    },
    LtqRow {
        index: 24,
        freq_hz: 2250.00,
        bark_z: 13.855,
        ltq_db: -1.43,
    },
    LtqRow {
        index: 25,
        freq_hz: 2343.75,
        bark_z: 14.111,
        ltq_db: -1.88,
    },
    LtqRow {
        index: 26,
        freq_hz: 2437.50,
        bark_z: 14.354,
        ltq_db: -2.34,
    },
    LtqRow {
        index: 27,
        freq_hz: 2531.25,
        bark_z: 14.585,
        ltq_db: -2.79,
    },
    LtqRow {
        index: 28,
        freq_hz: 2625.00,
        bark_z: 14.807,
        ltq_db: -3.22,
    },
    LtqRow {
        index: 29,
        freq_hz: 2718.75,
        bark_z: 15.018,
        ltq_db: -3.62,
    },
    LtqRow {
        index: 30,
        freq_hz: 2812.50,
        bark_z: 15.221,
        ltq_db: -3.98,
    },
    LtqRow {
        index: 31,
        freq_hz: 2906.25,
        bark_z: 15.415,
        ltq_db: -4.30,
    },
    LtqRow {
        index: 32,
        freq_hz: 3000.00,
        bark_z: 15.602,
        ltq_db: -4.57,
    },
    LtqRow {
        index: 33,
        freq_hz: 3093.75,
        bark_z: 15.783,
        ltq_db: -4.77,
    },
    LtqRow {
        index: 34,
        freq_hz: 3187.50,
        bark_z: 15.956,
        ltq_db: -4.91,
    },
    LtqRow {
        index: 35,
        freq_hz: 3281.25,
        bark_z: 16.124,
        ltq_db: -4.98,
    },
    LtqRow {
        index: 36,
        freq_hz: 3375.00,
        bark_z: 16.287,
        ltq_db: -4.97,
    },
    LtqRow {
        index: 37,
        freq_hz: 3468.75,
        bark_z: 16.445,
        ltq_db: -4.90,
    },
    LtqRow {
        index: 38,
        freq_hz: 3562.50,
        bark_z: 16.598,
        ltq_db: -4.76,
    },
    LtqRow {
        index: 39,
        freq_hz: 3656.25,
        bark_z: 16.746,
        ltq_db: -4.55,
    },
    LtqRow {
        index: 40,
        freq_hz: 3750.00,
        bark_z: 16.891,
        ltq_db: -4.29,
    },
    LtqRow {
        index: 41,
        freq_hz: 3843.75,
        bark_z: 17.032,
        ltq_db: -3.99,
    },
    LtqRow {
        index: 42,
        freq_hz: 3937.50,
        bark_z: 17.169,
        ltq_db: -3.64,
    },
    LtqRow {
        index: 43,
        freq_hz: 4031.25,
        bark_z: 17.303,
        ltq_db: -3.26,
    },
    LtqRow {
        index: 44,
        freq_hz: 4125.00,
        bark_z: 17.434,
        ltq_db: -2.86,
    },
    LtqRow {
        index: 45,
        freq_hz: 4218.75,
        bark_z: 17.563,
        ltq_db: -2.45,
    },
    LtqRow {
        index: 46,
        freq_hz: 4312.50,
        bark_z: 17.688,
        ltq_db: -2.04,
    },
    LtqRow {
        index: 47,
        freq_hz: 4406.25,
        bark_z: 17.811,
        ltq_db: -1.63,
    },
    LtqRow {
        index: 48,
        freq_hz: 4500.00,
        bark_z: 17.932,
        ltq_db: -1.24,
    },
    LtqRow {
        index: 49,
        freq_hz: 4687.50,
        bark_z: 18.166,
        ltq_db: -0.51,
    },
    LtqRow {
        index: 50,
        freq_hz: 4875.00,
        bark_z: 18.392,
        ltq_db: 0.12,
    },
    LtqRow {
        index: 51,
        freq_hz: 5062.50,
        bark_z: 18.611,
        ltq_db: 0.64,
    },
    LtqRow {
        index: 52,
        freq_hz: 5250.00,
        bark_z: 18.823,
        ltq_db: 1.06,
    },
    LtqRow {
        index: 53,
        freq_hz: 5437.50,
        bark_z: 19.028,
        ltq_db: 1.39,
    },
    LtqRow {
        index: 54,
        freq_hz: 5625.00,
        bark_z: 19.226,
        ltq_db: 1.66,
    },
    LtqRow {
        index: 55,
        freq_hz: 5812.50,
        bark_z: 19.419,
        ltq_db: 1.88,
    },
    LtqRow {
        index: 56,
        freq_hz: 6000.00,
        bark_z: 19.606,
        ltq_db: 2.08,
    },
    LtqRow {
        index: 57,
        freq_hz: 6187.50,
        bark_z: 19.788,
        ltq_db: 2.27,
    },
    LtqRow {
        index: 58,
        freq_hz: 6375.00,
        bark_z: 19.964,
        ltq_db: 2.46,
    },
    LtqRow {
        index: 59,
        freq_hz: 6562.50,
        bark_z: 20.135,
        ltq_db: 2.65,
    },
    LtqRow {
        index: 60,
        freq_hz: 6750.00,
        bark_z: 20.300,
        ltq_db: 2.86,
    },
    LtqRow {
        index: 61,
        freq_hz: 6937.50,
        bark_z: 20.461,
        ltq_db: 3.09,
    },
    LtqRow {
        index: 62,
        freq_hz: 7125.00,
        bark_z: 20.616,
        ltq_db: 3.33,
    },
    LtqRow {
        index: 63,
        freq_hz: 7312.50,
        bark_z: 20.766,
        ltq_db: 3.60,
    },
    LtqRow {
        index: 64,
        freq_hz: 7500.00,
        bark_z: 20.912,
        ltq_db: 3.89,
    },
    LtqRow {
        index: 65,
        freq_hz: 7687.50,
        bark_z: 21.052,
        ltq_db: 4.20,
    },
    LtqRow {
        index: 66,
        freq_hz: 7875.00,
        bark_z: 21.188,
        ltq_db: 4.54,
    },
    LtqRow {
        index: 67,
        freq_hz: 8062.50,
        bark_z: 21.318,
        ltq_db: 4.91,
    },
    LtqRow {
        index: 68,
        freq_hz: 8250.00,
        bark_z: 21.445,
        ltq_db: 5.31,
    },
    LtqRow {
        index: 69,
        freq_hz: 8437.50,
        bark_z: 21.567,
        ltq_db: 5.73,
    },
    LtqRow {
        index: 70,
        freq_hz: 8625.00,
        bark_z: 21.684,
        ltq_db: 6.18,
    },
    LtqRow {
        index: 71,
        freq_hz: 8812.50,
        bark_z: 21.797,
        ltq_db: 6.67,
    },
    LtqRow {
        index: 72,
        freq_hz: 9000.00,
        bark_z: 21.906,
        ltq_db: 7.19,
    },
    LtqRow {
        index: 73,
        freq_hz: 9375.00,
        bark_z: 22.113,
        ltq_db: 8.33,
    },
    LtqRow {
        index: 74,
        freq_hz: 9750.00,
        bark_z: 22.304,
        ltq_db: 9.63,
    },
    LtqRow {
        index: 75,
        freq_hz: 10125.00,
        bark_z: 22.482,
        ltq_db: 11.08,
    },
    LtqRow {
        index: 76,
        freq_hz: 10500.00,
        bark_z: 22.646,
        ltq_db: 12.71,
    },
    LtqRow {
        index: 77,
        freq_hz: 10875.00,
        bark_z: 22.799,
        ltq_db: 14.53,
    },
    LtqRow {
        index: 78,
        freq_hz: 11250.00,
        bark_z: 22.941,
        ltq_db: 16.54,
    },
    LtqRow {
        index: 79,
        freq_hz: 11625.00,
        bark_z: 23.072,
        ltq_db: 18.77,
    },
    LtqRow {
        index: 80,
        freq_hz: 12000.00,
        bark_z: 23.195,
        ltq_db: 21.23,
    },
    LtqRow {
        index: 81,
        freq_hz: 12375.00,
        bark_z: 23.309,
        ltq_db: 23.94,
    },
    LtqRow {
        index: 82,
        freq_hz: 12750.00,
        bark_z: 23.415,
        ltq_db: 26.90,
    },
    LtqRow {
        index: 83,
        freq_hz: 13125.00,
        bark_z: 23.515,
        ltq_db: 30.14,
    },
    LtqRow {
        index: 84,
        freq_hz: 13500.00,
        bark_z: 23.607,
        ltq_db: 33.67,
    },
    LtqRow {
        index: 85,
        freq_hz: 13875.00,
        bark_z: 23.694,
        ltq_db: 37.51,
    },
    LtqRow {
        index: 86,
        freq_hz: 14250.00,
        bark_z: 23.775,
        ltq_db: 41.67,
    },
    LtqRow {
        index: 87,
        freq_hz: 14625.00,
        bark_z: 23.852,
        ltq_db: 46.17,
    },
    LtqRow {
        index: 88,
        freq_hz: 15000.00,
        bark_z: 23.923,
        ltq_db: 51.04,
    },
    LtqRow {
        index: 89,
        freq_hz: 15375.00,
        bark_z: 23.991,
        ltq_db: 56.29,
    },
    LtqRow {
        index: 90,
        freq_hz: 15750.00,
        bark_z: 24.054,
        ltq_db: 61.94,
    },
    LtqRow {
        index: 91,
        freq_hz: 16125.00,
        bark_z: 24.114,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 92,
        freq_hz: 16500.00,
        bark_z: 24.171,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 93,
        freq_hz: 16875.00,
        bark_z: 24.224,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 94,
        freq_hz: 17250.00,
        bark_z: 24.275,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 95,
        freq_hz: 17625.00,
        bark_z: 24.322,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 96,
        freq_hz: 18000.00,
        bark_z: 24.368,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 97,
        freq_hz: 18375.00,
        bark_z: 24.411,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 98,
        freq_hz: 18750.00,
        bark_z: 24.452,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 99,
        freq_hz: 19125.00,
        bark_z: 24.491,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 100,
        freq_hz: 19500.00,
        bark_z: 24.528,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 101,
        freq_hz: 19875.00,
        bark_z: 24.564,
        ltq_db: 68.00,
    },
    LtqRow {
        index: 102,
        freq_hz: 20250.00,
        bark_z: 24.597,
        ltq_db: 68.00,
    },
];

/// Look up a row of Annex D **Table D.1c** (Layer I, Fs = 48 kHz)
/// by its 1-based index number `i`.
///
/// Returns `Some(row)` for every printed index `i ∈ 1..=102`
/// (transcribed in full from
/// `docs/audio/mp3/annex-d-table-D1c-threshold-48kHz.csv`) and
/// `None` for `i == 0` (the spec is 1-based) and for `i > 102`
/// (above the printed table). The lookup indexes directly: row `i`
/// lives at `LTQ_L1_48K[i - 1]`.
pub fn ltq_layer1_48k(i: u16) -> Option<LtqRow> {
    if i == 0 {
        return None;
    }
    LTQ_L1_48K.get((i - 1) as usize).copied()
}

/// Annex D Step 3 — per-line `LTq_used` for **Table D.1c** (Layer I,
/// Fs = 48 kHz): the tabulated threshold-in-quiet with the bit-rate
/// offset applied.
///
/// Composes [`ltq_layer1_48k`] with [`step3_apply_ltq_offset`]:
/// `LTq_used(i) = LTq_table(i) + ltq_offset_db(per-channel rate)` —
/// `−12 dB` for per-channel rates `>= 96` kbit/s, `0 dB` below.
/// Returns `None` exactly when [`ltq_layer1_48k`] does (1-based
/// underflow `i == 0`, or above the printed table `i > 102`).
pub fn ltq_layer1_48k_used(i: u16, bit_rate_per_channel_kbps: u32) -> Option<f64> {
    ltq_layer1_48k(i).map(|r| step3_apply_ltq_offset(r.ltq_db, bit_rate_per_channel_kbps))
}

// -----------------------------------------------------------------
// Annex D clause D.2.3 — Model 2 partition-domain spreading operator
// at Fs = 32 kHz.
//
// The per-pair spreading function `model2_sprdngf(j, i)` (above)
// gives the power-domain weight with which a unit of energy located
// at the Bark value `i` (the masker / source) spreads onto the Bark
// value `j` (the band spread into / destination). Model 2 evaluates
// that function over the **calculation partitions** of Table D.3a —
// each partition `b` carries a representative (median) Bark value
// `bval[b]` (the third column of the table). The spreading operator
// is therefore the `bmax × bmax` matrix
//
//     sprdngf[d][s] = model2_sprdngf(bval[d], bval[s])
//
// (`d` = destination partition, `s` = source partition), and the
// energy spread into partition `d` is `Σ_s e[s] · sprdngf[d][s]`.
//
// Clause D.2.3 further normalises the spread so the operator
// conserves total energy: `rnorm[s] = 1 / Σ_d sprdngf[d][s]` scales
// each source column, equivalently the normalised destination value
// is divided by the column sum of the contributing source. The
// per-partition `bval` data needed for this composition is the
// complete 49-row `CALC_PARTITION_32K` (Table D.3a), so the 32 kHz
// operator is fully derivable in tree; the 44,1 / 48 kHz operators
// stay DOCS-GAP until Tables D.3b / D.3c are transcribed off their
// PNG renders.
// -----------------------------------------------------------------

/// Number of Model 2 calculation partitions at Fs = 32 kHz
/// (`bmax = 49`, the length of Table D.3a / [`CALC_PARTITION_32K`]).
pub const MODEL2_PARTITIONS_32K: usize = CALC_PARTITION_32K.len();

/// Model 2 per-pair partition spreading weight at Fs = 32 kHz: the
/// power-domain weight with which energy in source partition
/// `from_partition` spreads onto destination partition
/// `into_partition`.
///
/// Both arguments are **1-based** partition indices into Table D.3a
/// (`1..=49`), matching [`calc_partition_32k`]. The weight is
/// `model2_sprdngf(bval[into], bval[from])` — i.e. the destination
/// partition's median Bark is the `j` (spread-into) argument and the
/// source partition's median Bark is the `i` (masker) argument. The
/// result is a non-negative power weight `≈ 1` on the diagonal
/// (`from == into`), decaying away from it and cut off entirely
/// beyond the `model2_sprdngf` `tmpy < −100` window.
///
/// Returns `None` if either index is `0` (the table is 1-based) or
/// greater than [`MODEL2_PARTITIONS_32K`].
pub fn model2_spread_weight_32k(into_partition: u16, from_partition: u16) -> Option<f64> {
    let into = calc_partition_32k(into_partition)?;
    let from = calc_partition_32k(from_partition)?;
    Some(model2_sprdngf(into.bval, from.bval))
}

/// The complete Model 2 partition-domain spreading matrix at
/// Fs = 32 kHz: `matrix[d][s]` is the spreading weight from source
/// partition `s` onto destination partition `d`, both 0-based here
/// (the returned matrix is dense over all
/// [`MODEL2_PARTITIONS_32K`] × [`MODEL2_PARTITIONS_32K`] partitions).
///
/// `matrix[d][s] == model2_sprdngf(bval[d], bval[s])` where
/// `bval[k] == CALC_PARTITION_32K[k].bval`. Each row `d` collects
/// every source partition's contribution into destination `d`; each
/// column `s` is source partition `s` spread across all
/// destinations. The matrix is **not** symmetric — the spreading
/// function is asymmetric in Bark (steeper below the masker than
/// above) — but every entry is a non-negative power weight that
/// never exceeds the on-diagonal value.
pub fn model2_spreading_matrix_32k() -> Vec<Vec<f64>> {
    let n = MODEL2_PARTITIONS_32K;
    let mut matrix = vec![vec![0.0f64; n]; n];
    for (d, row) in matrix.iter_mut().enumerate() {
        let bval_d = CALC_PARTITION_32K[d].bval;
        for (s, cell) in row.iter_mut().enumerate() {
            let bval_s = CALC_PARTITION_32K[s].bval;
            *cell = model2_sprdngf(bval_d, bval_s);
        }
    }
    matrix
}

/// Model 2 spreading normalisation factors at Fs = 32 kHz
/// (clause D.2.3 `rnorm`): for each **source** partition `s`,
/// `rnorm[s] = 1 / Σ_d sprdngf[d][s]` — the reciprocal of the total
/// power the source spreads across every destination partition.
///
/// Multiplying source-partition energy by `rnorm[s]` before
/// spreading makes the operator energy-conserving (the spread total
/// over all destinations equals the source energy). The returned
/// vector is indexed 0-based by source partition over all
/// [`MODEL2_PARTITIONS_32K`] partitions. Every column sum of
/// [`model2_spreading_matrix_32k`] is strictly positive (the
/// diagonal entry alone is `≈ 1`), so each factor is finite and
/// strictly positive.
pub fn model2_spread_normalization_32k() -> Vec<f64> {
    let matrix = model2_spreading_matrix_32k();
    let n = MODEL2_PARTITIONS_32K;
    (0..n)
        .map(|s| {
            let col_sum: f64 = (0..n).map(|d| matrix[d][s]).sum();
            1.0 / col_sum
        })
        .collect()
}

/// Apply the clause D.2.3 energy-conserving spreading operator at
/// Fs = 32 kHz to a per-partition source-energy vector, producing the
/// spread excitation `eb(b)` per destination partition `b`.
///
/// `partition_energy[s]` is the (linear power) energy accumulated into
/// source partition `s`, indexed 0-based over the
/// [`MODEL2_PARTITIONS_32K`] calculation partitions of Table D.3a. The
/// returned vector `eb[d]` is the same length and carries the spread
/// excitation
///
/// ```text
/// eb[d] = Σ_s  (partition_energy[s] · rnorm[s]) · sprdngf[d][s]
/// ```
///
/// — i.e. each source's energy is first scaled by its clause D.2.3
/// `rnorm[s]` energy-conservation factor (so a unit source impulse
/// spreads to a total of exactly 1 over all destinations, see
/// [`model2_spread_normalization_32k`]) and then distributed across
/// destinations through the asymmetric Bark spreading weights of
/// [`model2_spreading_matrix_32k`]. This is the matrix–vector product
/// the eventual Model 2 threshold step consumes as its spread
/// excitation; it does **not** yet apply the per-partition `minval`
/// floor or the TMN/NMT tonality offset (clause D.2.4 combination
/// rule), which remain a DOCS-GAP — the noise-masking-tone offset and
/// the tonality-index blend are not transcribed in the staged Annex D
/// extract.
///
/// Returns `None` if `partition_energy.len() != MODEL2_PARTITIONS_32K`.
/// Because the operator is energy-conserving, the spread total
/// `Σ_d eb[d]` equals the source total `Σ_s partition_energy[s]` (up
/// to floating-point rounding), so the operator redistributes energy
/// across partitions without creating or destroying it.
pub fn model2_spread_energy_32k(partition_energy: &[f64]) -> Option<Vec<f64>> {
    let n = MODEL2_PARTITIONS_32K;
    if partition_energy.len() != n {
        return None;
    }
    let matrix = model2_spreading_matrix_32k();
    let rnorm = model2_spread_normalization_32k();
    // Pre-scale each source by its energy-conservation factor once,
    // then accumulate into every destination.
    let scaled: Vec<f64> = (0..n).map(|s| partition_energy[s] * rnorm[s]).collect();
    let eb = (0..n)
        .map(|d| (0..n).map(|s| scaled[s] * matrix[d][s]).sum())
        .collect();
    Some(eb)
}

// -----------------------------------------------------------------
// Annex D clause D.2.3 — Model 2 partition-domain spreading operator
// at Fs = 44,1 kHz and Fs = 48 kHz.
//
// Identical construction to the 32 kHz operator above, parameterised
// over the calculation-partition table of the target rate (Table D.3b
// `bmax = 57` at 44,1 kHz, Table D.3c `bmax = 58` at 48 kHz). Each
// partition `b` carries its representative median Bark value `bval[b]`,
// the spreading matrix is `sprdngf[d][s] = model2_sprdngf(bval[d],
// bval[s])`, and the clause D.2.3 `rnorm[s] = 1 / Σ_d sprdngf[d][s]`
// energy-conservation factor scales each source column before
// spreading. The `model2_sprdngf` backbone is rate-independent (it is
// defined purely over Bark differences), so only the per-partition
// `bval` vector changes between rates.
// -----------------------------------------------------------------

/// Number of Model 2 calculation partitions at Fs = 44,1 kHz
/// (`bmax = 57`, the length of Table D.3b / [`CALC_PARTITION_44K1`]).
pub const MODEL2_PARTITIONS_44K1: usize = CALC_PARTITION_44K1.len();

/// Number of Model 2 calculation partitions at Fs = 48 kHz
/// (`bmax = 58`, the length of Table D.3c / [`CALC_PARTITION_48K`]).
pub const MODEL2_PARTITIONS_48K: usize = CALC_PARTITION_48K.len();

/// The complete Model 2 partition-domain spreading matrix for a given
/// calculation-partition table: `matrix[d][s]` is the spreading weight
/// from source partition `s` onto destination partition `d`, both
/// 0-based, dense over `bvals.len() × bvals.len()`.
///
/// `matrix[d][s] == model2_sprdngf(bvals[d], bvals[s])`. Shared
/// machinery behind the per-rate operators; `bvals` is the median-Bark
/// column of the rate's Table D.3x.
fn model2_spreading_matrix_from_bvals(bvals: &[f64]) -> Vec<Vec<f64>> {
    let n = bvals.len();
    let mut matrix = vec![vec![0.0f64; n]; n];
    for (d, row) in matrix.iter_mut().enumerate() {
        for (s, cell) in row.iter_mut().enumerate() {
            *cell = model2_sprdngf(bvals[d], bvals[s]);
        }
    }
    matrix
}

/// Apply the clause D.2.3 energy-conserving spreading operator to a
/// per-partition source-energy vector, for an arbitrary
/// calculation-partition `bval` column.
///
/// Shared implementation behind [`model2_spread_energy_44k1`] and
/// [`model2_spread_energy_48k`]: pre-scales each source by its
/// `rnorm[s] = 1 / Σ_d sprdngf[d][s]` factor, then accumulates into
/// every destination through the asymmetric Bark spreading weights.
/// Returns `None` if `partition_energy.len() != bvals.len()`.
fn model2_spread_energy_from_bvals(bvals: &[f64], partition_energy: &[f64]) -> Option<Vec<f64>> {
    let n = bvals.len();
    if partition_energy.len() != n {
        return None;
    }
    let matrix = model2_spreading_matrix_from_bvals(bvals);
    let rnorm: Vec<f64> = (0..n)
        .map(|s| {
            let col_sum: f64 = (0..n).map(|d| matrix[d][s]).sum();
            1.0 / col_sum
        })
        .collect();
    let scaled: Vec<f64> = (0..n).map(|s| partition_energy[s] * rnorm[s]).collect();
    let eb = (0..n)
        .map(|d| (0..n).map(|s| scaled[s] * matrix[d][s]).sum())
        .collect();
    Some(eb)
}

/// Model 2 per-pair partition spreading weight at Fs = 44,1 kHz:
/// `model2_sprdngf(bval[into], bval[from])` over Table D.3b. Both
/// arguments are 1-based partition indices `1..=57`; `None` outside.
pub fn model2_spread_weight_44k1(into_partition: u16, from_partition: u16) -> Option<f64> {
    let into = calc_partition_44k1(into_partition)?;
    let from = calc_partition_44k1(from_partition)?;
    Some(model2_sprdngf(into.bval, from.bval))
}

/// Model 2 per-pair partition spreading weight at Fs = 48 kHz:
/// `model2_sprdngf(bval[into], bval[from])` over Table D.3c. Both
/// arguments are 1-based partition indices `1..=58`; `None` outside.
pub fn model2_spread_weight_48k(into_partition: u16, from_partition: u16) -> Option<f64> {
    let into = calc_partition_48k(into_partition)?;
    let from = calc_partition_48k(from_partition)?;
    Some(model2_sprdngf(into.bval, from.bval))
}

/// The complete Model 2 partition-domain spreading matrix at
/// Fs = 44,1 kHz (Table D.3b, `bmax = 57`). See
/// [`model2_spreading_matrix_32k`] for the matrix semantics.
pub fn model2_spreading_matrix_44k1() -> Vec<Vec<f64>> {
    let bvals: Vec<f64> = CALC_PARTITION_44K1.iter().map(|p| p.bval).collect();
    model2_spreading_matrix_from_bvals(&bvals)
}

/// The complete Model 2 partition-domain spreading matrix at
/// Fs = 48 kHz (Table D.3c, `bmax = 58`).
pub fn model2_spreading_matrix_48k() -> Vec<Vec<f64>> {
    let bvals: Vec<f64> = CALC_PARTITION_48K.iter().map(|p| p.bval).collect();
    model2_spreading_matrix_from_bvals(&bvals)
}

/// Model 2 spreading normalisation factors at Fs = 44,1 kHz
/// (clause D.2.3 `rnorm`), one per source partition. See
/// [`model2_spread_normalization_32k`].
pub fn model2_spread_normalization_44k1() -> Vec<f64> {
    let matrix = model2_spreading_matrix_44k1();
    let n = MODEL2_PARTITIONS_44K1;
    (0..n)
        .map(|s| 1.0 / (0..n).map(|d| matrix[d][s]).sum::<f64>())
        .collect()
}

/// Model 2 spreading normalisation factors at Fs = 48 kHz
/// (clause D.2.3 `rnorm`), one per source partition.
pub fn model2_spread_normalization_48k() -> Vec<f64> {
    let matrix = model2_spreading_matrix_48k();
    let n = MODEL2_PARTITIONS_48K;
    (0..n)
        .map(|s| 1.0 / (0..n).map(|d| matrix[d][s]).sum::<f64>())
        .collect()
}

/// Apply the clause D.2.3 energy-conserving spreading operator at
/// Fs = 44,1 kHz (Table D.3b, `bmax = 57`) to a per-partition
/// source-energy vector. See [`model2_spread_energy_32k`] for the
/// operator semantics; `None` if the length is not
/// [`MODEL2_PARTITIONS_44K1`].
pub fn model2_spread_energy_44k1(partition_energy: &[f64]) -> Option<Vec<f64>> {
    let bvals: Vec<f64> = CALC_PARTITION_44K1.iter().map(|p| p.bval).collect();
    model2_spread_energy_from_bvals(&bvals, partition_energy)
}

/// Apply the clause D.2.3 energy-conserving spreading operator at
/// Fs = 48 kHz (Table D.3c, `bmax = 58`) to a per-partition
/// source-energy vector. `None` if the length is not
/// [`MODEL2_PARTITIONS_48K`].
pub fn model2_spread_energy_48k(partition_energy: &[f64]) -> Option<Vec<f64>> {
    let bvals: Vec<f64> = CALC_PARTITION_48K.iter().map(|p| p.bval).collect();
    model2_spread_energy_from_bvals(&bvals, partition_energy)
}

// -----------------------------------------------------------------
// Annex D Tables D.4a–c — Model 2 absolute-threshold table (per FFT
// line), clause D.2.
//
// Each table maps a *range* of FFT lines of the 1024-point Model 2
// analysis FFT to a single absolute-threshold value: the row
// `(line_low, line_high, absthr_db)` says every FFT line `i` with
// `line_low <= i <= line_high` shares the threshold `absthr_db`. The
// ranges tile the FFT lines contiguously (`line_low_{n+1} =
// line_high_n + 1`) from line 1, so a per-line lookup is a search for
// the unique covering range. Per the printed note "a value of 0 dB
// represents a level in the absolute-threshold calculation of 96 dB
// below the energy of a sine wave of amplitude +-32 760."
//
// The three tables share their `absthr` column entry-for-entry with
// the Layer II D.1 thresholds at the same rate (D.4a↔D.1d, D.4b↔D.1e,
// D.4c↔D.1f) except for the documented last-digit / ceiling
// divergences captured in the staged extract: D.4a's last range prints
// 51,03 dB; D.4b runs 0,01 dB low over a top stretch and caps at the
// distinctive 69,13 dB ceiling; D.4c matches its D.1f twin at the
// 68,00 dB ceiling. The CSVs (and these consts) carry the as-printed
// D.4 values.
// -----------------------------------------------------------------

/// One row of an Annex D Table D.4x (Model 2 per-FFT-line absolute
/// threshold): the inclusive FFT-line range `[line_low, line_high]`
/// that shares a single absolute-threshold value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AbsThrRange {
    /// Lower FFT-line bound (1-based, inclusive).
    pub line_low: u16,
    /// Upper FFT-line bound (1-based, inclusive).
    pub line_high: u16,
    /// Absolute threshold (threshold in quiet) in dB for this range.
    pub absthr_db: f64,
}

impl AbsThrRange {
    /// Number of FFT lines this range covers, `line_high - line_low + 1`.
    pub fn width(self) -> u16 {
        self.line_high - self.line_low + 1
    }
}

/// Binary-search a contiguous-tiling Table D.4x for the row covering
/// FFT line `line` (1-based), returning its absolute threshold in dB.
///
/// Returns `None` for `line == 0` and for any line above the table's
/// last covered FFT line. Shared lookup behind the per-rate accessors.
fn absthr_for_line_in(table: &[AbsThrRange], line: u16) -> Option<f64> {
    if line == 0 {
        return None;
    }
    let idx = table
        .binary_search_by(|r| {
            if line < r.line_low {
                core::cmp::Ordering::Greater
            } else if line > r.line_high {
                core::cmp::Ordering::Less
            } else {
                core::cmp::Ordering::Equal
            }
        })
        .ok()?;
    Some(table[idx].absthr_db)
}

/// Annex D Table D.4a — Model 2 per-FFT-line absolute-threshold table
/// at **Fs = 32 kHz**, all 132 line-ranges printed in ISO/IEC 11172-3
/// (1993) PDF page 142 (printed 136).
///
/// Transcribed from `docs/audio/mp3/annex-d-table-D4a-absolute-threshold-32kHz.csv`
/// (columns `index_lower, index_higher, absthr_dB`), cross-checked
/// against the render `…annex-d-renders/Table-D.4a-absolute-threshold-32kHz-p136.png`.
/// The ranges tile FFT lines 1…480 contiguously. The threshold column
/// reaches a minimum of `-4.97 dB` (lines 105–108) and the printed
/// final range (lines 473–480) carries 51,03 dB (the documented
/// last-digit divergence from the D.1d twin's 51,04 dB).
// The line-8 threshold `6.28 dB` is a spec table value, not an
// approximation of `TAU` (6.283…); silence clippy's approx_constant.
#[allow(clippy::approx_constant)]
pub const ABSTHR_D4A_32K: [AbsThrRange; 132] = [
    AbsThrRange {
        line_low: 1,
        line_high: 1,
        absthr_db: 58.23,
    },
    AbsThrRange {
        line_low: 2,
        line_high: 2,
        absthr_db: 33.44,
    },
    AbsThrRange {
        line_low: 3,
        line_high: 3,
        absthr_db: 24.17,
    },
    AbsThrRange {
        line_low: 4,
        line_high: 4,
        absthr_db: 19.20,
    },
    AbsThrRange {
        line_low: 5,
        line_high: 5,
        absthr_db: 16.05,
    },
    AbsThrRange {
        line_low: 6,
        line_high: 6,
        absthr_db: 13.87,
    },
    AbsThrRange {
        line_low: 7,
        line_high: 7,
        absthr_db: 12.26,
    },
    AbsThrRange {
        line_low: 8,
        line_high: 8,
        absthr_db: 11.01,
    },
    AbsThrRange {
        line_low: 9,
        line_high: 9,
        absthr_db: 10.01,
    },
    AbsThrRange {
        line_low: 10,
        line_high: 10,
        absthr_db: 9.20,
    },
    AbsThrRange {
        line_low: 11,
        line_high: 11,
        absthr_db: 8.52,
    },
    AbsThrRange {
        line_low: 12,
        line_high: 12,
        absthr_db: 7.94,
    },
    AbsThrRange {
        line_low: 13,
        line_high: 13,
        absthr_db: 7.44,
    },
    AbsThrRange {
        line_low: 14,
        line_high: 14,
        absthr_db: 7.00,
    },
    AbsThrRange {
        line_low: 15,
        line_high: 15,
        absthr_db: 6.62,
    },
    AbsThrRange {
        line_low: 16,
        line_high: 16,
        absthr_db: 6.28,
    },
    AbsThrRange {
        line_low: 17,
        line_high: 17,
        absthr_db: 5.97,
    },
    AbsThrRange {
        line_low: 18,
        line_high: 18,
        absthr_db: 5.70,
    },
    AbsThrRange {
        line_low: 19,
        line_high: 19,
        absthr_db: 5.44,
    },
    AbsThrRange {
        line_low: 20,
        line_high: 20,
        absthr_db: 5.21,
    },
    AbsThrRange {
        line_low: 21,
        line_high: 21,
        absthr_db: 5.00,
    },
    AbsThrRange {
        line_low: 22,
        line_high: 22,
        absthr_db: 4.80,
    },
    AbsThrRange {
        line_low: 23,
        line_high: 23,
        absthr_db: 4.62,
    },
    AbsThrRange {
        line_low: 24,
        line_high: 24,
        absthr_db: 4.45,
    },
    AbsThrRange {
        line_low: 25,
        line_high: 25,
        absthr_db: 4.29,
    },
    AbsThrRange {
        line_low: 26,
        line_high: 26,
        absthr_db: 4.14,
    },
    AbsThrRange {
        line_low: 27,
        line_high: 27,
        absthr_db: 4.00,
    },
    AbsThrRange {
        line_low: 28,
        line_high: 28,
        absthr_db: 3.86,
    },
    AbsThrRange {
        line_low: 29,
        line_high: 29,
        absthr_db: 3.73,
    },
    AbsThrRange {
        line_low: 30,
        line_high: 30,
        absthr_db: 3.61,
    },
    AbsThrRange {
        line_low: 31,
        line_high: 31,
        absthr_db: 3.49,
    },
    AbsThrRange {
        line_low: 32,
        line_high: 32,
        absthr_db: 3.37,
    },
    AbsThrRange {
        line_low: 33,
        line_high: 33,
        absthr_db: 3.26,
    },
    AbsThrRange {
        line_low: 34,
        line_high: 34,
        absthr_db: 3.15,
    },
    AbsThrRange {
        line_low: 35,
        line_high: 35,
        absthr_db: 3.04,
    },
    AbsThrRange {
        line_low: 36,
        line_high: 36,
        absthr_db: 2.93,
    },
    AbsThrRange {
        line_low: 37,
        line_high: 37,
        absthr_db: 2.83,
    },
    AbsThrRange {
        line_low: 38,
        line_high: 38,
        absthr_db: 2.73,
    },
    AbsThrRange {
        line_low: 39,
        line_high: 39,
        absthr_db: 2.63,
    },
    AbsThrRange {
        line_low: 40,
        line_high: 40,
        absthr_db: 2.53,
    },
    AbsThrRange {
        line_low: 41,
        line_high: 41,
        absthr_db: 2.42,
    },
    AbsThrRange {
        line_low: 42,
        line_high: 42,
        absthr_db: 2.32,
    },
    AbsThrRange {
        line_low: 43,
        line_high: 43,
        absthr_db: 2.22,
    },
    AbsThrRange {
        line_low: 44,
        line_high: 44,
        absthr_db: 2.12,
    },
    AbsThrRange {
        line_low: 45,
        line_high: 45,
        absthr_db: 2.02,
    },
    AbsThrRange {
        line_low: 46,
        line_high: 46,
        absthr_db: 1.92,
    },
    AbsThrRange {
        line_low: 47,
        line_high: 47,
        absthr_db: 1.81,
    },
    AbsThrRange {
        line_low: 48,
        line_high: 48,
        absthr_db: 1.71,
    },
    AbsThrRange {
        line_low: 49,
        line_high: 50,
        absthr_db: 1.49,
    },
    AbsThrRange {
        line_low: 51,
        line_high: 52,
        absthr_db: 1.27,
    },
    AbsThrRange {
        line_low: 53,
        line_high: 54,
        absthr_db: 1.04,
    },
    AbsThrRange {
        line_low: 55,
        line_high: 56,
        absthr_db: 0.80,
    },
    AbsThrRange {
        line_low: 57,
        line_high: 58,
        absthr_db: 0.55,
    },
    AbsThrRange {
        line_low: 59,
        line_high: 60,
        absthr_db: 0.29,
    },
    AbsThrRange {
        line_low: 61,
        line_high: 62,
        absthr_db: 0.02,
    },
    AbsThrRange {
        line_low: 63,
        line_high: 64,
        absthr_db: -0.25,
    },
    AbsThrRange {
        line_low: 65,
        line_high: 66,
        absthr_db: -0.54,
    },
    AbsThrRange {
        line_low: 67,
        line_high: 68,
        absthr_db: -0.83,
    },
    AbsThrRange {
        line_low: 69,
        line_high: 70,
        absthr_db: -1.12,
    },
    AbsThrRange {
        line_low: 71,
        line_high: 72,
        absthr_db: -1.43,
    },
    AbsThrRange {
        line_low: 73,
        line_high: 74,
        absthr_db: -1.73,
    },
    AbsThrRange {
        line_low: 75,
        line_high: 76,
        absthr_db: -2.04,
    },
    AbsThrRange {
        line_low: 77,
        line_high: 78,
        absthr_db: -2.34,
    },
    AbsThrRange {
        line_low: 79,
        line_high: 80,
        absthr_db: -2.64,
    },
    AbsThrRange {
        line_low: 81,
        line_high: 82,
        absthr_db: -2.93,
    },
    AbsThrRange {
        line_low: 83,
        line_high: 84,
        absthr_db: -3.22,
    },
    AbsThrRange {
        line_low: 85,
        line_high: 86,
        absthr_db: -3.49,
    },
    AbsThrRange {
        line_low: 87,
        line_high: 88,
        absthr_db: -3.74,
    },
    AbsThrRange {
        line_low: 89,
        line_high: 90,
        absthr_db: -3.98,
    },
    AbsThrRange {
        line_low: 91,
        line_high: 92,
        absthr_db: -4.20,
    },
    AbsThrRange {
        line_low: 93,
        line_high: 94,
        absthr_db: -4.40,
    },
    AbsThrRange {
        line_low: 95,
        line_high: 96,
        absthr_db: -4.57,
    },
    AbsThrRange {
        line_low: 97,
        line_high: 100,
        absthr_db: -4.82,
    },
    AbsThrRange {
        line_low: 101,
        line_high: 104,
        absthr_db: -4.96,
    },
    AbsThrRange {
        line_low: 105,
        line_high: 108,
        absthr_db: -4.97,
    },
    AbsThrRange {
        line_low: 109,
        line_high: 112,
        absthr_db: -4.86,
    },
    AbsThrRange {
        line_low: 113,
        line_high: 116,
        absthr_db: -4.63,
    },
    AbsThrRange {
        line_low: 117,
        line_high: 120,
        absthr_db: -4.29,
    },
    AbsThrRange {
        line_low: 121,
        line_high: 124,
        absthr_db: -3.87,
    },
    AbsThrRange {
        line_low: 125,
        line_high: 128,
        absthr_db: -3.39,
    },
    AbsThrRange {
        line_low: 129,
        line_high: 132,
        absthr_db: -2.86,
    },
    AbsThrRange {
        line_low: 133,
        line_high: 136,
        absthr_db: -2.31,
    },
    AbsThrRange {
        line_low: 137,
        line_high: 140,
        absthr_db: -1.77,
    },
    AbsThrRange {
        line_low: 141,
        line_high: 144,
        absthr_db: -1.24,
    },
    AbsThrRange {
        line_low: 145,
        line_high: 148,
        absthr_db: -0.74,
    },
    AbsThrRange {
        line_low: 149,
        line_high: 152,
        absthr_db: -0.29,
    },
    AbsThrRange {
        line_low: 153,
        line_high: 156,
        absthr_db: 0.12,
    },
    AbsThrRange {
        line_low: 157,
        line_high: 160,
        absthr_db: 0.48,
    },
    AbsThrRange {
        line_low: 161,
        line_high: 164,
        absthr_db: 0.79,
    },
    AbsThrRange {
        line_low: 165,
        line_high: 168,
        absthr_db: 1.06,
    },
    AbsThrRange {
        line_low: 169,
        line_high: 172,
        absthr_db: 1.29,
    },
    AbsThrRange {
        line_low: 173,
        line_high: 176,
        absthr_db: 1.49,
    },
    AbsThrRange {
        line_low: 177,
        line_high: 180,
        absthr_db: 1.66,
    },
    AbsThrRange {
        line_low: 181,
        line_high: 184,
        absthr_db: 1.81,
    },
    AbsThrRange {
        line_low: 185,
        line_high: 188,
        absthr_db: 1.95,
    },
    AbsThrRange {
        line_low: 189,
        line_high: 192,
        absthr_db: 2.08,
    },
    AbsThrRange {
        line_low: 193,
        line_high: 200,
        absthr_db: 2.33,
    },
    AbsThrRange {
        line_low: 201,
        line_high: 208,
        absthr_db: 2.59,
    },
    AbsThrRange {
        line_low: 209,
        line_high: 216,
        absthr_db: 2.86,
    },
    AbsThrRange {
        line_low: 217,
        line_high: 224,
        absthr_db: 3.17,
    },
    AbsThrRange {
        line_low: 225,
        line_high: 232,
        absthr_db: 3.51,
    },
    AbsThrRange {
        line_low: 233,
        line_high: 240,
        absthr_db: 3.89,
    },
    AbsThrRange {
        line_low: 241,
        line_high: 248,
        absthr_db: 4.31,
    },
    AbsThrRange {
        line_low: 249,
        line_high: 256,
        absthr_db: 4.79,
    },
    AbsThrRange {
        line_low: 257,
        line_high: 264,
        absthr_db: 5.31,
    },
    AbsThrRange {
        line_low: 265,
        line_high: 272,
        absthr_db: 5.88,
    },
    AbsThrRange {
        line_low: 273,
        line_high: 280,
        absthr_db: 6.50,
    },
    AbsThrRange {
        line_low: 281,
        line_high: 288,
        absthr_db: 7.19,
    },
    AbsThrRange {
        line_low: 289,
        line_high: 296,
        absthr_db: 7.93,
    },
    AbsThrRange {
        line_low: 297,
        line_high: 304,
        absthr_db: 8.75,
    },
    AbsThrRange {
        line_low: 305,
        line_high: 312,
        absthr_db: 9.63,
    },
    AbsThrRange {
        line_low: 313,
        line_high: 320,
        absthr_db: 10.58,
    },
    AbsThrRange {
        line_low: 321,
        line_high: 328,
        absthr_db: 11.60,
    },
    AbsThrRange {
        line_low: 329,
        line_high: 336,
        absthr_db: 12.71,
    },
    AbsThrRange {
        line_low: 337,
        line_high: 344,
        absthr_db: 13.90,
    },
    AbsThrRange {
        line_low: 345,
        line_high: 352,
        absthr_db: 15.18,
    },
    AbsThrRange {
        line_low: 353,
        line_high: 360,
        absthr_db: 16.54,
    },
    AbsThrRange {
        line_low: 361,
        line_high: 368,
        absthr_db: 18.01,
    },
    AbsThrRange {
        line_low: 369,
        line_high: 376,
        absthr_db: 19.57,
    },
    AbsThrRange {
        line_low: 377,
        line_high: 384,
        absthr_db: 21.23,
    },
    AbsThrRange {
        line_low: 385,
        line_high: 392,
        absthr_db: 23.01,
    },
    AbsThrRange {
        line_low: 393,
        line_high: 400,
        absthr_db: 24.90,
    },
    AbsThrRange {
        line_low: 401,
        line_high: 408,
        absthr_db: 26.90,
    },
    AbsThrRange {
        line_low: 409,
        line_high: 416,
        absthr_db: 29.03,
    },
    AbsThrRange {
        line_low: 417,
        line_high: 424,
        absthr_db: 31.28,
    },
    AbsThrRange {
        line_low: 425,
        line_high: 432,
        absthr_db: 33.67,
    },
    AbsThrRange {
        line_low: 433,
        line_high: 440,
        absthr_db: 36.19,
    },
    AbsThrRange {
        line_low: 441,
        line_high: 448,
        absthr_db: 38.86,
    },
    AbsThrRange {
        line_low: 449,
        line_high: 456,
        absthr_db: 41.67,
    },
    AbsThrRange {
        line_low: 457,
        line_high: 464,
        absthr_db: 44.63,
    },
    AbsThrRange {
        line_low: 465,
        line_high: 472,
        absthr_db: 47.76,
    },
    AbsThrRange {
        line_low: 473,
        line_high: 480,
        absthr_db: 51.03,
    },
];

/// Annex D Table D.4b — Model 2 per-FFT-line absolute-threshold table
/// at **Fs = 44,1 kHz**, all 130 line-ranges printed in ISO/IEC
/// 11172-3 (1993) PDF page 143 (printed 137).
///
/// Transcribed from `docs/audio/mp3/annex-d-table-D4b-absolute-threshold-44k1Hz.csv`.
/// The ranges tile FFT lines 1…464 contiguously. This table carries
/// the genuinely-surprising **69,13 dB** saturation ceiling (lines
/// 369–464) — higher than the matching Model-1 D.1e table's 68,00 dB —
/// alongside a top run printing 0,01 dB below the D.1e twins; the
/// const reproduces the as-printed D.4b values.
pub const ABSTHR_D4B_44K1: [AbsThrRange; 130] = [
    AbsThrRange {
        line_low: 1,
        line_high: 1,
        absthr_db: 45.05,
    },
    AbsThrRange {
        line_low: 2,
        line_high: 2,
        absthr_db: 25.87,
    },
    AbsThrRange {
        line_low: 3,
        line_high: 3,
        absthr_db: 18.70,
    },
    AbsThrRange {
        line_low: 4,
        line_high: 4,
        absthr_db: 14.85,
    },
    AbsThrRange {
        line_low: 5,
        line_high: 5,
        absthr_db: 12.41,
    },
    AbsThrRange {
        line_low: 6,
        line_high: 6,
        absthr_db: 10.72,
    },
    AbsThrRange {
        line_low: 7,
        line_high: 7,
        absthr_db: 9.47,
    },
    AbsThrRange {
        line_low: 8,
        line_high: 8,
        absthr_db: 8.50,
    },
    AbsThrRange {
        line_low: 9,
        line_high: 9,
        absthr_db: 7.73,
    },
    AbsThrRange {
        line_low: 10,
        line_high: 10,
        absthr_db: 7.10,
    },
    AbsThrRange {
        line_low: 11,
        line_high: 11,
        absthr_db: 6.56,
    },
    AbsThrRange {
        line_low: 12,
        line_high: 12,
        absthr_db: 6.11,
    },
    AbsThrRange {
        line_low: 13,
        line_high: 13,
        absthr_db: 5.72,
    },
    AbsThrRange {
        line_low: 14,
        line_high: 14,
        absthr_db: 5.37,
    },
    AbsThrRange {
        line_low: 15,
        line_high: 15,
        absthr_db: 5.07,
    },
    AbsThrRange {
        line_low: 16,
        line_high: 16,
        absthr_db: 4.79,
    },
    AbsThrRange {
        line_low: 17,
        line_high: 17,
        absthr_db: 4.55,
    },
    AbsThrRange {
        line_low: 18,
        line_high: 18,
        absthr_db: 4.32,
    },
    AbsThrRange {
        line_low: 19,
        line_high: 19,
        absthr_db: 4.11,
    },
    AbsThrRange {
        line_low: 20,
        line_high: 20,
        absthr_db: 3.92,
    },
    AbsThrRange {
        line_low: 21,
        line_high: 21,
        absthr_db: 3.74,
    },
    AbsThrRange {
        line_low: 22,
        line_high: 22,
        absthr_db: 3.57,
    },
    AbsThrRange {
        line_low: 23,
        line_high: 23,
        absthr_db: 3.40,
    },
    AbsThrRange {
        line_low: 24,
        line_high: 24,
        absthr_db: 3.25,
    },
    AbsThrRange {
        line_low: 25,
        line_high: 25,
        absthr_db: 3.10,
    },
    AbsThrRange {
        line_low: 26,
        line_high: 26,
        absthr_db: 2.95,
    },
    AbsThrRange {
        line_low: 27,
        line_high: 27,
        absthr_db: 2.81,
    },
    AbsThrRange {
        line_low: 28,
        line_high: 28,
        absthr_db: 2.67,
    },
    AbsThrRange {
        line_low: 29,
        line_high: 29,
        absthr_db: 2.53,
    },
    AbsThrRange {
        line_low: 30,
        line_high: 30,
        absthr_db: 2.39,
    },
    AbsThrRange {
        line_low: 31,
        line_high: 31,
        absthr_db: 2.25,
    },
    AbsThrRange {
        line_low: 32,
        line_high: 32,
        absthr_db: 2.11,
    },
    AbsThrRange {
        line_low: 33,
        line_high: 33,
        absthr_db: 1.97,
    },
    AbsThrRange {
        line_low: 34,
        line_high: 34,
        absthr_db: 1.83,
    },
    AbsThrRange {
        line_low: 35,
        line_high: 35,
        absthr_db: 1.68,
    },
    AbsThrRange {
        line_low: 36,
        line_high: 36,
        absthr_db: 1.53,
    },
    AbsThrRange {
        line_low: 37,
        line_high: 37,
        absthr_db: 1.38,
    },
    AbsThrRange {
        line_low: 38,
        line_high: 38,
        absthr_db: 1.23,
    },
    AbsThrRange {
        line_low: 39,
        line_high: 39,
        absthr_db: 1.07,
    },
    AbsThrRange {
        line_low: 40,
        line_high: 40,
        absthr_db: 0.90,
    },
    AbsThrRange {
        line_low: 41,
        line_high: 41,
        absthr_db: 0.74,
    },
    AbsThrRange {
        line_low: 42,
        line_high: 42,
        absthr_db: 0.56,
    },
    AbsThrRange {
        line_low: 43,
        line_high: 43,
        absthr_db: 0.39,
    },
    AbsThrRange {
        line_low: 44,
        line_high: 44,
        absthr_db: 0.21,
    },
    AbsThrRange {
        line_low: 45,
        line_high: 45,
        absthr_db: 0.02,
    },
    AbsThrRange {
        line_low: 46,
        line_high: 46,
        absthr_db: -0.17,
    },
    AbsThrRange {
        line_low: 47,
        line_high: 47,
        absthr_db: -0.36,
    },
    AbsThrRange {
        line_low: 48,
        line_high: 48,
        absthr_db: -0.56,
    },
    AbsThrRange {
        line_low: 49,
        line_high: 50,
        absthr_db: -0.96,
    },
    AbsThrRange {
        line_low: 51,
        line_high: 52,
        absthr_db: -1.38,
    },
    AbsThrRange {
        line_low: 53,
        line_high: 54,
        absthr_db: -1.79,
    },
    AbsThrRange {
        line_low: 55,
        line_high: 56,
        absthr_db: -2.21,
    },
    AbsThrRange {
        line_low: 57,
        line_high: 58,
        absthr_db: -2.63,
    },
    AbsThrRange {
        line_low: 59,
        line_high: 60,
        absthr_db: -3.03,
    },
    AbsThrRange {
        line_low: 61,
        line_high: 62,
        absthr_db: -3.41,
    },
    AbsThrRange {
        line_low: 63,
        line_high: 64,
        absthr_db: -3.77,
    },
    AbsThrRange {
        line_low: 65,
        line_high: 66,
        absthr_db: -4.09,
    },
    AbsThrRange {
        line_low: 67,
        line_high: 68,
        absthr_db: -4.37,
    },
    AbsThrRange {
        line_low: 69,
        line_high: 70,
        absthr_db: -4.60,
    },
    AbsThrRange {
        line_low: 71,
        line_high: 72,
        absthr_db: -4.78,
    },
    AbsThrRange {
        line_low: 73,
        line_high: 74,
        absthr_db: -4.91,
    },
    AbsThrRange {
        line_low: 75,
        line_high: 76,
        absthr_db: -4.97,
    },
    AbsThrRange {
        line_low: 77,
        line_high: 78,
        absthr_db: -4.98,
    },
    AbsThrRange {
        line_low: 79,
        line_high: 80,
        absthr_db: -4.92,
    },
    AbsThrRange {
        line_low: 81,
        line_high: 82,
        absthr_db: -4.81,
    },
    AbsThrRange {
        line_low: 83,
        line_high: 84,
        absthr_db: -4.65,
    },
    AbsThrRange {
        line_low: 85,
        line_high: 86,
        absthr_db: -4.43,
    },
    AbsThrRange {
        line_low: 87,
        line_high: 88,
        absthr_db: -4.17,
    },
    AbsThrRange {
        line_low: 89,
        line_high: 90,
        absthr_db: -3.87,
    },
    AbsThrRange {
        line_low: 91,
        line_high: 92,
        absthr_db: -3.54,
    },
    AbsThrRange {
        line_low: 93,
        line_high: 94,
        absthr_db: -3.19,
    },
    AbsThrRange {
        line_low: 95,
        line_high: 96,
        absthr_db: -2.82,
    },
    AbsThrRange {
        line_low: 97,
        line_high: 100,
        absthr_db: -2.06,
    },
    AbsThrRange {
        line_low: 101,
        line_high: 104,
        absthr_db: -1.32,
    },
    AbsThrRange {
        line_low: 105,
        line_high: 108,
        absthr_db: -0.64,
    },
    AbsThrRange {
        line_low: 109,
        line_high: 112,
        absthr_db: -0.04,
    },
    AbsThrRange {
        line_low: 113,
        line_high: 116,
        absthr_db: 0.47,
    },
    AbsThrRange {
        line_low: 117,
        line_high: 120,
        absthr_db: 0.89,
    },
    AbsThrRange {
        line_low: 121,
        line_high: 124,
        absthr_db: 1.23,
    },
    AbsThrRange {
        line_low: 125,
        line_high: 128,
        absthr_db: 1.51,
    },
    AbsThrRange {
        line_low: 129,
        line_high: 132,
        absthr_db: 1.74,
    },
    AbsThrRange {
        line_low: 133,
        line_high: 136,
        absthr_db: 1.93,
    },
    AbsThrRange {
        line_low: 137,
        line_high: 140,
        absthr_db: 2.11,
    },
    AbsThrRange {
        line_low: 141,
        line_high: 144,
        absthr_db: 2.28,
    },
    AbsThrRange {
        line_low: 145,
        line_high: 148,
        absthr_db: 2.46,
    },
    AbsThrRange {
        line_low: 149,
        line_high: 152,
        absthr_db: 2.63,
    },
    AbsThrRange {
        line_low: 153,
        line_high: 156,
        absthr_db: 2.82,
    },
    AbsThrRange {
        line_low: 157,
        line_high: 160,
        absthr_db: 3.03,
    },
    AbsThrRange {
        line_low: 161,
        line_high: 164,
        absthr_db: 3.25,
    },
    AbsThrRange {
        line_low: 165,
        line_high: 168,
        absthr_db: 3.49,
    },
    AbsThrRange {
        line_low: 169,
        line_high: 172,
        absthr_db: 3.74,
    },
    AbsThrRange {
        line_low: 173,
        line_high: 176,
        absthr_db: 4.02,
    },
    AbsThrRange {
        line_low: 177,
        line_high: 180,
        absthr_db: 4.32,
    },
    AbsThrRange {
        line_low: 181,
        line_high: 184,
        absthr_db: 4.64,
    },
    AbsThrRange {
        line_low: 185,
        line_high: 188,
        absthr_db: 4.98,
    },
    AbsThrRange {
        line_low: 189,
        line_high: 192,
        absthr_db: 5.35,
    },
    AbsThrRange {
        line_low: 193,
        line_high: 200,
        absthr_db: 6.15,
    },
    AbsThrRange {
        line_low: 201,
        line_high: 208,
        absthr_db: 7.07,
    },
    AbsThrRange {
        line_low: 209,
        line_high: 216,
        absthr_db: 8.10,
    },
    AbsThrRange {
        line_low: 217,
        line_high: 224,
        absthr_db: 9.25,
    },
    AbsThrRange {
        line_low: 225,
        line_high: 232,
        absthr_db: 10.54,
    },
    AbsThrRange {
        line_low: 233,
        line_high: 240,
        absthr_db: 11.97,
    },
    AbsThrRange {
        line_low: 241,
        line_high: 248,
        absthr_db: 13.56,
    },
    AbsThrRange {
        line_low: 249,
        line_high: 256,
        absthr_db: 15.30,
    },
    AbsThrRange {
        line_low: 257,
        line_high: 264,
        absthr_db: 17.23,
    },
    AbsThrRange {
        line_low: 265,
        line_high: 272,
        absthr_db: 19.33,
    },
    AbsThrRange {
        line_low: 273,
        line_high: 280,
        absthr_db: 21.64,
    },
    AbsThrRange {
        line_low: 281,
        line_high: 288,
        absthr_db: 24.15,
    },
    AbsThrRange {
        line_low: 289,
        line_high: 296,
        absthr_db: 26.88,
    },
    AbsThrRange {
        line_low: 297,
        line_high: 304,
        absthr_db: 29.84,
    },
    AbsThrRange {
        line_low: 305,
        line_high: 312,
        absthr_db: 33.04,
    },
    AbsThrRange {
        line_low: 313,
        line_high: 320,
        absthr_db: 36.51,
    },
    AbsThrRange {
        line_low: 321,
        line_high: 328,
        absthr_db: 40.24,
    },
    AbsThrRange {
        line_low: 329,
        line_high: 336,
        absthr_db: 44.26,
    },
    AbsThrRange {
        line_low: 337,
        line_high: 344,
        absthr_db: 48.58,
    },
    AbsThrRange {
        line_low: 345,
        line_high: 352,
        absthr_db: 53.21,
    },
    AbsThrRange {
        line_low: 353,
        line_high: 360,
        absthr_db: 58.17,
    },
    AbsThrRange {
        line_low: 361,
        line_high: 368,
        absthr_db: 63.48,
    },
    AbsThrRange {
        line_low: 369,
        line_high: 376,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 377,
        line_high: 384,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 385,
        line_high: 392,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 393,
        line_high: 400,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 401,
        line_high: 408,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 409,
        line_high: 416,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 417,
        line_high: 424,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 425,
        line_high: 432,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 433,
        line_high: 440,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 441,
        line_high: 448,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 449,
        line_high: 456,
        absthr_db: 69.13,
    },
    AbsThrRange {
        line_low: 457,
        line_high: 464,
        absthr_db: 69.13,
    },
];

/// Annex D Table D.4c — Model 2 per-FFT-line absolute-threshold table
/// at **Fs = 48 kHz**, all 126 line-ranges printed in ISO/IEC 11172-3
/// (1993) PDF page 144 (printed 138).
///
/// Transcribed from `docs/audio/mp3/annex-d-table-D4c-absolute-threshold-48kHz.csv`.
/// The ranges tile FFT lines 1…432 contiguously, saturating at the
/// 68,00 dB ceiling (lines 333–432), matching its D.1f twin entry-for-
/// entry.
pub const ABSTHR_D4C_48K: [AbsThrRange; 126] = [
    AbsThrRange {
        line_low: 1,
        line_high: 1,
        absthr_db: 42.10,
    },
    AbsThrRange {
        line_low: 2,
        line_high: 2,
        absthr_db: 24.17,
    },
    AbsThrRange {
        line_low: 3,
        line_high: 3,
        absthr_db: 17.47,
    },
    AbsThrRange {
        line_low: 4,
        line_high: 4,
        absthr_db: 13.87,
    },
    AbsThrRange {
        line_low: 5,
        line_high: 5,
        absthr_db: 11.60,
    },
    AbsThrRange {
        line_low: 6,
        line_high: 6,
        absthr_db: 10.01,
    },
    AbsThrRange {
        line_low: 7,
        line_high: 7,
        absthr_db: 8.84,
    },
    AbsThrRange {
        line_low: 8,
        line_high: 8,
        absthr_db: 7.94,
    },
    AbsThrRange {
        line_low: 9,
        line_high: 9,
        absthr_db: 7.22,
    },
    AbsThrRange {
        line_low: 10,
        line_high: 10,
        absthr_db: 6.62,
    },
    AbsThrRange {
        line_low: 11,
        line_high: 11,
        absthr_db: 6.12,
    },
    AbsThrRange {
        line_low: 12,
        line_high: 12,
        absthr_db: 5.70,
    },
    AbsThrRange {
        line_low: 13,
        line_high: 13,
        absthr_db: 5.33,
    },
    AbsThrRange {
        line_low: 14,
        line_high: 14,
        absthr_db: 5.00,
    },
    AbsThrRange {
        line_low: 15,
        line_high: 15,
        absthr_db: 4.71,
    },
    AbsThrRange {
        line_low: 16,
        line_high: 16,
        absthr_db: 4.45,
    },
    AbsThrRange {
        line_low: 17,
        line_high: 17,
        absthr_db: 4.21,
    },
    AbsThrRange {
        line_low: 18,
        line_high: 18,
        absthr_db: 4.00,
    },
    AbsThrRange {
        line_low: 19,
        line_high: 19,
        absthr_db: 3.79,
    },
    AbsThrRange {
        line_low: 20,
        line_high: 20,
        absthr_db: 3.61,
    },
    AbsThrRange {
        line_low: 21,
        line_high: 21,
        absthr_db: 3.43,
    },
    AbsThrRange {
        line_low: 22,
        line_high: 22,
        absthr_db: 3.26,
    },
    AbsThrRange {
        line_low: 23,
        line_high: 23,
        absthr_db: 3.09,
    },
    AbsThrRange {
        line_low: 24,
        line_high: 24,
        absthr_db: 2.93,
    },
    AbsThrRange {
        line_low: 25,
        line_high: 25,
        absthr_db: 2.78,
    },
    AbsThrRange {
        line_low: 26,
        line_high: 26,
        absthr_db: 2.63,
    },
    AbsThrRange {
        line_low: 27,
        line_high: 27,
        absthr_db: 2.47,
    },
    AbsThrRange {
        line_low: 28,
        line_high: 28,
        absthr_db: 2.32,
    },
    AbsThrRange {
        line_low: 29,
        line_high: 29,
        absthr_db: 2.17,
    },
    AbsThrRange {
        line_low: 30,
        line_high: 30,
        absthr_db: 2.02,
    },
    AbsThrRange {
        line_low: 31,
        line_high: 31,
        absthr_db: 1.86,
    },
    AbsThrRange {
        line_low: 32,
        line_high: 32,
        absthr_db: 1.71,
    },
    AbsThrRange {
        line_low: 33,
        line_high: 33,
        absthr_db: 1.55,
    },
    AbsThrRange {
        line_low: 34,
        line_high: 34,
        absthr_db: 1.38,
    },
    AbsThrRange {
        line_low: 35,
        line_high: 35,
        absthr_db: 1.21,
    },
    AbsThrRange {
        line_low: 36,
        line_high: 36,
        absthr_db: 1.04,
    },
    AbsThrRange {
        line_low: 37,
        line_high: 37,
        absthr_db: 0.86,
    },
    AbsThrRange {
        line_low: 38,
        line_high: 38,
        absthr_db: 0.67,
    },
    AbsThrRange {
        line_low: 39,
        line_high: 39,
        absthr_db: 0.49,
    },
    AbsThrRange {
        line_low: 40,
        line_high: 40,
        absthr_db: 0.29,
    },
    AbsThrRange {
        line_low: 41,
        line_high: 41,
        absthr_db: 0.09,
    },
    AbsThrRange {
        line_low: 42,
        line_high: 42,
        absthr_db: -0.11,
    },
    AbsThrRange {
        line_low: 43,
        line_high: 43,
        absthr_db: -0.32,
    },
    AbsThrRange {
        line_low: 44,
        line_high: 44,
        absthr_db: -0.54,
    },
    AbsThrRange {
        line_low: 45,
        line_high: 45,
        absthr_db: -0.75,
    },
    AbsThrRange {
        line_low: 46,
        line_high: 46,
        absthr_db: -0.97,
    },
    AbsThrRange {
        line_low: 47,
        line_high: 47,
        absthr_db: -1.20,
    },
    AbsThrRange {
        line_low: 48,
        line_high: 48,
        absthr_db: -1.43,
    },
    AbsThrRange {
        line_low: 49,
        line_high: 50,
        absthr_db: -1.88,
    },
    AbsThrRange {
        line_low: 51,
        line_high: 52,
        absthr_db: -2.34,
    },
    AbsThrRange {
        line_low: 53,
        line_high: 54,
        absthr_db: -2.79,
    },
    AbsThrRange {
        line_low: 55,
        line_high: 56,
        absthr_db: -3.22,
    },
    AbsThrRange {
        line_low: 57,
        line_high: 58,
        absthr_db: -3.62,
    },
    AbsThrRange {
        line_low: 59,
        line_high: 60,
        absthr_db: -3.98,
    },
    AbsThrRange {
        line_low: 61,
        line_high: 62,
        absthr_db: -4.30,
    },
    AbsThrRange {
        line_low: 63,
        line_high: 64,
        absthr_db: -4.57,
    },
    AbsThrRange {
        line_low: 65,
        line_high: 66,
        absthr_db: -4.77,
    },
    AbsThrRange {
        line_low: 67,
        line_high: 68,
        absthr_db: -4.91,
    },
    AbsThrRange {
        line_low: 69,
        line_high: 70,
        absthr_db: -4.98,
    },
    AbsThrRange {
        line_low: 71,
        line_high: 72,
        absthr_db: -4.97,
    },
    AbsThrRange {
        line_low: 73,
        line_high: 74,
        absthr_db: -4.90,
    },
    AbsThrRange {
        line_low: 75,
        line_high: 76,
        absthr_db: -4.76,
    },
    AbsThrRange {
        line_low: 77,
        line_high: 78,
        absthr_db: -4.55,
    },
    AbsThrRange {
        line_low: 79,
        line_high: 80,
        absthr_db: -4.29,
    },
    AbsThrRange {
        line_low: 81,
        line_high: 82,
        absthr_db: -3.99,
    },
    AbsThrRange {
        line_low: 83,
        line_high: 84,
        absthr_db: -3.64,
    },
    AbsThrRange {
        line_low: 85,
        line_high: 86,
        absthr_db: -3.26,
    },
    AbsThrRange {
        line_low: 87,
        line_high: 88,
        absthr_db: -2.86,
    },
    AbsThrRange {
        line_low: 89,
        line_high: 90,
        absthr_db: -2.45,
    },
    AbsThrRange {
        line_low: 91,
        line_high: 92,
        absthr_db: -2.04,
    },
    AbsThrRange {
        line_low: 93,
        line_high: 94,
        absthr_db: -1.63,
    },
    AbsThrRange {
        line_low: 95,
        line_high: 96,
        absthr_db: -1.24,
    },
    AbsThrRange {
        line_low: 97,
        line_high: 100,
        absthr_db: -0.51,
    },
    AbsThrRange {
        line_low: 101,
        line_high: 104,
        absthr_db: 0.12,
    },
    AbsThrRange {
        line_low: 105,
        line_high: 108,
        absthr_db: 0.64,
    },
    AbsThrRange {
        line_low: 109,
        line_high: 112,
        absthr_db: 1.06,
    },
    AbsThrRange {
        line_low: 113,
        line_high: 116,
        absthr_db: 1.39,
    },
    AbsThrRange {
        line_low: 117,
        line_high: 120,
        absthr_db: 1.66,
    },
    AbsThrRange {
        line_low: 121,
        line_high: 124,
        absthr_db: 1.88,
    },
    AbsThrRange {
        line_low: 125,
        line_high: 128,
        absthr_db: 2.08,
    },
    AbsThrRange {
        line_low: 129,
        line_high: 132,
        absthr_db: 2.27,
    },
    AbsThrRange {
        line_low: 133,
        line_high: 136,
        absthr_db: 2.46,
    },
    AbsThrRange {
        line_low: 137,
        line_high: 140,
        absthr_db: 2.65,
    },
    AbsThrRange {
        line_low: 141,
        line_high: 144,
        absthr_db: 2.86,
    },
    AbsThrRange {
        line_low: 145,
        line_high: 148,
        absthr_db: 3.09,
    },
    AbsThrRange {
        line_low: 149,
        line_high: 152,
        absthr_db: 3.33,
    },
    AbsThrRange {
        line_low: 153,
        line_high: 156,
        absthr_db: 3.60,
    },
    AbsThrRange {
        line_low: 157,
        line_high: 160,
        absthr_db: 3.89,
    },
    AbsThrRange {
        line_low: 161,
        line_high: 164,
        absthr_db: 4.20,
    },
    AbsThrRange {
        line_low: 165,
        line_high: 168,
        absthr_db: 4.54,
    },
    AbsThrRange {
        line_low: 169,
        line_high: 172,
        absthr_db: 4.91,
    },
    AbsThrRange {
        line_low: 173,
        line_high: 176,
        absthr_db: 5.31,
    },
    AbsThrRange {
        line_low: 177,
        line_high: 180,
        absthr_db: 5.73,
    },
    AbsThrRange {
        line_low: 181,
        line_high: 184,
        absthr_db: 6.18,
    },
    AbsThrRange {
        line_low: 185,
        line_high: 188,
        absthr_db: 6.67,
    },
    AbsThrRange {
        line_low: 189,
        line_high: 192,
        absthr_db: 7.19,
    },
    AbsThrRange {
        line_low: 193,
        line_high: 200,
        absthr_db: 8.33,
    },
    AbsThrRange {
        line_low: 201,
        line_high: 208,
        absthr_db: 9.63,
    },
    AbsThrRange {
        line_low: 209,
        line_high: 216,
        absthr_db: 11.08,
    },
    AbsThrRange {
        line_low: 217,
        line_high: 224,
        absthr_db: 12.71,
    },
    AbsThrRange {
        line_low: 225,
        line_high: 232,
        absthr_db: 14.53,
    },
    AbsThrRange {
        line_low: 233,
        line_high: 240,
        absthr_db: 16.54,
    },
    AbsThrRange {
        line_low: 241,
        line_high: 248,
        absthr_db: 18.77,
    },
    AbsThrRange {
        line_low: 249,
        line_high: 256,
        absthr_db: 21.23,
    },
    AbsThrRange {
        line_low: 257,
        line_high: 264,
        absthr_db: 23.94,
    },
    AbsThrRange {
        line_low: 265,
        line_high: 272,
        absthr_db: 26.90,
    },
    AbsThrRange {
        line_low: 273,
        line_high: 280,
        absthr_db: 30.14,
    },
    AbsThrRange {
        line_low: 281,
        line_high: 288,
        absthr_db: 33.67,
    },
    AbsThrRange {
        line_low: 289,
        line_high: 296,
        absthr_db: 37.51,
    },
    AbsThrRange {
        line_low: 297,
        line_high: 304,
        absthr_db: 41.67,
    },
    AbsThrRange {
        line_low: 305,
        line_high: 312,
        absthr_db: 46.17,
    },
    AbsThrRange {
        line_low: 313,
        line_high: 320,
        absthr_db: 51.04,
    },
    AbsThrRange {
        line_low: 321,
        line_high: 328,
        absthr_db: 56.29,
    },
    AbsThrRange {
        line_low: 329,
        line_high: 336,
        absthr_db: 61.94,
    },
    AbsThrRange {
        line_low: 337,
        line_high: 344,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 345,
        line_high: 352,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 353,
        line_high: 360,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 361,
        line_high: 368,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 369,
        line_high: 376,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 377,
        line_high: 384,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 385,
        line_high: 392,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 393,
        line_high: 400,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 401,
        line_high: 408,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 409,
        line_high: 416,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 417,
        line_high: 424,
        absthr_db: 68.00,
    },
    AbsThrRange {
        line_low: 425,
        line_high: 432,
        absthr_db: 68.00,
    },
];

/// Look up the Model 2 absolute threshold (dB) for FFT line `line` at
/// **Fs = 32 kHz** (Table D.4a). `None` for `line == 0` and beyond the
/// last covered line (480).
pub fn absthr_for_line_32k(line: u16) -> Option<f64> {
    absthr_for_line_in(&ABSTHR_D4A_32K, line)
}

/// Look up the Model 2 absolute threshold (dB) for FFT line `line` at
/// **Fs = 44,1 kHz** (Table D.4b). `None` for `line == 0` and beyond
/// the last covered line (464).
pub fn absthr_for_line_44k1(line: u16) -> Option<f64> {
    absthr_for_line_in(&ABSTHR_D4B_44K1, line)
}

/// Look up the Model 2 absolute threshold (dB) for FFT line `line` at
/// **Fs = 48 kHz** (Table D.4c). `None` for `line == 0` and beyond the
/// last covered line (432).
pub fn absthr_for_line_48k(line: u16) -> Option<f64> {
    absthr_for_line_in(&ABSTHR_D4C_48K, line)
}

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------
    // Table D.2 sanity — band count + monotonicity + endpoints
    // -----------------------------------------------------------

    #[test]
    fn d2_band_counts_match_spec_prose() {
        // Layer I: 24 / 25 / 26 bands; Layer II: 25 / 27 / 27 bands.
        assert_eq!(critical_band_table(Layer::I, 32_000).unwrap().len(), 24);
        assert_eq!(critical_band_table(Layer::I, 44_100).unwrap().len(), 25);
        assert_eq!(critical_band_table(Layer::I, 48_000).unwrap().len(), 26);
        assert_eq!(critical_band_table(Layer::II, 32_000).unwrap().len(), 25);
        assert_eq!(critical_band_table(Layer::II, 44_100).unwrap().len(), 27);
        assert_eq!(critical_band_table(Layer::II, 48_000).unwrap().len(), 27);
    }

    #[test]
    fn d2_unsupported_lsf_rates_return_none() {
        // Annex D makes no LSF reference; 16 / 22.05 / 24 kHz are
        // outside its scope.
        for &fs in &[16_000u32, 22_050, 24_000, 8_000, 11_025, 12_000] {
            assert!(critical_band_table(Layer::I, fs).is_none(), "fs={fs}");
            assert!(critical_band_table(Layer::II, fs).is_none(), "fs={fs}");
        }
    }

    #[test]
    fn d2_strict_monotonicity_each_table() {
        for &(layer, fs) in &[
            (Layer::I, 32_000u32),
            (Layer::I, 44_100),
            (Layer::I, 48_000),
            (Layer::II, 32_000),
            (Layer::II, 44_100),
            (Layer::II, 48_000),
        ] {
            let rows = critical_band_table(layer, fs).unwrap();
            for w in rows.windows(2) {
                assert!(
                    w[1].index_fcb > w[0].index_fcb,
                    "index_fcb non-monotone in ({layer:?}, {fs}) at {:?} -> {:?}",
                    w[0],
                    w[1]
                );
                assert!(
                    w[1].top_freq_hz > w[0].top_freq_hz,
                    "freq non-monotone in ({layer:?}, {fs})"
                );
                assert!(
                    w[1].bark_z > w[0].bark_z,
                    "bark_z non-monotone in ({layer:?}, {fs})"
                );
            }
        }
    }

    #[test]
    fn d2a_endpoints_match_spec() {
        // First and last rows of Table D.2a (verbatim from the docs).
        let t = critical_band_table(Layer::I, 32_000).unwrap();
        assert_eq!(t[0].index_fcb, 1);
        assert!((t[0].top_freq_hz - 62.500).abs() < 1e-9);
        assert!((t[0].bark_z - 0.617).abs() < 1e-9);
        assert_eq!(t[23].index_fcb, 108);
        assert!((t[23].top_freq_hz - 15_000.000).abs() < 1e-9);
        assert!((t[23].bark_z - 23.923).abs() < 1e-9);
    }

    #[test]
    fn d2c_endpoints_match_spec() {
        // First and last rows of Table D.2c (Layer I, 48 kHz).
        let t = critical_band_table(Layer::I, 48_000).unwrap();
        assert_eq!(t[0].index_fcb, 1);
        assert!((t[0].top_freq_hz - 93.750).abs() < 1e-9);
        assert!((t[0].bark_z - 0.925).abs() < 1e-9);
        assert_eq!(t[25].index_fcb, 102);
        assert!((t[25].top_freq_hz - 20_250.000).abs() < 1e-9);
        assert!((t[25].bark_z - 24.597).abs() < 1e-9);
    }

    #[test]
    fn d2d_endpoints_match_spec() {
        // Layer II, 32 kHz — first and last rows.
        let t = critical_band_table(Layer::II, 32_000).unwrap();
        assert_eq!(t[0].index_fcb, 1);
        assert!((t[0].top_freq_hz - 31.250).abs() < 1e-9);
        assert!((t[0].bark_z - 0.309).abs() < 1e-9);
        assert_eq!(t[24].index_fcb, 132);
        assert!((t[24].top_freq_hz - 15_000.000).abs() < 1e-9);
    }

    #[test]
    fn d2_top_freq_below_nyquist_for_each_fs() {
        // Every band's top edge must sit below Nyquist for the
        // table's sampling frequency. (D.2b @ 44.1 kHz tops at
        // ~19 982.8 Hz; D.2c @ 48 kHz tops at 20 250 Hz; both
        // valid.)
        for &(layer, fs) in &[
            (Layer::I, 32_000u32),
            (Layer::I, 44_100),
            (Layer::I, 48_000),
            (Layer::II, 32_000),
            (Layer::II, 44_100),
            (Layer::II, 48_000),
        ] {
            let rows = critical_band_table(layer, fs).unwrap();
            let nyq = fs as f64 / 2.0;
            assert!(
                rows.last().unwrap().top_freq_hz <= nyq + 1e-6,
                "({layer:?}, {fs}) last band exceeds Nyquist {nyq}"
            );
        }
    }

    // -----------------------------------------------------------
    // Step 6 — av masking-index
    // -----------------------------------------------------------

    #[test]
    fn av_tm_at_z_zero_matches_spec_constants() {
        // av_tm(0) = -1.525 - 0 - 4.5 = -6.025 dB
        assert!((masking_index_tonal(0.0) - (-6.025)).abs() < 1e-12);
    }

    #[test]
    fn av_nm_at_z_zero_matches_spec_constants() {
        // av_nm(0) = -1.525 - 0 - 0.5 = -2.025 dB
        assert!((masking_index_non_tonal(0.0) - (-2.025)).abs() < 1e-12);
    }

    #[test]
    fn av_tonal_lower_than_non_tonal_for_positive_z() {
        // Tonal maskers are weaker maskers than non-tonal (av_tm has a
        // -4.5 dB offset vs av_nm's -0.5 dB), so av_tm(z) < av_nm(z)
        // for every z >= 0 in the standard band range.
        for &z in &[0.0f64, 1.0, 5.0, 12.0, 24.0] {
            assert!(
                masking_index_tonal(z) < masking_index_non_tonal(z),
                "tonal av not < non-tonal av at z={z}"
            );
        }
    }

    #[test]
    fn av_monotonically_decreasing_in_z() {
        let mut prev_tm = f64::INFINITY;
        let mut prev_nm = f64::INFINITY;
        for i in 0..=24 {
            let z = i as f64;
            let tm = masking_index_tonal(z);
            let nm = masking_index_non_tonal(z);
            assert!(tm < prev_tm, "av_tm not monotone at z={z}");
            assert!(nm < prev_nm, "av_nm not monotone at z={z}");
            prev_tm = tm;
            prev_nm = nm;
        }
    }

    // -----------------------------------------------------------
    // Step 6 — vf masking-function piecewise branches
    // -----------------------------------------------------------

    #[test]
    fn vf_branch_classification_matches_spec_intervals() {
        // Outside [-3, 8): ignored.
        assert_eq!(masking_branch(-3.5), None);
        assert_eq!(masking_branch(8.0), None);
        assert_eq!(masking_branch(8.5), None);
        // -3 <= dz < -1
        assert_eq!(masking_branch(-3.0), Some(MaskingBranch::LowFar));
        assert_eq!(masking_branch(-2.0), Some(MaskingBranch::LowFar));
        // -1 <= dz < 0
        assert_eq!(masking_branch(-1.0), Some(MaskingBranch::LowNear));
        assert_eq!(masking_branch(-0.5), Some(MaskingBranch::LowNear));
        // 0 <= dz < 1
        assert_eq!(masking_branch(0.0), Some(MaskingBranch::HighNear));
        assert_eq!(masking_branch(0.999), Some(MaskingBranch::HighNear));
        // 1 <= dz < 8
        assert_eq!(masking_branch(1.0), Some(MaskingBranch::HighFar));
        assert_eq!(masking_branch(7.5), Some(MaskingBranch::HighFar));
    }

    #[test]
    fn vf_peak_at_dz_zero_is_zero() {
        // -17 * 0 = 0; the spreading function tops out at the masker
        // bin itself, so vf(0, X) = 0 for every X.
        for &x in &[0.0f64, 30.0, 60.0, 96.0] {
            assert!((masking_function(0.0, x).unwrap() - 0.0).abs() < 1e-12);
        }
    }

    #[test]
    fn vf_high_near_branch_matches_minus_seventeen_dz() {
        // 0 <= dz < 1: vf = -17 * dz, independent of X.
        let x = 60.0;
        for &dz in &[0.0f64, 0.1, 0.5, 0.9] {
            let got = masking_function(dz, x).unwrap();
            assert!(
                (got - (-17.0 * dz)).abs() < 1e-12,
                "vf({dz}, {x}) = {got}, expected {}",
                -17.0 * dz
            );
        }
    }

    #[test]
    fn vf_low_near_branch_passes_through_origin() {
        // -1 <= dz < 0: vf = (0.4*X + 6) * dz. At dz = 0 vf -> 0
        // (boundary belongs to high-near branch); at dz slightly
        // negative the value is small-negative, proportional to (X+).
        let x = 60.0;
        let coeff = 0.4 * x + 6.0;
        for &dz in &[-0.99f64, -0.5, -0.1] {
            let got = masking_function(dz, x).unwrap();
            assert!(
                (got - coeff * dz).abs() < 1e-12,
                "vf({dz}, {x}) = {got}, expected {}",
                coeff * dz
            );
        }
    }

    #[test]
    fn vf_low_far_branch_matches_formula() {
        // -3 <= dz < -1: vf = 17*(dz+1) - (0.4*X + 6).
        let x = 80.0;
        for &dz in &[-3.0f64, -2.0, -1.5, -1.01] {
            let got = masking_function(dz, x).unwrap();
            let want = 17.0 * (dz + 1.0) - (0.4 * x + 6.0);
            assert!((got - want).abs() < 1e-12, "vf({dz}, {x})");
        }
    }

    #[test]
    fn vf_high_far_branch_matches_formula() {
        // 1 <= dz < 8: vf = -(dz - 1)*(17 - 0.15*X) - 17
        let x = 60.0;
        for &dz in &[1.0f64, 2.0, 5.0, 7.99] {
            let got = masking_function(dz, x).unwrap();
            let want = -(dz - 1.0) * (17.0 - 0.15 * x) - 17.0;
            assert!((got - want).abs() < 1e-12, "vf({dz}, {x})");
        }
    }

    #[test]
    fn vf_outside_window_is_none() {
        // Maskers below -3 Bark or at/above +8 Bark are ignored.
        assert!(masking_function(-3.001, 60.0).is_none());
        assert!(masking_function(-10.0, 60.0).is_none());
        assert!(masking_function(8.0, 60.0).is_none());
        assert!(masking_function(20.0, 60.0).is_none());
    }

    #[test]
    fn individual_threshold_tonal_combines_x_av_vf() {
        // LT_tm = X + av_tm(z_j) + vf(z_i - z_j, X). At dz = 0 vf = 0
        // so LT_tm reduces to X + av_tm(z_j).
        let z_j = 5.0;
        let x = 60.0;
        let lt = individual_threshold_tonal(z_j, z_j, x).unwrap();
        let want = x + masking_index_tonal(z_j);
        assert!((lt - want).abs() < 1e-12);
    }

    #[test]
    fn individual_threshold_non_tonal_combines_x_av_vf() {
        let z_j = 7.0;
        let x = 50.0;
        let lt = individual_threshold_non_tonal(z_j, z_j, x).unwrap();
        let want = x + masking_index_non_tonal(z_j);
        assert!((lt - want).abs() < 1e-12);
    }

    #[test]
    fn individual_threshold_outside_window_is_none() {
        assert!(individual_threshold_tonal(2.0, 12.0, 60.0).is_none());
        assert!(individual_threshold_non_tonal(2.0, 12.0, 60.0).is_none());
    }

    // -----------------------------------------------------------
    // Step 7 — global threshold (power-domain sum)
    // -----------------------------------------------------------

    #[test]
    fn global_threshold_no_maskers_returns_ltq() {
        // No tonal, no non-tonal -> LTg = LTq.
        for &ltq in &[-20.0f64, 0.0, 30.0, 60.0] {
            let lt = global_threshold_db(ltq, &[], &[]);
            assert!((lt - ltq).abs() < 1e-12);
        }
    }

    #[test]
    fn global_threshold_two_equal_db_powers_sum_to_3_db() {
        // Two equal-power sources sum to +3.0103 dB in the
        // power-domain (10*log10(2) ≈ 3.0103).
        let ltq = -200.0; // negligible; pushes 10^(-200/10) ~ 0
        let lt = global_threshold_db(ltq, &[40.0], &[40.0]);
        assert!((lt - (40.0 + 10.0 * 2f64.log10())).abs() < 1e-9);
    }

    #[test]
    fn global_threshold_dominated_by_loudest_term() {
        // Loudest single term dominates the sum; result must be just
        // slightly above max term but well below max + 3 dB if the
        // smaller terms are 20+ dB lower.
        let ltq = -10.0;
        let tonal = [80.0, 50.0];
        let non_tonal = [55.0, 20.0];
        let lt = global_threshold_db(ltq, &tonal, &non_tonal);
        assert!(lt > 80.0);
        assert!(lt < 80.5);
    }

    // -----------------------------------------------------------
    // Table D.5 — coder partition table
    // -----------------------------------------------------------

    #[test]
    fn coder_partitions_have_33_rows() {
        assert_eq!(CODER_PARTITIONS.len(), 33);
    }

    #[test]
    fn coder_partitions_boundary_endpoints() {
        assert_eq!(CODER_PARTITIONS[0].boundary, 1);
        assert_eq!(CODER_PARTITIONS[32].boundary, 513);
    }

    #[test]
    fn coder_partitions_widths_change_at_index_13() {
        // Partitions 0..12 use width 0; 13..32 use width 1.
        for (i, p) in CODER_PARTITIONS.iter().enumerate() {
            let want = if i < 13 { 0 } else { 1 };
            assert_eq!(p.width, want, "partition {i}");
        }
    }

    #[test]
    fn coder_partitions_strictly_monotonic_boundaries() {
        for w in CODER_PARTITIONS.windows(2) {
            assert!(w[1].boundary > w[0].boundary);
        }
    }

    #[test]
    fn coder_partitions_boundary_step_is_sixteen_from_index_one() {
        // From n = 1 the boundary advances by 16 each step
        // (1, 17, 33, 49, …, 513).
        for w in CODER_PARTITIONS[1..].windows(2) {
            assert_eq!(w[1].boundary - w[0].boundary, 16);
        }
    }

    // -----------------------------------------------------------
    // Annex D Step 3 — LTq bit-rate offset
    // -----------------------------------------------------------

    #[test]
    fn ltq_offset_below_96_kbps_is_zero() {
        // Per-channel rates strictly below 96 kbit/s get a 0 dB offset.
        for &kbps in &[8u32, 16, 32, 48, 64, 80, 95] {
            assert_eq!(
                ltq_offset_db(kbps),
                0.0,
                "per-channel rate {kbps} kbps should map to 0 dB offset"
            );
        }
    }

    #[test]
    fn ltq_offset_at_and_above_96_kbps_is_minus_twelve() {
        // The boundary value 96 kbit/s and every per-channel rate
        // above it get -12 dB. The spec wording is ">= 96 kbits/s".
        for &kbps in &[96u32, 112, 128, 160, 192, 224, 256, 320, 384, 448] {
            assert_eq!(
                ltq_offset_db(kbps),
                -12.0,
                "per-channel rate {kbps} kbps should map to -12 dB offset"
            );
        }
    }

    #[test]
    fn ltq_offset_boundary_is_inclusive_at_96() {
        // The spec wording draws the line at ">= 96 kbits/s"
        // (inclusive), not ">" — 95 stays at 0 dB and 96 jumps to
        // -12 dB, which is the entire spec-relevant edge case.
        assert_eq!(ltq_offset_db(95), 0.0);
        assert_eq!(ltq_offset_db(96), -12.0);
    }

    // -----------------------------------------------------------
    // Annex D clause D.2 — Model 2 spreading function (text pieces)
    // -----------------------------------------------------------

    #[test]
    fn model2_tmpx_matches_closed_form() {
        // tmpx = 1.05 * (j - i); sign follows the literal (j - i) order.
        for &(j, i) in &[(0.0f64, 0.0), (5.0, 3.0), (3.0, 5.0), (12.5, 8.0)] {
            let got = model2_tmpx(j, i);
            let want = 1.05 * (j - i);
            assert!(
                (got - want).abs() < 1e-12,
                "model2_tmpx({j}, {i}) = {got}, expected {want}"
            );
        }
    }

    #[test]
    fn model2_tmpx_sign_matches_spec_order() {
        // j > i  -> tmpx > 0; j < i -> tmpx < 0; j == i -> 0.
        assert!(model2_tmpx(5.0, 3.0) > 0.0);
        assert!(model2_tmpx(3.0, 5.0) < 0.0);
        assert!((model2_tmpx(4.0, 4.0) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn model2_x_peak_at_tmpx_one_half() {
        // x = 8 * min((tmpx-0.5)^2 - 2*(tmpx-0.5), 0). At tmpx = 0.5
        // the inner expression is 0, so x = 0 (the peak — x is clamped
        // to non-positive).
        assert!((model2_x(0.5) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn model2_x_zero_again_at_tmpx_two_and_a_half() {
        // At tmpx = 2.5 the inner s = 2.0, s^2 - 2s = 4 - 4 = 0. The
        // clamp keeps the value at 0 (the inner is 0, not negative,
        // so min(0, 0) = 0).
        assert!((model2_x(2.5) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn model2_x_is_always_non_positive() {
        // The min(_, 0) clamp guarantees x <= 0 everywhere.
        for &tmpx in &[-3.0f64, -1.0, 0.0, 0.5, 1.0, 1.5, 2.5, 3.0, 5.0, 10.0] {
            assert!(
                model2_x(tmpx) <= 0.0,
                "model2_x({tmpx}) = {} should be <= 0",
                model2_x(tmpx)
            );
        }
    }

    #[test]
    fn model2_x_matches_closed_form_when_inner_negative() {
        // For tmpx in the (0.5, 2.5) open interval the inner term
        // (s^2 - 2s) is negative, so the clamp is inactive and x =
        // 8 * (s^2 - 2s).
        for &tmpx in &[0.6f64, 1.0, 1.5, 2.0, 2.4] {
            let s = tmpx - 0.5;
            let want = 8.0 * (s * s - 2.0 * s);
            let got = model2_x(tmpx);
            assert!(
                (got - want).abs() < 1e-12,
                "model2_x({tmpx}) = {got}, expected {want}"
            );
            // And the value must be strictly negative in this interval.
            assert!(got < 0.0, "expected x < 0 inside (0.5, 2.5)");
        }
    }

    #[test]
    fn model2_x_clamps_to_zero_below_tmpx_one_half() {
        // For tmpx < 0.5, s < 0, so s^2 - 2s = s^2 + 2|s| > 0; the
        // clamp activates and x = 0.
        for &tmpx in &[-5.0f64, -2.0, -1.0, 0.0, 0.49] {
            assert_eq!(
                model2_x(tmpx),
                0.0,
                "model2_x({tmpx}) should clamp to 0 below tmpx=0.5"
            );
        }
    }

    #[test]
    fn model2_x_clamps_to_zero_above_tmpx_two_and_a_half() {
        // For tmpx > 2.5, s > 2, so s^2 - 2s = s*(s-2) > 0; clamp -> 0.
        for &tmpx in &[2.51f64, 3.0, 5.0, 10.0, 100.0] {
            assert_eq!(
                model2_x(tmpx),
                0.0,
                "model2_x({tmpx}) should clamp to 0 above tmpx=2.5"
            );
        }
    }

    #[test]
    fn sprdngf_from_tmpy_returns_zero_below_minus_100() {
        // tmpy strictly below -100 dB -> sprdngf = 0 (spec cutoff).
        for &tmpy in &[-101.0f64, -150.0, -1000.0, f64::MIN] {
            assert_eq!(
                sprdngf_from_tmpy(tmpy),
                0.0,
                "sprdngf_from_tmpy({tmpy}) should cut off to 0"
            );
        }
    }

    #[test]
    fn sprdngf_from_tmpy_at_minus_100_is_active() {
        // tmpy = -100 (boundary) is "not less than -100", so the
        // 10^(tmpy/10) branch runs and produces 10^-10 — small but
        // strictly positive.
        let v = sprdngf_from_tmpy(-100.0);
        assert!(v > 0.0, "sprdngf at boundary tmpy = -100 should be > 0");
        assert!((v - 1e-10).abs() < 1e-14);
    }

    #[test]
    fn sprdngf_from_tmpy_matches_power_conversion() {
        // sprdngf = 10^(tmpy/10) for tmpy >= -100. Spot checks at a
        // few legible levels.
        for &(tmpy, want) in &[
            (0.0f64, 1.0f64),
            (-10.0, 0.1),
            (-20.0, 0.01),
            (-30.0, 0.001),
            (10.0, 10.0),
        ] {
            let got = sprdngf_from_tmpy(tmpy);
            assert!(
                (got - want).abs() < 1e-12,
                "sprdngf_from_tmpy({tmpy}) = {got}, expected {want}"
            );
        }
    }

    #[test]
    fn sprdngf_from_tmpy_is_strictly_monotone_in_tmpy() {
        // Inside the active region (tmpy >= -100) the conversion is
        // 10^(tmpy/10), strictly monotone in tmpy.
        let xs = [-100.0f64, -50.0, -10.0, 0.0, 5.0, 10.0, 20.0];
        for w in xs.windows(2) {
            assert!(
                sprdngf_from_tmpy(w[1]) > sprdngf_from_tmpy(w[0]),
                "sprdngf not monotone at {} -> {}",
                w[0],
                w[1]
            );
        }
    }

    // ---- Annex D Table D.3a complete table (Fs = 32 kHz) ----

    #[test]
    fn calc_partition_32k_has_forty_nine_rows() {
        // The staged docs extract now transcribes the complete
        // 32 kHz partition table (D.3a): 49 rows, corrected from
        // the earlier 63 miscount per `docs` #129. The constant
        // length and `_FULL_LEN` (`bmax`) must both be 49.
        assert_eq!(CALC_PARTITION_32K.len(), 49);
        assert_eq!(CALC_PARTITION_32K_FULL_LEN, 49);
    }

    #[test]
    fn calc_partition_32k_indices_are_one_based_and_dense() {
        // Spec column 1 ("Index") is 1-based and dense over the
        // full table 1..=49 with no gaps; the `index` field
        // stamped into each row must match its array slot's
        // 1-based counterpart.
        for (slot, row) in CALC_PARTITION_32K.iter().enumerate() {
            let expected = (slot + 1) as u16;
            assert_eq!(
                row.index,
                expected,
                "row at slot {slot} carries index {got}, expected {expected}",
                got = row.index
            );
        }
    }

    #[test]
    fn calc_partition_32k_omega_ranges_are_contiguous_to_nyquist() {
        // The spec's per-partition FFT-line spans tile the line
        // axis with no overlap and no gap: the first row anchors
        // at line 1, each `omega_low` equals the previous row's
        // `omega_high + 1`, and the final partition's `omega_high`
        // reaches FFT line 513 — the Nyquist line of the
        // 1024-point Model 2 analysis FFT.
        assert_eq!(CALC_PARTITION_32K[0].omega_low, 1);
        for w in CALC_PARTITION_32K.windows(2) {
            let prev = w[0];
            let next = w[1];
            assert_eq!(
                next.omega_low,
                prev.omega_high + 1,
                "discontinuity between partition {} (omega_high = {}) and {} (omega_low = {})",
                prev.index,
                prev.omega_high,
                next.index,
                next.omega_low
            );
            assert!(
                prev.omega_high >= prev.omega_low,
                "partition {} has inverted omega range [{}, {}]",
                prev.index,
                prev.omega_low,
                prev.omega_high
            );
        }
        assert_eq!(
            CALC_PARTITION_32K[48].omega_high, 513,
            "last partition must reach the Nyquist FFT line 513"
        );
    }

    #[test]
    fn calc_partition_32k_widths_match_omega_span() {
        // The `width()` accessor must equal `omega_high - omega_low + 1`
        // for every row.
        for row in CALC_PARTITION_32K.iter() {
            assert_eq!(
                row.width(),
                row.omega_high - row.omega_low + 1,
                "width mismatch on partition {}",
                row.index
            );
        }
    }

    #[test]
    fn calc_partition_32k_first_and_last_rows_match_spec() {
        // Spec anchors at both ends of the table:
        //   partition 1  spans line ω = 1, bval 0,00, minval 0,0,
        //                TMN 24,5;
        //   partition 49 spans [497, 513], bval 24,07, minval 4,5,
        //                TMN 38,6 (the highest TMN in the table).
        let p1 = CALC_PARTITION_32K[0];
        assert_eq!(p1.index, 1);
        assert_eq!(p1.omega_low, 1);
        assert_eq!(p1.omega_high, 1);
        assert!((p1.bval - 0.00).abs() < 1e-9);
        assert!((p1.minval - 0.0).abs() < 1e-9);
        assert!((p1.tmn - 24.5).abs() < 1e-9);

        let p49 = CALC_PARTITION_32K[48];
        assert_eq!(p49.index, 49);
        assert_eq!(p49.omega_low, 497);
        assert_eq!(p49.omega_high, 513);
        assert!((p49.bval - 24.07).abs() < 1e-9);
        assert!((p49.minval - 4.5).abs() < 1e-9);
        assert!((p49.tmn - 38.6).abs() < 1e-9);
    }

    #[test]
    fn calc_partition_32k_bval_is_strictly_increasing() {
        // Bark values are monotonically increasing across the
        // partition axis (the Bark scale is monotonic in
        // frequency, and the partitions tile by ascending FFT
        // line); strict over the whole 49-row table.
        for w in CALC_PARTITION_32K.windows(2) {
            assert!(
                w[1].bval > w[0].bval,
                "bval not strictly increasing at partition {} -> {}",
                w[0].index,
                w[1].index
            );
        }
    }

    #[test]
    fn calc_partition_32k_tmn_constant_then_rises_at_partition_15() {
        // Partitions 1..=14 hold TMN at 24,5 dB; from partition 15
        // (10,28 Bark, TMN = 24,8 dB) the TMN rises monotonically
        // through to 38,6 dB at partition 49.
        for row in CALC_PARTITION_32K.iter().take(14) {
            assert!(
                (row.tmn - 24.5).abs() < 1e-9,
                "partition {} (head region) carries TMN = {}, expected 24.5",
                row.index,
                row.tmn
            );
        }
        // Partitions 15..=49 carry monotonically non-decreasing
        // TMN (the printed table has a few equal-to-one-decimal
        // adjacent steps, so non-strict).
        let tail: Vec<f64> = CALC_PARTITION_32K[14..].iter().map(|r| r.tmn).collect();
        for w in tail.windows(2) {
            assert!(
                w[1] >= w[0],
                "TMN must be non-decreasing in the tail: {} -> {}",
                w[0],
                w[1]
            );
        }
        // The partition-15 anchor and the partition-49 endpoint
        // land exactly per the printed table.
        assert!((CALC_PARTITION_32K[14].tmn - 24.8).abs() < 1e-9);
        assert!((CALC_PARTITION_32K[48].tmn - 38.6).abs() < 1e-9);
    }

    #[test]
    fn calc_partition_32k_minval_settles_to_four_point_five_from_partition_17() {
        // Per the printed table: from partition 17 onward minval is
        // a constant 4,5 dB. Check explicitly for partitions
        // 17..=49 (the whole settled tail).
        for row in CALC_PARTITION_32K.iter().filter(|r| r.index >= 17) {
            assert!(
                (row.minval - 4.5).abs() < 1e-9,
                "partition {} (settled region) carries minval = {}, expected 4.5",
                row.index,
                row.minval
            );
        }
    }

    #[test]
    fn calc_partition_32k_lookup_resolves_every_partition() {
        // The `calc_partition_32k(n)` helper resolves 1-based `n`
        // into the complete table. Every index 1..=49 returns
        // `Some(row)` with a matching `index`; the `n == 0`
        // underflow and the `n > 49` out-of-range both return
        // `None`.
        assert!(calc_partition_32k(0).is_none());
        for n in 1..=CALC_PARTITION_32K_FULL_LEN as u16 {
            let row = calc_partition_32k(n).expect("partition resolves");
            assert_eq!(row.index, n);
        }
        assert!(calc_partition_32k(50).is_none());
        assert!(calc_partition_32k(63).is_none());
    }

    #[test]
    fn calc_partition_32k_widths_match_extract() {
        // Width-by-width regression against the printed extract:
        // partition 1 spans 1 line, partitions 2..=13 span 3 lines,
        // partitions 14..=29 span 4..=9 lines (widening as the Bark
        // scale exits the linear regime). Sum of widths covers the
        // full FFT line span 1..=513.
        let widths: Vec<u16> = CALC_PARTITION_32K.iter().map(|r| r.width()).collect();
        assert_eq!(widths[0], 1, "partition 1 spans one FFT line");
        for w in &widths[1..=12] {
            assert_eq!(*w, 3, "partitions 2..=13 span three FFT lines");
        }
        // Total FFT-line coverage equals the Nyquist line count.
        let total: u32 = widths.iter().map(|&w| u32::from(w)).sum();
        assert_eq!(total, 513, "partition widths must tile FFT lines 1..=513");
    }

    // -----------------------------------------------------------
    // Step 4 helper — `critical_band_for_line`
    // -----------------------------------------------------------

    #[test]
    fn critical_band_for_line_rejects_zero_line() {
        for &(layer, fs) in &[
            (Layer::I, 32_000u32),
            (Layer::I, 44_100),
            (Layer::I, 48_000),
            (Layer::II, 32_000),
            (Layer::II, 44_100),
            (Layer::II, 48_000),
        ] {
            assert!(
                critical_band_for_line(layer, fs, 0).is_none(),
                "(layer={layer:?}, fs={fs}) line 0 must be rejected (spec is 1-based)"
            );
        }
    }

    #[test]
    fn critical_band_for_line_rejects_unsupported_rates() {
        for &fs in &[16_000u32, 22_050, 24_000, 8_000, 11_025, 12_000] {
            assert!(critical_band_for_line(Layer::I, fs, 1).is_none());
            assert!(critical_band_for_line(Layer::II, fs, 1).is_none());
        }
    }

    #[test]
    fn critical_band_for_line_returns_none_above_top() {
        // The Table D.2x boundary list's last row's `index_fcb` is the
        // highest FFT line covered. A line above that lies outside the
        // tabulated audio band.
        for &(layer, fs) in &[
            (Layer::I, 32_000u32),
            (Layer::I, 44_100),
            (Layer::I, 48_000),
            (Layer::II, 32_000),
            (Layer::II, 44_100),
            (Layer::II, 48_000),
        ] {
            let bands = critical_band_table(layer, fs).unwrap();
            let top = bands.last().unwrap().index_fcb;
            assert!(critical_band_for_line(layer, fs, top + 1).is_none());
            assert!(critical_band_for_line(layer, fs, u16::MAX).is_none());
        }
    }

    #[test]
    fn critical_band_for_line_lookup_matches_boundary_walk() {
        // Brute-force cross-check: for every FFT line up to and
        // including the highest tabulated index_fcb, the lookup
        // returns the smallest band whose top index_fcb >= line.
        for &(layer, fs) in &[
            (Layer::I, 32_000u32),
            (Layer::I, 44_100),
            (Layer::I, 48_000),
            (Layer::II, 32_000),
            (Layer::II, 44_100),
            (Layer::II, 48_000),
        ] {
            let bands = critical_band_table(layer, fs).unwrap();
            let top = bands.last().unwrap().index_fcb;
            for line in 1..=top {
                let expect = bands
                    .iter()
                    .position(|b| b.index_fcb >= line)
                    .expect("line within top must hit a band");
                assert_eq!(
                    critical_band_for_line(layer, fs, line),
                    Some(expect),
                    "mismatch at (layer={layer:?}, fs={fs}, line={line})"
                );
            }
        }
    }

    #[test]
    fn critical_band_for_line_top_edge_belongs_to_band_k_not_k_plus_1() {
        // The Table D.2x `index_fcb` column is the **upper** edge of
        // each band, so a line equal to a band's top must map to
        // that band (k) not the next one (k+1).
        for &(layer, fs) in &[
            (Layer::I, 32_000u32),
            (Layer::I, 44_100),
            (Layer::I, 48_000),
            (Layer::II, 32_000),
            (Layer::II, 44_100),
            (Layer::II, 48_000),
        ] {
            let bands = critical_band_table(layer, fs).unwrap();
            for (k, band) in bands.iter().enumerate() {
                assert_eq!(
                    critical_band_for_line(layer, fs, band.index_fcb),
                    Some(k),
                    "(layer={layer:?}, fs={fs}) line {} (top of band {k}) must map to band {k}",
                    band.index_fcb
                );
            }
        }
    }

    #[test]
    fn critical_band_for_line_d2a_known_anchors() {
        // Layer I, 32 kHz: band 0 covers FFT line 1 only
        // (band 0 top = 1); band 1 covers lines 2..=3 (band 1 top =
        // 3); band 2 covers lines 4..=5; band 23 covers lines
        // 95..=108. These anchors come straight from the D.2a
        // boundary list.
        assert_eq!(
            critical_band_for_line(Layer::I, 32_000, 1),
            Some(0),
            "line 1 falls in band 0 (top index_fcb = 1)"
        );
        assert_eq!(
            critical_band_for_line(Layer::I, 32_000, 2),
            Some(1),
            "line 2 falls in band 1 (top index_fcb = 3)"
        );
        assert_eq!(
            critical_band_for_line(Layer::I, 32_000, 3),
            Some(1),
            "line 3 falls in band 1 (top index_fcb = 3)"
        );
        assert_eq!(
            critical_band_for_line(Layer::I, 32_000, 4),
            Some(2),
            "line 4 falls in band 2 (top index_fcb = 5)"
        );
        assert_eq!(
            critical_band_for_line(Layer::I, 32_000, 95),
            Some(23),
            "line 95 falls in band 23 (top index_fcb = 108)"
        );
        assert_eq!(
            critical_band_for_line(Layer::I, 32_000, 108),
            Some(23),
            "line 108 is the top of band 23"
        );
    }

    #[test]
    fn critical_band_for_line_d2e_layer2_44k1_anchors() {
        // Layer II, 44.1 kHz: a few midband + top anchors against the
        // staged D.2e boundaries (bands 0/1/2 = lines up to 1/2/3,
        // last band 26 = line 130).
        assert_eq!(critical_band_for_line(Layer::II, 44_100, 1), Some(0));
        assert_eq!(critical_band_for_line(Layer::II, 44_100, 2), Some(1));
        assert_eq!(critical_band_for_line(Layer::II, 44_100, 3), Some(2));
        assert_eq!(critical_band_for_line(Layer::II, 44_100, 130), Some(26));
        assert!(critical_band_for_line(Layer::II, 44_100, 131).is_none());
    }

    // -----------------------------------------------------------
    // Step 3 — `step3_apply_ltq_offset`
    // -----------------------------------------------------------

    #[test]
    fn step3_apply_ltq_offset_high_rate_subtracts_twelve_db() {
        // ≥ 96 kbit/s per channel → -12 dB offset added to LTq.
        let tabulated = 33.44; // Table D.1a row 1 anchor value.
        assert!((step3_apply_ltq_offset(tabulated, 96) - 21.44).abs() < 1e-9);
        assert!((step3_apply_ltq_offset(tabulated, 192) - 21.44).abs() < 1e-9);
        assert!((step3_apply_ltq_offset(tabulated, 448) - 21.44).abs() < 1e-9);
    }

    #[test]
    fn step3_apply_ltq_offset_low_rate_passes_through() {
        // < 96 kbit/s per channel → 0 dB offset; LTq passes through.
        let tabulated = 33.44;
        for kbps in [8u32, 32, 64, 80, 95] {
            assert!((step3_apply_ltq_offset(tabulated, kbps) - tabulated).abs() < 1e-9);
        }
    }

    #[test]
    fn step3_apply_ltq_offset_boundary_at_96_inclusive() {
        // The boundary is inclusive on the -12 dB side: exactly 96
        // already counts as "≥ 96", matching ltq_offset_db's
        // documented semantics.
        let tabulated = -4.97; // i ≈ 51, near the LTq minimum.
        assert!((step3_apply_ltq_offset(tabulated, 95) - tabulated).abs() < 1e-9);
        assert!((step3_apply_ltq_offset(tabulated, 96) - (tabulated - 12.0)).abs() < 1e-9);
    }

    // -----------------------------------------------------------
    // Step 7 — `global_threshold_db_from_maskers`
    // -----------------------------------------------------------

    #[test]
    fn global_threshold_db_from_maskers_empty_yields_ltq() {
        // No maskers → the per-line LTq is the global threshold.
        let v = global_threshold_db_from_maskers(8.0, -10.0, &[], &[]);
        assert!((v - -10.0).abs() < 1e-9);
    }

    #[test]
    fn global_threshold_db_from_maskers_drops_out_of_window_maskers() {
        // dz = z_i - z_j: a masker at z_j = 0 evaluated at z_i = 9
        // (dz = +9) is outside the [-3, 8) vf window and must be
        // dropped. The remaining call should match an empty-masker
        // call.
        let z_i = 9.0;
        let ltq = 0.0;
        let v_with = global_threshold_db_from_maskers(z_i, ltq, &[(0.0, 60.0)], &[]);
        let v_without = global_threshold_db_from_maskers(z_i, ltq, &[], &[]);
        assert!((v_with - v_without).abs() < 1e-9);
    }

    #[test]
    fn global_threshold_db_from_maskers_matches_pre_filtered_sum() {
        // Cross-check against the existing `global_threshold_db`
        // (which takes pre-filtered LT slices) on a contrived
        // masker set entirely inside the window.
        let z_i = 8.0;
        let ltq = -5.0;
        let tonal_maskers = [(7.5, 70.0), (8.2, 65.0)];
        let non_tonal_maskers = [(7.0, 55.0)];

        let lt_tm: Vec<f64> = tonal_maskers
            .iter()
            .map(|&(z_j, x)| individual_threshold_tonal(z_j, z_i, x).unwrap())
            .collect();
        let lt_nm: Vec<f64> = non_tonal_maskers
            .iter()
            .map(|&(z_j, x)| individual_threshold_non_tonal(z_j, z_i, x).unwrap())
            .collect();
        let expect = global_threshold_db(ltq, &lt_tm, &lt_nm);

        let got = global_threshold_db_from_maskers(z_i, ltq, &tonal_maskers, &non_tonal_maskers);
        assert!(
            (got - expect).abs() < 1e-9,
            "expected {expect:.6}, got {got:.6}"
        );
    }

    #[test]
    fn global_threshold_db_from_maskers_mixed_in_and_out_of_window() {
        // One in-window tonal masker, one out-of-window tonal
        // masker, one in-window non-tonal masker. The result must
        // equal the same call with the out-of-window entry removed.
        let z_i = 10.0;
        let ltq = -8.0;
        let with_outlier = global_threshold_db_from_maskers(
            z_i,
            ltq,
            &[(9.5, 72.0), (0.0, 90.0)], // second masker has dz = 10, outside [-3, 8).
            &[(10.2, 50.0)],
        );
        let only_inside =
            global_threshold_db_from_maskers(z_i, ltq, &[(9.5, 72.0)], &[(10.2, 50.0)]);
        assert!((with_outlier - only_inside).abs() < 1e-9);
    }

    #[test]
    fn global_threshold_db_from_maskers_strong_masker_dominates() {
        // A loud in-window masker pushes LTg(i) well above LTq.
        let z_i = 8.0;
        let ltq = 0.0;
        let weak = global_threshold_db_from_maskers(z_i, ltq, &[], &[]);
        let strong = global_threshold_db_from_maskers(z_i, ltq, &[(8.0, 80.0)], &[]);
        assert!(
            strong > weak + 30.0,
            "loud masker (X=80 dB) should raise LTg by ≫ 30 dB; got weak={weak:.2} strong={strong:.2}"
        );
    }

    // -----------------------------------------------------------
    // Model 2 spreading function — `x` term per (j_bark, i_bark)
    // -----------------------------------------------------------

    #[test]
    fn model2_x_for_pair_matches_two_step_call() {
        // The `_for_pair` adapter is defined as the composition of
        // `model2_tmpx` and `model2_x` — verify literal agreement on
        // a handful of `(j, i)` pairs spanning the relevant regions.
        for &(j, i) in &[
            (0.0f64, 0.0), // tmpx = 0  -> clamp -> 0
            (0.5, 0.0),    // tmpx ≈ 0.525, near peak edge (still 0)
            (1.0, 0.0),    // tmpx = 1.05, inside (0.5, 2.5) — active
            (2.0, 0.5),    // tmpx ≈ 1.575, active
            (2.5, 0.0),    // tmpx ≈ 2.625, outside (clamp -> 0)
            (3.0, 5.0),    // tmpx < 0, clamp -> 0
            (10.0, 8.0),   // tmpx = 2.1, active
        ] {
            let want = model2_x(model2_tmpx(j, i));
            let got = model2_x_for_pair(j, i);
            assert!(
                (got - want).abs() < 1e-12,
                "model2_x_for_pair({j}, {i}) = {got}, two-step = {want}"
            );
        }
    }

    #[test]
    fn model2_x_for_pair_is_non_positive_everywhere() {
        // The `min(_, 0)` clamp inside `model2_x` propagates through
        // the adapter; the per-pair value is `<= 0` for any input.
        for j in -5..=20 {
            for i in -5..=20 {
                let (jb, ib) = (j as f64, i as f64);
                let v = model2_x_for_pair(jb, ib);
                assert!(
                    v <= 0.0,
                    "model2_x_for_pair({jb}, {ib}) = {v} should be <= 0"
                );
            }
        }
    }

    #[test]
    fn model2_x_for_pair_zero_when_j_equals_i() {
        // j == i  →  tmpx = 0  →  inner = 0.25 > 0  →  clamp -> 0.
        for &z in &[0.0f64, 1.5, 5.0, 12.0, 23.5] {
            assert_eq!(model2_x_for_pair(z, z), 0.0);
        }
    }

    #[test]
    fn model2_x_for_pair_active_inside_bark_window() {
        // tmpx is in (0.5, 2.5) iff (j - i) is in
        // (0.5/1.05, 2.5/1.05) ≈ (0.47619, 2.38095). Inside that
        // window the `x` term must be strictly negative.
        for &dz in &[0.48f64, 0.6, 1.0, 1.5, 2.0, 2.37] {
            let v = model2_x_for_pair(dz, 0.0);
            assert!(
                v < 0.0,
                "model2_x_for_pair with j-i = {dz} should be < 0, got {v}"
            );
        }
    }

    #[test]
    fn model2_x_for_pair_zero_outside_bark_window() {
        // Outside the (≈0.476, ≈2.381) (j - i) window the inner term
        // is non-negative so the clamp engages and the value is
        // exactly 0. Cover both sides plus the closed boundaries.
        for &dz in &[-5.0f64, -1.0, 0.0, 0.4, 0.47619047, 2.38095239, 2.5, 5.0] {
            assert_eq!(
                model2_x_for_pair(dz, 0.0),
                0.0,
                "expected exact 0 for (j - i) = {dz}"
            );
        }
    }

    #[test]
    fn model2_x_is_active_predicate_matches_negative_region() {
        // The predicate is `true` iff `model2_x_for_pair` is strictly
        // negative — they must agree as boolean / numeric forms of
        // the same condition. Sweep a fine grid of (j - i) values.
        let step = 0.05;
        let mut dz = -3.0;
        while dz <= 3.0 + step / 2.0 {
            let active = model2_x_is_active(dz, 0.0);
            let v = model2_x_for_pair(dz, 0.0);
            assert_eq!(
                active,
                v < 0.0,
                "predicate {active} but value {v} at (j - i) = {dz}"
            );
            dz += step;
        }
    }

    #[test]
    fn model2_x_is_active_endpoints_are_exclusive() {
        // tmpx == 0.5 -> inner = 0 -> v == 0 -> predicate false.
        // tmpx == 2.5 -> inner = 0 -> v == 0 -> predicate false.
        // The matching (j - i) values are 0.5/1.05 and 2.5/1.05.
        let dz_low = 0.5 / 1.05;
        let dz_high = 2.5 / 1.05;
        assert!(!model2_x_is_active(dz_low, 0.0));
        assert!(!model2_x_is_active(dz_high, 0.0));
        // Just inside the open interval the predicate must be true.
        assert!(model2_x_is_active(dz_low + 1e-9, 0.0));
        assert!(model2_x_is_active(dz_high - 1e-9, 0.0));
    }

    #[test]
    fn model2_x_is_active_false_when_j_equals_i() {
        // j == i -> tmpx = 0 -> outside (0.5, 2.5) -> not active.
        for &z in &[0.0f64, 1.5, 5.0, 12.0, 23.5] {
            assert!(
                !model2_x_is_active(z, z),
                "model2_x_is_active({z}, {z}) should be false"
            );
        }
    }

    #[test]
    fn model2_x_is_active_false_for_j_below_i() {
        // tmpx = 1.05 * (j - i); when j < i the value is negative
        // and certainly outside (0.5, 2.5).
        for &(j, i) in &[(0.0f64, 1.0), (3.0, 5.0), (5.0, 12.5), (1.0, 10.0)] {
            assert!(!model2_x_is_active(j, i));
        }
    }

    // -----------------------------------------------------------
    // Clause D.2.3 — `tmpy` backbone + full `sprdngf` composition
    // -----------------------------------------------------------

    #[test]
    fn model2_tmpy_matches_printed_form_at_spot_values() {
        // Direct evaluation of the printed line
        //   tmpy = 15,811389 + 7,5(tmpx + 0,474)
        //          - 17,5(1,0 + (tmpx + 0,474)^2)^0,5
        // at a handful of points, computed independently here.
        for &tmpx in &[-6.0f64, -2.0, -0.474, 0.0, 1.0, 2.5, 8.0, 12.0] {
            let u = tmpx + 0.474;
            let expect = 15.811_389 + 7.5 * u - 17.5 * (1.0 + u * u).sqrt();
            let got = model2_tmpy(tmpx);
            assert!(
                (got - expect).abs() < 1e-12,
                "tmpy({tmpx}) = {got}, expected {expect}"
            );
        }
        // u = 0 collapses the square root to 1:
        // tmpy(-0.474) = 15.811389 - 17.5 = -1.688611 exactly.
        assert!((model2_tmpy(-0.474) - (-1.688_611)).abs() < 1e-12);
    }

    #[test]
    fn model2_tmpy_peaks_at_zero_db_near_tmpx_zero() {
        // The stationary point is at u0 = 3/sqrt(40) (where
        // 7.5 = 17.5 * u/sqrt(1+u^2)), i.e. tmpx = u0 - 0.474 ~= 0.
        // The printed constant 15.811389 equals
        // 17.5*sqrt(1 + u0^2) - 7.5*u0 to within 1e-6, so the
        // maximum of the backbone is ~0 dB.
        let u0 = 3.0 / 40.0f64.sqrt();
        let peak = model2_tmpy(u0 - 0.474);
        assert!(peak.abs() < 1e-5, "peak {peak} not ~0 dB");
        // ... and tmpy never exceeds ~0 anywhere on a wide grid.
        let mut tmpx = -30.0;
        while tmpx <= 30.0 {
            assert!(
                model2_tmpy(tmpx) <= 1e-5,
                "tmpy({tmpx}) = {} > 0",
                model2_tmpy(tmpx)
            );
            tmpx += 0.01;
        }
        // tmpx = 0 itself is within 1e-6 of the peak.
        assert!(model2_tmpy(0.0).abs() < 1e-6);
    }

    #[test]
    fn model2_tmpy_unimodal_around_the_peak() {
        // Strictly increasing left of the stationary point, strictly
        // decreasing right of it.
        let t0 = 3.0 / 40.0f64.sqrt() - 0.474;
        let step = 0.05;
        let mut tmpx = -20.0;
        while tmpx + step <= t0 {
            assert!(
                model2_tmpy(tmpx) < model2_tmpy(tmpx + step),
                "not increasing at tmpx = {tmpx}"
            );
            tmpx += step;
        }
        let mut tmpx = t0 + step;
        while tmpx + step <= 20.0 {
            assert!(
                model2_tmpy(tmpx) > model2_tmpy(tmpx + step),
                "not decreasing at tmpx = {tmpx}"
            );
            tmpx += step;
        }
    }

    #[test]
    fn model2_tmpy_asymptote_slopes() {
        // u -> -inf: sqrt(1 + u^2) -> -u, so dtmpy/dtmpx -> 7.5 +
        // 17.5 = 25 dB per tmpx unit. u -> +inf: -> 7.5 - 17.5 =
        // -10 dB per tmpx unit.
        let left = model2_tmpy(-40.0) - model2_tmpy(-41.0);
        assert!((left - 25.0).abs() < 0.01, "left slope {left}");
        let right = model2_tmpy(41.0) - model2_tmpy(40.0);
        assert!((right - (-10.0)).abs() < 0.01, "right slope {right}");
    }

    #[test]
    fn model2_sprdngf_is_one_at_j_equals_i() {
        // tmpx = 0 -> x = 0 (clamp engaged), tmpy ~= 0 -> 10^~0 ~= 1.
        for &z in &[0.0f64, 1.5, 5.0, 12.0, 23.5] {
            let v = model2_sprdngf(z, z);
            assert!((v - 1.0).abs() < 1e-6, "sprdngf({z}, {z}) = {v}");
        }
    }

    #[test]
    fn model2_sprdngf_matches_term_by_term_composition() {
        // Recompose the printed chain from the staged pieces and
        // check agreement across a grid covering both branches.
        let mut dz = -8.0;
        while dz <= 14.0 {
            let j = dz;
            let i = 0.0;
            let tmpx = model2_tmpx(j, i);
            let tmpy = model2_tmpy(tmpx);
            let expect = if tmpy < -100.0 {
                0.0
            } else {
                10f64.powf((model2_x(tmpx) + tmpy) / 10.0)
            };
            let got = model2_sprdngf(j, i);
            assert!(
                (got - expect).abs() <= 1e-15 * expect.abs().max(1.0),
                "sprdngf mismatch at dz = {dz}: {got} vs {expect}"
            );
            dz += 0.03;
        }
    }

    #[test]
    fn model2_sprdngf_never_amplifies() {
        // x <= 0 and tmpy <= ~1e-6 => sprdngf <= 10^(1e-7) ~ 1.
        let mut dz = -10.0;
        while dz <= 16.0 {
            let v = model2_sprdngf(dz, 0.0);
            assert!(
                (0.0..=1.0 + 1e-6).contains(&v),
                "sprdngf out of range at dz = {dz}: {v}"
            );
            dz += 0.01;
        }
    }

    #[test]
    fn model2_sprdngf_cutoff_engages_on_tmpy_alone() {
        // tmpy crosses -100 dB at tmpx ~= -5.0305 (j - i ~= -4.791)
        // and tmpx ~= +11.0312 (j - i ~= +10.506). Just inside the
        // window the value is positive; beyond it, exactly 0.
        assert!(model2_sprdngf(-4.7, 0.0) > 0.0);
        assert_eq!(model2_sprdngf(-4.9, 0.0), 0.0);
        assert!(model2_sprdngf(10.4, 0.0) > 0.0);
        assert_eq!(model2_sprdngf(10.6, 0.0), 0.0);
        // Spot anchors bracketing the same crossings in tmpx space:
        assert!(model2_tmpy(-4.5) > -100.0);
        assert!(model2_tmpy(-5.2) < -100.0);
        assert!(model2_tmpy(11.0) > -100.0);
        assert!(model2_tmpy(12.0) < -100.0);
    }

    #[test]
    fn model2_sprdngf_reduces_to_one_arg_form_outside_active_window() {
        // Outside the model2_x_is_active window x == 0 exactly, so
        // the printed 10^((x + tmpy)/10) collapses to the staged
        // one-argument sprdngf_from_tmpy(tmpy).
        let mut dz = -10.0;
        while dz <= 16.0 {
            if !model2_x_is_active(dz, 0.0) {
                let tmpy = model2_tmpy(model2_tmpx(dz, 0.0));
                let a = model2_sprdngf(dz, 0.0);
                let b = sprdngf_from_tmpy(tmpy);
                assert!(
                    (a - b).abs() <= 1e-15 * b.abs().max(1.0),
                    "x = 0 reduction mismatch at dz = {dz}: {a} vs {b}"
                );
            }
            dz += 0.07;
        }
    }

    #[test]
    fn model2_sprdngf_x_term_sharpens_inside_active_window() {
        // Inside (0.5, 2.5) the x term is strictly negative, so the
        // full form must sit strictly below the x = 0 reduction.
        for &tmpx in &[0.7f64, 1.0, 1.5, 2.0, 2.3] {
            let dz = tmpx / 1.05;
            let full = model2_sprdngf(dz, 0.0);
            let no_x = sprdngf_from_tmpy(model2_tmpy(tmpx));
            assert!(
                full < no_x,
                "x term did not lower sprdngf at tmpx = {tmpx}: {full} vs {no_x}"
            );
        }
    }

    #[test]
    fn model2_sprdngf_decays_away_from_the_masker() {
        // Downward (j < i, the steep 26.25 dB/Bark side) the weight
        // decays strictly monotonically.
        let mut prev = model2_sprdngf(-0.25, 0.0);
        let mut dz = -0.5;
        while dz >= -4.5 {
            let v = model2_sprdngf(dz, 0.0);
            assert!(v < prev, "not decaying downward at dz = {dz}");
            prev = v;
            dz -= 0.25;
        }
        // Upward the printed form is *piecewise* monotone: the x
        // term (active for tmpx in (0.5, 2.5)) overshoots the tmpy
        // backbone, producing a local dip at tmpx ~= 2.049
        // (dz ~= 1.951) and a local crest where x returns to 0 at
        // tmpx = 2.5 (dz ~= 2.381), after which the backbone decay
        // resumes. Assert strict decay on both monotone stretches.
        let mut prev = model2_sprdngf(0.25, 0.0);
        let mut dz = 0.5;
        while dz <= 1.95 {
            let v = model2_sprdngf(dz, 0.0);
            assert!(v < prev, "not decaying on the pre-dip stretch at dz = {dz}");
            prev = v;
            dz += 0.25;
        }
        let mut prev = model2_sprdngf(2.5, 0.0);
        let mut dz = 2.75;
        while dz <= 10.0 {
            let v = model2_sprdngf(dz, 0.0);
            assert!(v < prev, "not decaying past the x window at dz = {dz}");
            prev = v;
            dz += 0.25;
        }
        // ... and the dip/crest pair is real: the window-exit crest
        // sits strictly above the in-window dip.
        let dip = model2_sprdngf(2.049 / 1.05, 0.0);
        let crest = model2_sprdngf(2.5 / 1.05, 0.0);
        assert!(crest > dip, "expected local crest {crest} > dip {dip}");
    }

    // -----------------------------------------------------------
    // Table D.1a — threshold-in-quiet, complete 108-row table
    // (Layer I, 32 kHz)
    // -----------------------------------------------------------

    #[test]
    fn d1a_full_table_len_and_index_numbering() {
        assert_eq!(LTQ_L1_32K.len(), LTQ_LAYER1_FULL_LEN);
        assert_eq!(LTQ_LAYER1_FULL_LEN, 108);
        assert_eq!(LTQ_LAYER2_FULL_LEN, 132);
        // Rows are stored densely in 1-based print order: row at
        // slot k carries index k + 1.
        for (k, r) in LTQ_L1_32K.iter().enumerate() {
            assert_eq!(r.index as usize, k + 1, "slot {k}");
        }
    }

    #[test]
    fn d1a_head_rows_verbatim() {
        // Rows i = 1..=5, all four columns verbatim from the docs
        // extract (cross-checked against the PNG render there).
        let expect = [
            (1u16, 62.50, 0.617, 33.44),
            (2, 125.00, 1.232, 19.20),
            (3, 187.50, 1.842, 13.87),
            (4, 250.00, 2.445, 11.01),
            (5, 312.50, 3.037, 9.20),
        ];
        for &(i, f, z, ltq) in &expect {
            let r = ltq_layer1_32k(i).unwrap();
            assert_eq!(r.index, i);
            assert!((r.freq_hz - f).abs() < 1e-9, "freq at i = {i}");
            assert!((r.bark_z - z).abs() < 1e-9, "bark at i = {i}");
            assert!((r.ltq_db - ltq).abs() < 1e-9, "LTq at i = {i}");
        }
    }

    #[test]
    fn d1a_final_row_verbatim() {
        let r = ltq_layer1_32k(108).unwrap();
        assert_eq!(r.index, 108);
        assert!((r.freq_hz - 15_000.00).abs() < 1e-9);
        assert!((r.bark_z - 23.923).abs() < 1e-9);
        assert!((r.ltq_db - 51.04).abs() < 1e-9);
    }

    #[test]
    fn d1a_freq_and_bark_strictly_monotonic() {
        // Frequency and Bark rise monotonically across the whole
        // table; LTq does not (see d1a_ltq_minimum_near_i51).
        for w in LTQ_L1_32K.windows(2) {
            assert!(w[1].index == w[0].index + 1);
            assert!(w[1].freq_hz > w[0].freq_hz, "freq at i = {}", w[1].index);
            assert!(w[1].bark_z > w[0].bark_z, "bark at i = {}", w[1].index);
        }
    }

    #[test]
    fn d1a_head_line_spacing_62_5_hz() {
        // The head rows sit on the raw FFT-line grid: 62,5 Hz per
        // line at Fs = 32 kHz. The table decimates to a 2-line grid
        // beyond i = 48 (125 Hz spacing) and a 4-line grid beyond
        // i = 72 (250 Hz), so only the head sits on the 62,5 Hz grid.
        for r in &LTQ_L1_32K[..48] {
            assert!(
                (r.freq_hz - 62.5 * f64::from(r.index)).abs() < 1e-9,
                "head row i = {} off the 62,5 Hz grid",
                r.index
            );
        }
    }

    #[test]
    fn d1a_ltq_minimum_near_i51() {
        // Per the docs-extract prose the LTq column is non-monotonic
        // with a minimum near i = 51 (≈ 3,375 kHz) at -4.97 dB,
        // rising steeply at both ends. Verify the global minimum sits
        // exactly at i = 51 and the endpoints exceed it.
        let min = LTQ_L1_32K
            .iter()
            .min_by(|a, b| a.ltq_db.partial_cmp(&b.ltq_db).unwrap())
            .unwrap();
        assert_eq!(min.index, 51);
        assert!((min.ltq_db - (-4.97)).abs() < 1e-9);
        assert!(ltq_layer1_32k(1).unwrap().ltq_db > min.ltq_db);
        assert!(ltq_layer1_32k(108).unwrap().ltq_db > min.ltq_db);
        // The descent to i = 51 is strictly monotonic; the ascent
        // from i = 51 to i = 108 is strictly monotonic.
        for w in LTQ_L1_32K[..51].windows(2) {
            assert!(w[1].ltq_db < w[0].ltq_db, "descent at i = {}", w[1].index);
        }
        for w in LTQ_L1_32K[50..].windows(2) {
            assert!(w[1].ltq_db > w[0].ltq_db, "ascent at i = {}", w[1].index);
        }
    }

    #[test]
    fn d1a_cross_checks_against_d2a_band_boundaries() {
        // Table D.2a's `index F&CB` column points into Table D.1a;
        // every D.2a boundary index must carry the same frequency and
        // Bark values in the now-complete D.1a table.
        let bands = critical_band_table(Layer::I, 32_000).unwrap();
        let mut checked = 0;
        for b in bands {
            let r = ltq_layer1_32k(b.index_fcb).expect("D.2a index into full D.1a");
            assert!(
                (r.freq_hz - b.top_freq_hz).abs() < 1e-9,
                "freq mismatch at index {}",
                b.index_fcb
            );
            assert!(
                (r.bark_z - b.bark_z).abs() < 1e-9,
                "bark mismatch at index {}",
                b.index_fcb
            );
            checked += 1;
        }
        // All 24 D.2a band boundaries cross-check against D.1a.
        assert_eq!(checked, bands.len());
    }

    #[test]
    fn d1a_lookup_some_for_every_printed_index() {
        for i in 1u16..=108 {
            let r = ltq_layer1_32k(i).expect("printed index");
            assert_eq!(r.index, i);
        }
    }

    #[test]
    fn d1a_lookup_none_out_of_range() {
        // 1-based underflow and indices above the printed table miss.
        for &i in &[0u16, 109, 132, 200, u16::MAX] {
            assert!(ltq_layer1_32k(i).is_none(), "i = {i}");
        }
    }

    #[test]
    fn d1a_used_composes_step3_offset() {
        // >= 96 kbit/s per channel: −12 dB; below: pass-through.
        let r1 = ltq_layer1_32k(1).unwrap();
        assert!((ltq_layer1_32k_used(1, 96).unwrap() - (r1.ltq_db - 12.0)).abs() < 1e-12);
        assert!((ltq_layer1_32k_used(1, 95).unwrap() - r1.ltq_db).abs() < 1e-12);
        // Agreement with the generic Step 3 adapter at several rates.
        for &kbps in &[32u32, 64, 96, 128, 192, 448] {
            for &i in &[2u16, 5, 108] {
                let row = ltq_layer1_32k(i).unwrap();
                assert_eq!(
                    ltq_layer1_32k_used(i, kbps).unwrap(),
                    step3_apply_ltq_offset(row.ltq_db, kbps),
                    "i = {i}, kbps = {kbps}"
                );
            }
        }
        // None propagates through the composition for out-of-range
        // indices.
        assert!(ltq_layer1_32k_used(109, 192).is_none());
        assert!(ltq_layer1_32k_used(0, 192).is_none());
    }

    #[test]
    fn d1a_anchor_lines_map_into_d2a_bands() {
        // Step 4 consistency: every anchored line lands in a D.2a
        // critical band, at the expected band number (the D.2a
        // boundary column is the band's *top* line: band 0 tops at
        // index 1, band 1 at 3, band 2 at 5, band 23 at 108).
        let expect = [(1u16, 0usize), (2, 1), (3, 1), (4, 2), (5, 2), (108, 23)];
        let bands = critical_band_table(Layer::I, 32_000).unwrap();
        for &(i, band) in &expect {
            assert_eq!(
                critical_band_for_line(Layer::I, 32_000, i),
                Some(band),
                "line {i}"
            );
            // The anchored row's Bark value never exceeds its
            // band's top-edge Bark.
            let r = ltq_layer1_32k(i).unwrap();
            assert!(r.bark_z <= bands[band].bark_z + 1e-9, "line {i}");
        }
    }

    // --- Table D.1b (Layer I, Fs = 44,1 kHz) ---

    #[test]
    fn d1b_full_table_len_and_index_numbering() {
        assert_eq!(LTQ_L1_44K1.len(), LTQ_LAYER1_44K1_LEN);
        assert_eq!(LTQ_LAYER1_44K1_LEN, 106);
        // Rows are stored densely in 1-based print order: row at
        // slot k carries index k + 1.
        for (k, r) in LTQ_L1_44K1.iter().enumerate() {
            assert_eq!(r.index as usize, k + 1, "slot {k}");
        }
    }

    #[test]
    fn d1b_head_rows_verbatim() {
        // Rows i = 1..=5, all four columns verbatim from the docs
        // extract annex-d-table-D1b-threshold-44k1Hz.csv.
        let expect = [
            (1u16, 86.13, 0.850, 25.87),
            (2, 172.27, 1.694, 14.85),
            (3, 258.40, 2.525, 10.72),
            (4, 344.53, 3.337, 8.50),
            (5, 430.66, 4.124, 7.10),
        ];
        for &(i, f, z, ltq) in &expect {
            let r = ltq_layer1_44k1(i).unwrap();
            assert_eq!(r.index, i);
            assert!((r.freq_hz - f).abs() < 1e-9, "freq at i = {i}");
            assert!((r.bark_z - z).abs() < 1e-9, "bark at i = {i}");
            assert!((r.ltq_db - ltq).abs() < 1e-9, "LTq at i = {i}");
        }
    }

    #[test]
    fn d1b_final_row_verbatim() {
        let r = ltq_layer1_44k1(106).unwrap();
        assert_eq!(r.index, 106);
        assert!((r.freq_hz - 19_982.81).abs() < 1e-9);
        assert!((r.bark_z - 24.574).abs() < 1e-9);
        assert!((r.ltq_db - 68.00).abs() < 1e-9);
    }

    #[test]
    fn d1b_freq_and_bark_strictly_monotonic() {
        // Frequency and Bark rise monotonically across the whole
        // table; LTq does not (see d1b_ltq_minimum_near_i39).
        for w in LTQ_L1_44K1.windows(2) {
            assert!(w[1].index == w[0].index + 1);
            assert!(w[1].freq_hz > w[0].freq_hz, "freq at i = {}", w[1].index);
            assert!(w[1].bark_z > w[0].bark_z, "bark at i = {}", w[1].index);
        }
    }

    #[test]
    fn d1b_head_line_spacing_86_13_hz() {
        // The head rows sit on the raw FFT-line grid: ≈86,13 Hz per
        // line at Fs = 44,1 kHz (44100/512). The table decimates to a
        // 2-line grid beyond i = 48, so only the head sits on the
        // single-line grid.
        let line_hz = 44_100.0 / 512.0;
        for r in &LTQ_L1_44K1[..48] {
            // The CSV rounds frequencies to two decimals, so the
            // residual against the exact line grid can reach 5e-3.
            assert!(
                (r.freq_hz - line_hz * f64::from(r.index)).abs() < 6e-3,
                "head row i = {} off the 86,13 Hz grid (got {})",
                r.index,
                r.freq_hz
            );
        }
    }

    #[test]
    fn d1b_ltq_minimum_near_i39() {
        // Per the docs extract the LTq column is non-monotonic with a
        // minimum at i = 39 (≈ 3,36 kHz) at -4.98 dB, rising steeply
        // at both ends and saturating at the 68 dB ceiling near the
        // top. Verify the global minimum sits exactly at i = 39 and
        // the endpoints exceed it.
        let min = LTQ_L1_44K1
            .iter()
            .min_by(|a, b| a.ltq_db.partial_cmp(&b.ltq_db).unwrap())
            .unwrap();
        assert_eq!(min.index, 39);
        assert!((min.ltq_db - (-4.98)).abs() < 1e-9);
        assert!(ltq_layer1_44k1(1).unwrap().ltq_db > min.ltq_db);
        assert!(ltq_layer1_44k1(106).unwrap().ltq_db > min.ltq_db);
        // The descent to i = 39 is strictly monotonic.
        for w in LTQ_L1_44K1[..39].windows(2) {
            assert!(w[1].ltq_db < w[0].ltq_db, "descent at i = {}", w[1].index);
        }
        // The ascent from i = 39 is monotonically non-decreasing (it
        // flattens onto the 68,00 dB ceiling from i = 95 onward).
        for w in LTQ_L1_44K1[38..].windows(2) {
            assert!(w[1].ltq_db >= w[0].ltq_db, "ascent at i = {}", w[1].index);
        }
    }

    #[test]
    fn d1b_ceiling_saturates_at_68_db_from_i95() {
        // The printed column clamps at the 68,00 dB ceiling for the
        // top dozen rows (i = 95..=106).
        for i in 95u16..=106 {
            assert!(
                (ltq_layer1_44k1(i).unwrap().ltq_db - 68.00).abs() < 1e-9,
                "i = {i} not at ceiling"
            );
        }
        // i = 94 is the last sub-ceiling row.
        assert!(ltq_layer1_44k1(94).unwrap().ltq_db < 68.00);
    }

    #[test]
    fn d1b_cross_checks_against_d2b_band_boundaries() {
        // Table D.2b's `index F&CB` column points into Table D.1b;
        // every D.2b boundary index must carry the same Bark value in
        // the now-complete D.1b table, and a frequency agreeing to the
        // table's two-decimal rounding.
        let bands = critical_band_table(Layer::I, 44_100).unwrap();
        let mut checked = 0;
        for b in bands {
            let r = ltq_layer1_44k1(b.index_fcb).expect("D.2b index into full D.1b");
            // D.1b prints frequencies to 2 decimals; D.2b carries 3,
            // so the rounding residual can reach 5e-3.
            assert!(
                (r.freq_hz - b.top_freq_hz).abs() < 6e-3,
                "freq mismatch at index {} ({} vs {})",
                b.index_fcb,
                r.freq_hz,
                b.top_freq_hz
            );
            // Both tables print the Bark rate to three decimals; the
            // D.1 and D.2 extractions round the last digit
            // independently (e.g. index 50: 17,905 in D.1b vs 17,904
            // in D.2b), so allow a 1e-3 last-digit divergence.
            assert!(
                (r.bark_z - b.bark_z).abs() < 2e-3,
                "bark mismatch at index {} ({} vs {})",
                b.index_fcb,
                r.bark_z,
                b.bark_z
            );
            checked += 1;
        }
        assert_eq!(checked, bands.len());
    }

    #[test]
    fn d1b_lookup_some_for_every_printed_index() {
        for i in 1u16..=106 {
            let r = ltq_layer1_44k1(i).expect("printed index");
            assert_eq!(r.index, i);
        }
    }

    #[test]
    fn d1b_lookup_none_out_of_range() {
        // 1-based underflow and indices above the printed table miss.
        for &i in &[0u16, 107, 108, 132, 200, u16::MAX] {
            assert!(ltq_layer1_44k1(i).is_none(), "i = {i}");
        }
    }

    #[test]
    fn d1b_used_composes_step3_offset() {
        // >= 96 kbit/s per channel: −12 dB; below: pass-through.
        let r1 = ltq_layer1_44k1(1).unwrap();
        assert!((ltq_layer1_44k1_used(1, 96).unwrap() - (r1.ltq_db - 12.0)).abs() < 1e-12);
        assert!((ltq_layer1_44k1_used(1, 95).unwrap() - r1.ltq_db).abs() < 1e-12);
        // Agreement with the generic Step 3 adapter at several rates.
        for &kbps in &[32u32, 64, 96, 128, 192, 448] {
            for &i in &[2u16, 39, 106] {
                let row = ltq_layer1_44k1(i).unwrap();
                assert_eq!(
                    ltq_layer1_44k1_used(i, kbps).unwrap(),
                    step3_apply_ltq_offset(row.ltq_db, kbps),
                    "i = {i}, kbps = {kbps}"
                );
            }
        }
        // None propagates through the composition for out-of-range
        // indices.
        assert!(ltq_layer1_44k1_used(107, 192).is_none());
        assert!(ltq_layer1_44k1_used(0, 192).is_none());
    }

    #[test]
    fn d1b_distinct_from_d1a_grid() {
        // D.1b (44,1 kHz) and D.1a (32 kHz) sit on different FFT-line
        // grids: at a shared index the 44,1 kHz frequency is higher
        // (86,13 Hz/line vs 62,5 Hz/line), and the table is two rows
        // shorter (106 vs 108).
        assert!(LTQ_L1_44K1.len() < LTQ_L1_32K.len());
        for i in 1u16..=106 {
            let a = ltq_layer1_32k(i).unwrap();
            let b = ltq_layer1_44k1(i).unwrap();
            assert!(
                b.freq_hz > a.freq_hz,
                "i = {i}: 44,1 kHz freq {} not above 32 kHz freq {}",
                b.freq_hz,
                a.freq_hz
            );
        }
    }

    // --- Table D.1c (Layer I, Fs = 48 kHz) ---

    #[test]
    fn d1c_full_table_len_and_index_numbering() {
        assert_eq!(LTQ_L1_48K.len(), LTQ_LAYER1_48K_LEN);
        assert_eq!(LTQ_LAYER1_48K_LEN, 102);
        for (k, r) in LTQ_L1_48K.iter().enumerate() {
            assert_eq!(r.index as usize, k + 1, "slot {k}");
        }
    }

    #[test]
    fn d1c_head_rows_verbatim() {
        // Rows i = 1..=5, all four columns verbatim from the docs
        // extract annex-d-table-D1c-threshold-48kHz.csv.
        let expect = [
            (1u16, 93.75, 0.925, 24.17),
            (2, 187.50, 1.842, 13.87),
            (3, 281.25, 2.742, 10.01),
            (4, 375.00, 3.618, 7.94),
            (5, 468.75, 4.463, 6.62),
        ];
        for &(i, f, z, ltq) in &expect {
            let r = ltq_layer1_48k(i).unwrap();
            assert_eq!(r.index, i);
            assert!((r.freq_hz - f).abs() < 1e-9, "freq at i = {i}");
            assert!((r.bark_z - z).abs() < 1e-9, "bark at i = {i}");
            assert!((r.ltq_db - ltq).abs() < 1e-9, "LTq at i = {i}");
        }
    }

    #[test]
    fn d1c_final_row_verbatim() {
        let r = ltq_layer1_48k(102).unwrap();
        assert_eq!(r.index, 102);
        assert!((r.freq_hz - 20_250.00).abs() < 1e-9);
        assert!((r.bark_z - 24.597).abs() < 1e-9);
        assert!((r.ltq_db - 68.00).abs() < 1e-9);
    }

    #[test]
    fn d1c_freq_and_bark_strictly_monotonic() {
        for w in LTQ_L1_48K.windows(2) {
            assert!(w[1].index == w[0].index + 1);
            assert!(w[1].freq_hz > w[0].freq_hz, "freq at i = {}", w[1].index);
            assert!(w[1].bark_z > w[0].bark_z, "bark at i = {}", w[1].index);
        }
    }

    #[test]
    fn d1c_head_line_spacing_93_75_hz() {
        // The head rows sit on the raw FFT-line grid: 93,75 Hz per line
        // at Fs = 48 kHz (48000/512). The table decimates to a 2-line
        // grid beyond the head, so only the head sits on that grid.
        let line_hz = 48_000.0 / 512.0;
        for r in &LTQ_L1_48K[..44] {
            assert!(
                (r.freq_hz - line_hz * f64::from(r.index)).abs() < 6e-3,
                "head row i = {} off the 93,75 Hz grid (got {})",
                r.index,
                r.freq_hz
            );
        }
    }

    #[test]
    fn d1c_ltq_minimum_near_i35() {
        // Non-monotonic LTq column with a global minimum at i = 35
        // (≈ 3,28 kHz) at -4.98 dB; strictly descending to it and
        // non-decreasing after, saturating at the 68 dB ceiling.
        let min = LTQ_L1_48K
            .iter()
            .min_by(|a, b| a.ltq_db.partial_cmp(&b.ltq_db).unwrap())
            .unwrap();
        assert_eq!(min.index, 35);
        assert!((min.ltq_db - (-4.98)).abs() < 1e-9);
        assert!(ltq_layer1_48k(1).unwrap().ltq_db > min.ltq_db);
        assert!(ltq_layer1_48k(102).unwrap().ltq_db > min.ltq_db);
        for w in LTQ_L1_48K[..35].windows(2) {
            assert!(w[1].ltq_db < w[0].ltq_db, "descent at i = {}", w[1].index);
        }
        for w in LTQ_L1_48K[34..].windows(2) {
            assert!(w[1].ltq_db >= w[0].ltq_db, "ascent at i = {}", w[1].index);
        }
    }

    #[test]
    fn d1c_ceiling_saturates_at_68_db_from_i91() {
        for i in 91u16..=102 {
            assert!(
                (ltq_layer1_48k(i).unwrap().ltq_db - 68.00).abs() < 1e-9,
                "i = {i} not at ceiling"
            );
        }
        assert!(ltq_layer1_48k(90).unwrap().ltq_db < 68.00);
    }

    #[test]
    fn d1c_cross_checks_against_d2c_band_boundaries() {
        // Table D.2c's `index F&CB` column points into Table D.1c;
        // every D.2c boundary index must carry the same Bark value and
        // a frequency agreeing to the table's two-decimal rounding.
        let bands = critical_band_table(Layer::I, 48_000).unwrap();
        let mut checked = 0;
        for b in bands {
            let r = ltq_layer1_48k(b.index_fcb).expect("D.2c index into full D.1c");
            assert!(
                (r.freq_hz - b.top_freq_hz).abs() < 6e-3,
                "freq mismatch at index {} ({} vs {})",
                b.index_fcb,
                r.freq_hz,
                b.top_freq_hz
            );
            assert!(
                (r.bark_z - b.bark_z).abs() < 2e-3,
                "bark mismatch at index {} ({} vs {})",
                b.index_fcb,
                r.bark_z,
                b.bark_z
            );
            checked += 1;
        }
        assert_eq!(checked, bands.len());
    }

    #[test]
    fn d1c_lookup_some_for_every_printed_index() {
        for i in 1u16..=102 {
            let r = ltq_layer1_48k(i).expect("printed index");
            assert_eq!(r.index, i);
        }
    }

    #[test]
    fn d1c_lookup_none_out_of_range() {
        for &i in &[0u16, 103, 106, 108, 132, 200, u16::MAX] {
            assert!(ltq_layer1_48k(i).is_none(), "i = {i}");
        }
    }

    #[test]
    fn d1c_used_composes_step3_offset() {
        let r1 = ltq_layer1_48k(1).unwrap();
        assert!((ltq_layer1_48k_used(1, 96).unwrap() - (r1.ltq_db - 12.0)).abs() < 1e-12);
        assert!((ltq_layer1_48k_used(1, 95).unwrap() - r1.ltq_db).abs() < 1e-12);
        for &kbps in &[32u32, 64, 96, 128, 192, 448] {
            for &i in &[2u16, 35, 102] {
                let row = ltq_layer1_48k(i).unwrap();
                assert_eq!(
                    ltq_layer1_48k_used(i, kbps).unwrap(),
                    step3_apply_ltq_offset(row.ltq_db, kbps),
                    "i = {i}, kbps = {kbps}"
                );
            }
        }
        assert!(ltq_layer1_48k_used(103, 192).is_none());
        assert!(ltq_layer1_48k_used(0, 192).is_none());
    }

    #[test]
    fn d1c_distinct_from_d1a_and_d1b_grid() {
        // D.1c (48 kHz) sits on the coarsest FFT-line grid: at a shared
        // index the 48 kHz frequency exceeds both lower rates, and the
        // table is the shortest of the three (102 < 106 < 108).
        assert!(LTQ_L1_48K.len() < LTQ_L1_44K1.len());
        assert!(LTQ_L1_44K1.len() < LTQ_L1_32K.len());
        for i in 1u16..=102 {
            let a = ltq_layer1_32k(i).unwrap();
            let b = ltq_layer1_44k1(i).unwrap();
            let c = ltq_layer1_48k(i).unwrap();
            assert!(
                c.freq_hz > b.freq_hz && b.freq_hz > a.freq_hz,
                "i = {i}: 48k {} not above 44.1k {} not above 32k {}",
                c.freq_hz,
                b.freq_hz,
                a.freq_hz
            );
        }
    }

    // --- Model 2 partition-domain spreading operator (32 kHz) ---

    #[test]
    fn model2_partition_count_32k_matches_table() {
        // bmax = 49 at 32 kHz, equal to the complete D.3a length.
        assert_eq!(MODEL2_PARTITIONS_32K, 49);
        assert_eq!(MODEL2_PARTITIONS_32K, CALC_PARTITION_32K.len());
    }

    #[test]
    fn model2_spread_weight_32k_diagonal_is_one() {
        // from == into ⇒ bval[d] == bval[s] ⇒ sprdngf(z, z) ≈ 1
        // (the spreading backbone peaks at 0 dB on the diagonal).
        for n in 1..=MODEL2_PARTITIONS_32K as u16 {
            let w = model2_spread_weight_32k(n, n).unwrap();
            assert!(
                (w - 1.0).abs() < 1e-6,
                "diagonal weight at partition {n} = {w}, expected ≈ 1"
            );
        }
    }

    #[test]
    fn model2_spread_weight_32k_matches_sprdngf_composition() {
        // The per-pair weight is exactly model2_sprdngf(bval[into],
        // bval[from]) — the destination Bark is the spread-into (j)
        // argument and the source Bark is the masker (i) argument.
        for &(into, from) in &[(1u16, 1u16), (10, 8), (8, 10), (49, 1), (1, 49), (25, 30)] {
            let got = model2_spread_weight_32k(into, from).unwrap();
            let bval_into = calc_partition_32k(into).unwrap().bval;
            let bval_from = calc_partition_32k(from).unwrap().bval;
            let want = model2_sprdngf(bval_into, bval_from);
            assert!(
                (got - want).abs() < 1e-15,
                "weight({into},{from}) = {got}, expected {want}"
            );
        }
    }

    #[test]
    fn model2_spread_weight_32k_out_of_range_is_none() {
        assert!(model2_spread_weight_32k(0, 1).is_none());
        assert!(model2_spread_weight_32k(1, 0).is_none());
        assert!(model2_spread_weight_32k(50, 1).is_none());
        assert!(model2_spread_weight_32k(1, 50).is_none());
        // In-range endpoints are present.
        assert!(model2_spread_weight_32k(1, 1).is_some());
        assert!(model2_spread_weight_32k(49, 49).is_some());
    }

    #[test]
    fn model2_spreading_matrix_32k_shape_and_diagonal() {
        let m = model2_spreading_matrix_32k();
        assert_eq!(m.len(), MODEL2_PARTITIONS_32K);
        for row in &m {
            assert_eq!(row.len(), MODEL2_PARTITIONS_32K);
        }
        for (d, row) in m.iter().enumerate() {
            assert!(
                (row[d] - 1.0).abs() < 1e-6,
                "matrix[{d}][{d}] = {} expected ≈ 1",
                row[d]
            );
        }
    }

    #[test]
    fn model2_spreading_matrix_32k_agrees_with_per_pair_weight() {
        let m = model2_spreading_matrix_32k();
        for (d, row) in m.iter().enumerate() {
            for (s, &cell) in row.iter().enumerate() {
                // 0-based matrix index == 1-based partition number − 1.
                let want = model2_spread_weight_32k(d as u16 + 1, s as u16 + 1).unwrap();
                assert!(
                    (cell - want).abs() < 1e-15,
                    "matrix[{d}][{s}] = {cell}, per-pair = {want}"
                );
            }
        }
    }

    #[test]
    fn model2_spreading_matrix_32k_entries_never_amplify() {
        // Every entry is a non-negative power weight that never
        // exceeds the on-diagonal value (the backbone maxes at 0 dB).
        let m = model2_spreading_matrix_32k();
        for row in &m {
            for &cell in row {
                assert!(cell >= 0.0, "negative weight {cell}");
                assert!(
                    cell <= 1.0 + 1e-6,
                    "weight {cell} exceeds the diagonal peak"
                );
            }
        }
    }

    #[test]
    fn model2_spreading_matrix_32k_is_asymmetric() {
        // The Bark-domain spreading is steeper below the masker than
        // above, so spreading down (toward a lower destination
        // partition) differs from spreading up. Pick a near pair
        // inside the cutoff window where both directions are non-zero.
        let m = model2_spreading_matrix_32k();
        // Partitions 10 and 12 (bval 7.28 and 8.50, Δ ≈ 1.22 Bark).
        let up = m[11][9]; // source 10 → destination 12 (j > i)
        let down = m[9][11]; // source 12 → destination 10 (j < i)
        assert!(up > 0.0 && down > 0.0, "both directions should survive");
        assert!(
            (up - down).abs() > 1e-3,
            "spreading should be asymmetric: up {up} vs down {down}"
        );
    }

    #[test]
    fn model2_spreading_matrix_32k_decays_off_diagonal() {
        // Walking away from the masker source partition the weight
        // is monotonically non-increasing on the shallow (upward)
        // side until the cutoff zeroes it.
        let m = model2_spreading_matrix_32k();
        let src = 5usize; // 0-based source partition
        let mut prev = m[src][src]; // diagonal == 1
        for (d, row) in m.iter().enumerate().skip(src + 1) {
            let cur = row[src];
            assert!(
                cur <= prev + 1e-12,
                "weight rose moving away: m[{d}][{src}] = {cur} > prev {prev}"
            );
            prev = cur;
        }
    }

    #[test]
    fn model2_spread_normalization_32k_is_finite_positive() {
        let r = model2_spread_normalization_32k();
        assert_eq!(r.len(), MODEL2_PARTITIONS_32K);
        for (s, &factor) in r.iter().enumerate() {
            assert!(
                factor.is_finite() && factor > 0.0,
                "rnorm[{s}] = {factor} not finite-positive"
            );
            // The diagonal weight alone is ≈ 1, so each column sum is
            // ≥ 1, hence each normalisation factor is ≤ 1.
            assert!(factor <= 1.0 + 1e-9, "rnorm[{s}] = {factor} > 1");
        }
    }

    #[test]
    fn model2_spread_normalization_32k_conserves_energy() {
        // Multiplying a unit source impulse by rnorm[s] and spreading
        // it over every destination must sum back to exactly 1 (the
        // clause D.2.3 energy-conservation property of rnorm).
        let m = model2_spreading_matrix_32k();
        let rnorm = model2_spread_normalization_32k();
        for s in 0..MODEL2_PARTITIONS_32K {
            let spread_total: f64 = (0..MODEL2_PARTITIONS_32K).map(|d| m[d][s] * rnorm[s]).sum();
            assert!(
                (spread_total - 1.0).abs() < 1e-12,
                "rnorm-scaled spread of source {s} sums to {spread_total}, expected 1"
            );
        }
    }

    #[test]
    fn model2_spread_energy_32k_rejects_wrong_length() {
        assert!(model2_spread_energy_32k(&[]).is_none());
        assert!(model2_spread_energy_32k(&[1.0; MODEL2_PARTITIONS_32K - 1]).is_none());
        assert!(model2_spread_energy_32k(&[1.0; MODEL2_PARTITIONS_32K + 1]).is_none());
        // Exact length succeeds.
        assert!(model2_spread_energy_32k(&[0.0; MODEL2_PARTITIONS_32K]).is_some());
    }

    #[test]
    fn model2_spread_energy_32k_matches_explicit_matrix_product() {
        // The function must equal the explicit Σ_s (e[s]·rnorm[s])·m[d][s].
        let m = model2_spreading_matrix_32k();
        let rnorm = model2_spread_normalization_32k();
        let n = MODEL2_PARTITIONS_32K;
        // A non-trivial source vector (a ramp, so every source differs).
        let energy: Vec<f64> = (0..n).map(|s| (s as f64) + 1.0).collect();
        let eb = model2_spread_energy_32k(&energy).unwrap();
        assert_eq!(eb.len(), n);
        for d in 0..n {
            let want: f64 = (0..n).map(|s| energy[s] * rnorm[s] * m[d][s]).sum();
            assert!(
                (eb[d] - want).abs() <= 1e-12 * want.abs().max(1.0),
                "eb[{d}] = {} != explicit product {want}",
                eb[d]
            );
        }
    }

    #[test]
    fn model2_spread_energy_32k_conserves_total_energy() {
        // The energy-conserving operator redistributes but neither
        // creates nor destroys energy: Σ eb == Σ source.
        let n = MODEL2_PARTITIONS_32K;
        let energy: Vec<f64> = (0..n).map(|s| 0.5 + 0.25 * (s as f64)).collect();
        let total_in: f64 = energy.iter().sum();
        let eb = model2_spread_energy_32k(&energy).unwrap();
        let total_out: f64 = eb.iter().sum();
        assert!(
            (total_out - total_in).abs() <= 1e-9 * total_in,
            "spread total {total_out} != source total {total_in}"
        );
    }

    #[test]
    fn model2_spread_energy_32k_unit_impulse_recovers_normalized_column() {
        // A unit impulse at source s spreads, per destination, to
        // rnorm[s]·m[d][s] — and over all destinations sums to 1.
        let m = model2_spreading_matrix_32k();
        let rnorm = model2_spread_normalization_32k();
        let n = MODEL2_PARTITIONS_32K;
        for s in [0usize, 5, 24, n - 1] {
            let mut energy = vec![0.0f64; n];
            energy[s] = 1.0;
            let eb = model2_spread_energy_32k(&energy).unwrap();
            let sum: f64 = eb.iter().sum();
            assert!(
                (sum - 1.0).abs() < 1e-12,
                "unit impulse at {s} spreads to total {sum}, expected 1"
            );
            for (d, &got) in eb.iter().enumerate() {
                let want = rnorm[s] * m[d][s];
                assert!(
                    (got - want).abs() <= 1e-12,
                    "impulse {s}: eb[{d}] = {got} != rnorm·m {want}"
                );
            }
        }
    }

    #[test]
    fn model2_spread_energy_32k_zero_source_is_all_zero() {
        let eb = model2_spread_energy_32k(&[0.0; MODEL2_PARTITIONS_32K]).unwrap();
        assert!(eb.iter().all(|&e| e == 0.0), "zero source spread non-zero");
    }

    #[test]
    fn model2_spread_energy_32k_is_linear() {
        // Superposition: spread(a + b) == spread(a) + spread(b), and
        // spread(k·a) == k·spread(a). A linear operator must satisfy both.
        let n = MODEL2_PARTITIONS_32K;
        let a: Vec<f64> = (0..n).map(|s| (s as f64).sin().abs() + 0.1).collect();
        let b: Vec<f64> = (0..n).map(|s| (s as f64 * 0.3).cos().abs() + 0.2).collect();
        let sum: Vec<f64> = (0..n).map(|s| a[s] + b[s]).collect();
        let ea = model2_spread_energy_32k(&a).unwrap();
        let eb = model2_spread_energy_32k(&b).unwrap();
        let esum = model2_spread_energy_32k(&sum).unwrap();
        for d in 0..n {
            assert!(
                (esum[d] - (ea[d] + eb[d])).abs() <= 1e-12 * (ea[d] + eb[d]).abs().max(1.0),
                "additivity failed at {d}"
            );
        }
        let scaled: Vec<f64> = a.iter().map(|&x| 3.5 * x).collect();
        let escaled = model2_spread_energy_32k(&scaled).unwrap();
        for d in 0..n {
            assert!(
                (escaled[d] - 3.5 * ea[d]).abs() <= 1e-12 * (3.5 * ea[d]).abs().max(1.0),
                "homogeneity failed at {d}"
            );
        }
    }

    // --- Tables D.3b / D.3c (calculation partitions, 44,1 / 48 kHz) ---

    #[test]
    fn d3bc_partition_counts_match_spec() {
        assert_eq!(CALC_PARTITION_44K1.len(), 57);
        assert_eq!(CALC_PARTITION_48K.len(), 58);
        assert_eq!(MODEL2_PARTITIONS_44K1, 57);
        assert_eq!(MODEL2_PARTITIONS_48K, 58);
        // Strictly the most partitions at the highest rate, fewest at
        // 32 kHz: 49 < 57 < 58.
        assert!(CALC_PARTITION_32K.len() < CALC_PARTITION_44K1.len());
        assert!(CALC_PARTITION_44K1.len() < CALC_PARTITION_48K.len());
    }

    #[test]
    fn d3bc_dense_1based_index_numbering() {
        for (k, p) in CALC_PARTITION_44K1.iter().enumerate() {
            assert_eq!(p.index as usize, k + 1, "D.3b slot {k}");
        }
        for (k, p) in CALC_PARTITION_48K.iter().enumerate() {
            assert_eq!(p.index as usize, k + 1, "D.3c slot {k}");
        }
    }

    #[test]
    fn d3bc_partitions_tile_fft_lines_contiguously_to_nyquist() {
        // ωlow_{n+1} = ωhigh_n + 1, first ωlow = 1, last ωhigh = 513
        // (the Nyquist line of the 1024-point Model 2 FFT).
        for table in [&CALC_PARTITION_44K1[..], &CALC_PARTITION_48K[..]] {
            assert_eq!(table[0].omega_low, 1);
            assert_eq!(table.last().unwrap().omega_high, 513);
            for w in table.windows(2) {
                assert_eq!(
                    w[1].omega_low,
                    w[0].omega_high + 1,
                    "gap/overlap at partition {}",
                    w[1].index
                );
                assert!(
                    w[1].omega_high >= w[1].omega_low,
                    "empty partition {}",
                    w[1].index
                );
            }
        }
    }

    #[test]
    fn d3bc_bval_strictly_monotonic_and_endpoints() {
        for table in [&CALC_PARTITION_44K1[..], &CALC_PARTITION_48K[..]] {
            assert!((table[0].bval - 0.00).abs() < 1e-9);
            for w in table.windows(2) {
                assert!(
                    w[1].bval > w[0].bval,
                    "bval non-monotone at partition {}",
                    w[1].index
                );
            }
        }
        // Last-partition bval per rate (printed): 25,33 / 25,81.
        assert!((CALC_PARTITION_44K1.last().unwrap().bval - 25.33).abs() < 1e-9);
        assert!((CALC_PARTITION_48K.last().unwrap().bval - 25.81).abs() < 1e-9);
    }

    #[test]
    fn d3bc_head_and_minval_floor_drop_verbatim() {
        // Head rows verbatim (index 1..=3) from the CSV extracts.
        let b1 = calc_partition_44k1(1).unwrap();
        assert_eq!((b1.omega_low, b1.omega_high), (1, 1));
        assert!((b1.bval - 0.00).abs() < 1e-9 && (b1.tmn - 24.5).abs() < 1e-9);
        let c1 = calc_partition_48k(1).unwrap();
        assert_eq!((c1.omega_low, c1.omega_high), (1, 1));
        assert!((c1.tmn - 24.5).abs() < 1e-9);
        // The errata-noted minval drop to 3,5 dB on the final
        // partition(s): D.3b row 57 and D.3c rows 57–58.
        assert!((calc_partition_44k1(57).unwrap().minval - 3.5).abs() < 1e-9);
        assert!((calc_partition_44k1(56).unwrap().minval - 4.5).abs() < 1e-9);
        assert!((calc_partition_48k(57).unwrap().minval - 3.5).abs() < 1e-9);
        assert!((calc_partition_48k(58).unwrap().minval - 3.5).abs() < 1e-9);
        assert!((calc_partition_48k(56).unwrap().minval - 4.5).abs() < 1e-9);
    }

    #[test]
    fn d3bc_lookup_bounds() {
        assert!(calc_partition_44k1(0).is_none());
        assert!(calc_partition_44k1(57).is_some());
        assert!(calc_partition_44k1(58).is_none());
        assert!(calc_partition_48k(0).is_none());
        assert!(calc_partition_48k(58).is_some());
        assert!(calc_partition_48k(59).is_none());
    }

    // --- Model 2 spreading operator at 44,1 / 48 kHz ---

    #[test]
    fn model2_spread_weight_higher_rates_diagonal_is_one() {
        for n in 1..=MODEL2_PARTITIONS_44K1 as u16 {
            assert!((model2_spread_weight_44k1(n, n).unwrap() - 1.0).abs() < 1e-6);
        }
        for n in 1..=MODEL2_PARTITIONS_48K as u16 {
            assert!((model2_spread_weight_48k(n, n).unwrap() - 1.0).abs() < 1e-6);
        }
        assert!(model2_spread_weight_44k1(58, 1).is_none());
        assert!(model2_spread_weight_48k(59, 1).is_none());
    }

    #[test]
    fn model2_spread_weight_higher_rates_match_sprdngf() {
        for &(into, from) in &[(1u16, 1u16), (10, 8), (8, 10), (57, 1), (30, 35)] {
            let got = model2_spread_weight_44k1(into, from).unwrap();
            let want = model2_sprdngf(
                calc_partition_44k1(into).unwrap().bval,
                calc_partition_44k1(from).unwrap().bval,
            );
            assert!((got - want).abs() < 1e-15);
        }
        for &(into, from) in &[(1u16, 1u16), (12, 9), (58, 1), (40, 50)] {
            let got = model2_spread_weight_48k(into, from).unwrap();
            let want = model2_sprdngf(
                calc_partition_48k(into).unwrap().bval,
                calc_partition_48k(from).unwrap().bval,
            );
            assert!((got - want).abs() < 1e-15);
        }
    }

    #[test]
    fn model2_spreading_matrix_higher_rates_shape_and_diagonal() {
        let m44 = model2_spreading_matrix_44k1();
        assert_eq!(m44.len(), MODEL2_PARTITIONS_44K1);
        assert!(m44.iter().all(|r| r.len() == MODEL2_PARTITIONS_44K1));
        for (d, row) in m44.iter().enumerate() {
            assert!((row[d] - 1.0).abs() < 1e-6, "44.1k diagonal {d}");
            assert!(row.iter().all(|&w| (0.0..=1.0 + 1e-9).contains(&w)));
        }
        let m48 = model2_spreading_matrix_48k();
        assert_eq!(m48.len(), MODEL2_PARTITIONS_48K);
        for (d, row) in m48.iter().enumerate() {
            assert!((row[d] - 1.0).abs() < 1e-6, "48k diagonal {d}");
        }
    }

    #[test]
    fn model2_spread_energy_higher_rates_reject_wrong_length() {
        assert!(model2_spread_energy_44k1(&[]).is_none());
        assert!(model2_spread_energy_44k1(&[1.0; MODEL2_PARTITIONS_44K1 - 1]).is_none());
        assert!(model2_spread_energy_44k1(&[1.0; MODEL2_PARTITIONS_44K1]).is_some());
        assert!(model2_spread_energy_48k(&[1.0; MODEL2_PARTITIONS_48K + 1]).is_none());
        assert!(model2_spread_energy_48k(&[1.0; MODEL2_PARTITIONS_48K]).is_some());
    }

    #[test]
    fn model2_spread_energy_higher_rates_conserve_total_energy() {
        // The rnorm-normalised operator redistributes without creating
        // or destroying energy: Σ eb == Σ source.
        for (n, op) in [
            (
                MODEL2_PARTITIONS_44K1,
                model2_spread_energy_44k1 as fn(&[f64]) -> Option<Vec<f64>>,
            ),
            (MODEL2_PARTITIONS_48K, model2_spread_energy_48k),
        ] {
            let energy: Vec<f64> = (0..n)
                .map(|s| (s as f64 * 0.7).sin().abs() + 0.05)
                .collect();
            let total: f64 = energy.iter().sum();
            let eb = op(&energy).unwrap();
            let spread: f64 = eb.iter().sum();
            assert!(
                (spread - total).abs() <= 1e-9 * total,
                "energy not conserved: {spread} vs {total}"
            );
        }
    }

    #[test]
    fn model2_spread_energy_higher_rates_unit_impulse_normalized() {
        // A unit impulse at one source spreads to a total of exactly 1
        // across all destinations (the rnorm energy-conservation
        // property), at both higher rates.
        for (n, op) in [
            (
                MODEL2_PARTITIONS_44K1,
                model2_spread_energy_44k1 as fn(&[f64]) -> Option<Vec<f64>>,
            ),
            (MODEL2_PARTITIONS_48K, model2_spread_energy_48k),
        ] {
            for s in [0usize, n / 2, n - 1] {
                let mut energy = vec![0.0; n];
                energy[s] = 1.0;
                let eb = op(&energy).unwrap();
                let total: f64 = eb.iter().sum();
                assert!((total - 1.0).abs() < 1e-9, "impulse at {s}: total {total}");
            }
        }
    }

    // --- Tables D.4a–c (Model 2 per-FFT-line absolute threshold) ---

    #[test]
    fn d4_row_counts_and_last_covered_line() {
        assert_eq!(ABSTHR_D4A_32K.len(), 132);
        assert_eq!(ABSTHR_D4B_44K1.len(), 130);
        assert_eq!(ABSTHR_D4C_48K.len(), 126);
        assert_eq!(ABSTHR_D4A_32K.last().unwrap().line_high, 480);
        assert_eq!(ABSTHR_D4B_44K1.last().unwrap().line_high, 464);
        assert_eq!(ABSTHR_D4C_48K.last().unwrap().line_high, 432);
    }

    #[test]
    fn d4_ranges_tile_fft_lines_contiguously_from_1() {
        for table in [
            &ABSTHR_D4A_32K[..],
            &ABSTHR_D4B_44K1[..],
            &ABSTHR_D4C_48K[..],
        ] {
            assert_eq!(table[0].line_low, 1);
            for r in table {
                assert!(r.line_high >= r.line_low, "inverted range {r:?}");
                assert_eq!(r.width(), r.line_high - r.line_low + 1);
            }
            for w in table.windows(2) {
                assert_eq!(
                    w[1].line_low,
                    w[0].line_high + 1,
                    "non-contiguous at {:?} -> {:?}",
                    w[0],
                    w[1]
                );
            }
        }
    }

    #[test]
    fn d4_head_rows_verbatim() {
        // First rows of each table verbatim from the CSV extracts.
        assert_eq!(
            ABSTHR_D4A_32K[0],
            AbsThrRange {
                line_low: 1,
                line_high: 1,
                absthr_db: 58.23
            }
        );
        assert_eq!(
            ABSTHR_D4B_44K1[0],
            AbsThrRange {
                line_low: 1,
                line_high: 1,
                absthr_db: 45.05
            }
        );
        assert_eq!(
            ABSTHR_D4C_48K[0],
            AbsThrRange {
                line_low: 1,
                line_high: 1,
                absthr_db: 42.10
            }
        );
    }

    #[test]
    fn d4_documented_ceilings_and_minima() {
        // D.4a final range 473–480 prints 51,03 dB (the documented
        // last-digit divergence from D.1d's 51,04).
        assert!((ABSTHR_D4A_32K.last().unwrap().absthr_db - 51.03).abs() < 1e-9);
        // D.4b's distinctive 69,13 dB ceiling on its final range.
        assert!((ABSTHR_D4B_44K1.last().unwrap().absthr_db - 69.13).abs() < 1e-9);
        // D.4c matches its D.1f twin's 68,00 dB ceiling.
        assert!((ABSTHR_D4C_48K.last().unwrap().absthr_db - 68.00).abs() < 1e-9);
        // Per-table threshold minima from the extract.
        let mins = [
            (&ABSTHR_D4A_32K[..], -4.97),
            (&ABSTHR_D4B_44K1[..], -4.98),
            (&ABSTHR_D4C_48K[..], -4.98),
        ];
        for (table, expect) in mins {
            let got = table
                .iter()
                .map(|r| r.absthr_db)
                .fold(f64::INFINITY, f64::min);
            assert!((got - expect).abs() < 1e-9, "min {got} != {expect}");
        }
    }

    #[test]
    fn d4_line_lookup_resolves_within_range() {
        // Every FFT line in a range resolves to that range's threshold;
        // sample both endpoints and an interior line of a wide range.
        // D.4a final range 473–480 = 51,03 dB.
        for line in [473u16, 476, 480] {
            assert!((absthr_for_line_32k(line).unwrap() - 51.03).abs() < 1e-9);
        }
        // Line 1 hits the head range of each table.
        assert!((absthr_for_line_32k(1).unwrap() - 58.23).abs() < 1e-9);
        assert!((absthr_for_line_44k1(1).unwrap() - 45.05).abs() < 1e-9);
        assert!((absthr_for_line_48k(1).unwrap() - 42.10).abs() < 1e-9);
        // A line in D.4b's 69,13 ceiling stretch.
        assert!((absthr_for_line_44k1(464).unwrap() - 69.13).abs() < 1e-9);
    }

    #[test]
    fn d4_line_lookup_matches_linear_scan_for_every_covered_line() {
        // The binary-search lookup must agree with a brute-force scan
        // over every covered FFT line, for all three rates.
        for table_fn in [
            (
                &ABSTHR_D4A_32K[..],
                absthr_for_line_32k as fn(u16) -> Option<f64>,
            ),
            (&ABSTHR_D4B_44K1[..], absthr_for_line_44k1),
            (&ABSTHR_D4C_48K[..], absthr_for_line_48k),
        ] {
            let (table, lookup) = table_fn;
            let last = table.last().unwrap().line_high;
            for line in 1..=last {
                let scan = table
                    .iter()
                    .find(|r| line >= r.line_low && line <= r.line_high)
                    .map(|r| r.absthr_db);
                assert_eq!(lookup(line), scan, "mismatch at line {line}");
            }
        }
    }

    #[test]
    fn d4_line_lookup_none_out_of_range() {
        assert!(absthr_for_line_32k(0).is_none());
        assert!(absthr_for_line_32k(481).is_none());
        assert!(absthr_for_line_44k1(0).is_none());
        assert!(absthr_for_line_44k1(465).is_none());
        assert!(absthr_for_line_48k(0).is_none());
        assert!(absthr_for_line_48k(433).is_none());
        assert!(absthr_for_line_48k(u16::MAX).is_none());
    }
}
