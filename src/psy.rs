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
//! * The same file points at PNG renders of Tables D.1a-f, D.3a-c, and
//!   D.4a-c — those values still live behind a render the text layer
//!   does not preserve and are **NOT** in this module (DOCS-GAP for a
//!   later round; their absence does not block the formulas below).
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
//!
//! No allocator wiring is changed in this round; the existing
//! [`crate::encode::allocate_bits`] path still drives subband bits
//! from signal energy. Wiring the global masking threshold into the
//! allocator requires Tables D.1 (threshold-in-quiet) and D.3 / D.4
//! (Model 2 partition + per-line LTq), which remain PNG-only.

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
// Annex D clause D.2 (Psychoacoustic Model 2) — spreading function
// closed-form pieces (text-extractable)
//
// Quoted verbatim from the staged docs extract (the `tmpx` and `x`
// lines extract from the PDF text layer; the `tmpy` intermediate
// line is typeset as an image and remains a DOCS-GAP). The post-step
// `sprdngf` cutoff is text-extractable.
//
//   tmpx = 1,05 * (j - i)         ; i = Bark of signal spread,
//                                   j = Bark of band spread into
//   x    = 8 * minimum( (tmpx - 0,5)^2 - 2*(tmpx - 0,5), 0 )
//   tmpy = [illegible — typeset as image in PDF]
//   if (tmpy < -100) then sprdngf(i,j) = 0
//                    else sprdngf(i,j) = 10^(tmpy/10)
//
// `model2_tmpx` and `model2_x` ARE staged here; the `tmpy` →
// `sprdngf` post-step is staged as `sprdngf_from_tmpy` so callers
// (once `tmpy` becomes text-extractable) can plug a `tmpy` value
// straight into the legible cutoff. Intentionally absent: the
// `tmpy` line itself.
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

/// Model 2 spreading-function post-step (clause D.2, text-extracted
/// verbatim): given the intermediate `tmpy` (dB), returns the
/// spreading-function magnitude
/// `sprdngf(i, j) = 0` when `tmpy < -100`, else `10^(tmpy / 10)`.
///
/// `tmpy` itself is the spreading-function dB level; the line that
/// derives it from `x` / `bval` is rendered as a PDF image and is
/// **not** captured in this module. Once the docs collaborator stages
/// a text-extracted `tmpy` formula, the missing piece can be plugged in
/// without changing the cutoff staged here.
#[inline]
pub fn sprdngf_from_tmpy(tmpy_db: f64) -> f64 {
    if tmpy_db < -100.0 {
        0.0
    } else {
        10f64.powf(tmpy_db / 10.0)
    }
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
}
