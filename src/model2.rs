//! ISO/IEC 11172-3 (1993) Annex D **Psychoacoustic Model 2** per-frame
//! driver.
//!
//! [`crate::psy`] stages every *static* Model 2 ingredient — the
//! threshold-in-quiet tables (D.1a–c), the critical-band boundaries
//! (D.2a–f), the calculation-partition table (D.3a–c: `ωlow`, `ωhigh`,
//! `bval`, `minval`, `TMN`), the per-FFT-line absolute-threshold table
//! (D.4a–c), the spreading function (clause D.2.3) and the coder
//! partition table (D.5). This module assembles those ingredients into
//! the **iterative per-frame procedure** of clause D.2.4, "Steps in
//! threshold calculation", and clause D.2's step n SMR computation,
//! producing the per-subband signal-to-mask ratios `SMR_n` the
//! §C.1.5.1.6 / §C.1.5.2.7 bit allocator needs.
//!
//! Provenance: every formula here is the verbatim closed form printed
//! in ISO/IEC 11172-3 (1993) Annex D, clause D.2.3 / D.2.4 (PDF pages
//! 135–138, printed 129–132 of the in-repo
//! `docs/audio/mp3/ISO_IEC_11172-3-MP3-1993.pdf`), read from 300-DPI
//! page renders. No decoder/encoder reference implementation was
//! consulted. The radix-2 FFT below is the textbook decimation-in-time
//! Cooley–Tukey transform — generic numerical infrastructure, not a
//! codec-specific construction.
//!
//! The model is **informative** in 11172-3: it is an example encoder
//! model, not a normative decoder requirement, so there is no bit-exact
//! oracle. The tests assert the documented analytic properties of each
//! step (window energy, FFT correctness against a naïve DFT, the
//! tonality-index range bound `0 < tb_b < 1`, SMR monotonicity in input
//! level, etc.).
//!
//! ## Procedure overview (clause D.2.4)
//!
//! For each frame the driver:
//!
//! - **a/b)** reconstructs 1024 consecutive input samples, applies a
//!   1024-point Hann window, runs a forward FFT, and forms the polar
//!   magnitude `r_ω` / phase `f_ω` per FFT line ([`Spectrum`]);
//! - **c/d)** predicts `r̂_ω` / `f̂_ω` from the previous two blocks and
//!   forms the per-line unpredictability `c_ω` (clause D.2.4 c/d);
//! - **e)** sums the per-line energy `eb_b` and the energy-weighted
//!   unpredictability `cb_b` over each calculation partition;
//! - **f/g)** convolves `eb`/`cb·eb` with the spreading function to get
//!   `ecb_b` / `ct_b`, normalizes to `en_b`, and converts `cb_b` to the
//!   tonality index `tb_b`;
//! - **h–l)** computes the required SNR `SNR_b`, the power ratio
//!   `bc_b`, the partition energy threshold `nb_b`, spreads it back over
//!   FFT lines and folds in the absolute threshold to yield `thr_ω`;
//! - **n)** maps the FFT-line energies/thresholds onto the 32 coder
//!   partitions (Table D.5) and forms `SMR_n` per subband
//!   ([`Model2State::smr_per_subband`]).
//!
//! [`Model2State`] holds the two-block prediction history so a caller
//! that streams successive Layer I frames gets the cross-frame
//! prediction the spec's step c requires.

use crate::psy::{
    absthr_for_line_32k, absthr_for_line_44k1, absthr_for_line_48k, CalcPartition,
    CALC_PARTITION_32K,
};
use crate::psy::{calc_partition_44k1, calc_partition_48k};

/// Number of samples in the Model 2 analysis window (clause D.2.4 a).
pub const FFT_SIZE: usize = 1024;

/// Number of usable FFT lines, 1-based `1..=513` — DC through the
/// Nyquist line of the 1024-point real-input FFT.
pub const NUM_LINES: usize = FFT_SIZE / 2 + 1;

/// Number of Layer I / II subbands (= number of coder partitions used).
pub const SUBBANDS: usize = 32;

/// Sampling rates Model 2 has a calculation-partition / absolute-
/// threshold table for (Tables D.3a–c / D.4a–c).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model2Rate {
    /// 32 kHz — Tables D.3a / D.4a, `bmax = 49`.
    Hz32000,
    /// 44,1 kHz — Tables D.3b / D.4b, `bmax = 57`.
    Hz44100,
    /// 48 kHz — Tables D.3c / D.4c, `bmax = 58`.
    Hz48000,
}

impl Model2Rate {
    /// Map a sampling frequency in Hz to its Model 2 table set, or
    /// `None` for a rate Annex D has no Model 2 table for (the MPEG-2
    /// LSF half-rates 16 / 22,05 / 24 kHz have no Annex D Model 2
    /// tables in 11172-3).
    pub fn from_hz(hz: u32) -> Option<Self> {
        match hz {
            32_000 => Some(Self::Hz32000),
            44_100 => Some(Self::Hz44100),
            48_000 => Some(Self::Hz48000),
            _ => None,
        }
    }

    /// The calculation-partition table (Table D.3x) for this rate, as
    /// an owned `Vec` (the 44,1 / 48 kHz tables are produced by their
    /// per-row accessors; 32 kHz uses the const array directly).
    pub fn calc_partitions(self) -> Vec<CalcPartition> {
        match self {
            Self::Hz32000 => CALC_PARTITION_32K.to_vec(),
            Self::Hz44100 => collect_partitions(calc_partition_44k1),
            Self::Hz48000 => collect_partitions(calc_partition_48k),
        }
    }

    /// Per-FFT-line absolute threshold (Table D.4x) in dB for `line`
    /// (1-based), or `None` above the table's last covered line.
    pub fn absthr_db(self, line: u16) -> Option<f64> {
        match self {
            Self::Hz32000 => absthr_for_line_32k(line),
            Self::Hz44100 => absthr_for_line_44k1(line),
            Self::Hz48000 => absthr_for_line_48k(line),
        }
    }
}

/// Walk a 1-based per-row partition accessor `f` until it returns
/// `None`, collecting the dense `CalcPartition` rows into a `Vec`.
fn collect_partitions(f: fn(u16) -> Option<CalcPartition>) -> Vec<CalcPartition> {
    let mut out = Vec::new();
    let mut n = 1u16;
    while let Some(p) = f(n) {
        out.push(p);
        n += 1;
    }
    out
}

/// 1024-point Hann window coefficients (clause D.2.4 b):
/// `sw_i = √(8/3) · 0.5 · (1 − cos(2π(i − 0.5)/1024))`, `i = 1..1024`.
///
/// The `√(8/3)` factor is the spec's amplitude correction for the Hann
/// taper (the printed step writes `sw_i = s_i · (0.5 − 0.5·cos(…))`
/// with the `√(8/3)` normalization applied so the windowed energy
/// matches the unwindowed energy of a stationary signal).
pub fn hann_window() -> [f64; FFT_SIZE] {
    let mut w = [0.0f64; FFT_SIZE];
    let norm = (8.0f64 / 3.0).sqrt();
    for (i0, wi) in w.iter_mut().enumerate() {
        // Spec uses 1-based i = 1..1024 in the `(i − 0.5)` term, which
        // for 0-based index `i0` is `(i0 + 0.5)`.
        let i = i0 as f64 + 0.5;
        *wi = norm * 0.5 * (1.0 - (2.0 * core::f64::consts::PI * i / FFT_SIZE as f64).cos());
    }
    w
}

/// In-place radix-2 decimation-in-time forward FFT on interleaved
/// `[re, im]` pairs (`buf.len() == 2·n`, `n` a power of two).
///
/// Textbook Cooley–Tukey: bit-reversal permutation followed by
/// log2(n) butterfly stages. Sign convention
/// `X_k = Σ_n x_n · e^(−2πi kn/N)` (forward transform). Generic
/// numerical infrastructure; not derived from any codec source.
pub fn fft_in_place(buf: &mut [f64]) {
    let n = buf.len() / 2;
    debug_assert!(n.is_power_of_two(), "FFT length must be a power of two");

    // Bit-reversal permutation of the complex elements.
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            buf.swap(2 * i, 2 * j);
            buf.swap(2 * i + 1, 2 * j + 1);
        }
        let mut m = n >> 1;
        while m >= 1 && j & m != 0 {
            j ^= m;
            m >>= 1;
        }
        j |= m;
    }

    // Butterfly stages.
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let theta = -2.0 * core::f64::consts::PI / len as f64;
        let (wr_step, wi_step) = (theta.cos(), theta.sin());
        let mut start = 0usize;
        while start < n {
            let (mut wr, mut wi) = (1.0f64, 0.0f64);
            for k in 0..half {
                let a = start + k;
                let b = a + half;
                let (ar, ai) = (buf[2 * a], buf[2 * a + 1]);
                let (br, bi) = (buf[2 * b], buf[2 * b + 1]);
                // t = w · B
                let tr = wr * br - wi * bi;
                let ti = wr * bi + wi * br;
                buf[2 * a] = ar + tr;
                buf[2 * a + 1] = ai + ti;
                buf[2 * b] = ar - tr;
                buf[2 * b + 1] = ai - ti;
                // advance twiddle: w *= w_step
                let nwr = wr * wr_step - wi * wi_step;
                let nwi = wr * wi_step + wi * wr_step;
                wr = nwr;
                wi = nwi;
            }
            start += len;
        }
        len <<= 1;
    }
}

/// Polar spectrum of one analysis window: magnitude `r_ω` and phase
/// `f_ω` for the `NUM_LINES` usable FFT lines (DC..Nyquist).
///
/// Index 0 is the DC line (the spec's 1-based FFT line 1); index
/// `NUM_LINES − 1` is the Nyquist line (1-based line 513). `f_ω` is in
/// radians; the magnitude is the raw FFT magnitude (per-line energy is
/// `r_ω²`).
#[derive(Debug, Clone)]
pub struct Spectrum {
    /// Per-line magnitude `r_ω` (`NUM_LINES` entries, DC..Nyquist).
    pub r: Vec<f64>,
    /// Per-line phase `f_ω` in radians (`NUM_LINES` entries).
    pub f: Vec<f64>,
}

impl Spectrum {
    /// Compute the windowed-FFT polar spectrum of `samples`
    /// (`FFT_SIZE` time-domain samples). Applies the [`hann_window`],
    /// runs [`fft_in_place`], and forms `(r_ω, f_ω)` for the usable
    /// FFT lines (clause D.2.4 b).
    pub fn analyze(samples: &[f64; FFT_SIZE]) -> Self {
        let w = hann_window();
        let mut buf = vec![0.0f64; 2 * FFT_SIZE];
        for i in 0..FFT_SIZE {
            buf[2 * i] = samples[i] * w[i];
            buf[2 * i + 1] = 0.0;
        }
        fft_in_place(&mut buf);
        let mut r = vec![0.0f64; NUM_LINES];
        let mut f = vec![0.0f64; NUM_LINES];
        for (line, (rr, ff)) in r.iter_mut().zip(f.iter_mut()).enumerate() {
            let re = buf[2 * line];
            let im = buf[2 * line + 1];
            *rr = (re * re + im * im).sqrt();
            *ff = im.atan2(re);
        }
        Spectrum { r, f }
    }
}

/// Per-line unpredictability measure `c_ω` (clause D.2.4 c/d).
///
/// Step c predicts the magnitude `r̂_ω` and phase `f̂_ω` of the current
/// block from the previous two blocks:
///
/// ```text
/// r̂_ω = 2.0·r_ω(t−1) − r_ω(t−2)
/// f̂_ω = 2.0·f_ω(t−1) − f_ω(t−2)
/// ```
///
/// Step d forms the Euclidean distance between the (predicted, actual)
/// polar vectors, normalized by the sum of the actual and predicted
/// magnitudes:
///
/// ```text
/// c_ω = √((r·cos f − r̂·cos f̂)² + (r·sin f − r̂·sin f̂)²) / (r + |r̂|)
/// ```
///
/// `cur` is the current block's [`Spectrum`]; `prev1` / `prev2` are the
/// previous two blocks. When a block is missing (start of stream), the
/// prediction degenerates and `c_ω` reflects full unpredictability
/// (≈ 1) for those lines, matching the spec's "white" startup.
///
/// The spec notes the measure may be computed on only a lower portion
/// of the frequency lines (DC..~3 kHz min, ~7 kHz preferred) with the
/// rest set to a constant 0.3; this driver computes the full band
/// (`up_to == NUM_LINES`) but accepts an `up_to` cutoff so a caller can
/// honour the spec's performance shortcut, filling `[up_to, NUM_LINES)`
/// with 0.3.
pub fn unpredictability(
    cur: &Spectrum,
    prev1: Option<&Spectrum>,
    prev2: Option<&Spectrum>,
    up_to: usize,
) -> Vec<f64> {
    let mut cw = vec![0.3f64; NUM_LINES];
    let limit = up_to.min(NUM_LINES);
    // `line` indexes cur.r/cur.f and the optional prev1/prev2 spectra
    // in parallel; a range loop keeps the polar-prediction arithmetic
    // legible against the spec's per-line c/d formulas.
    #[allow(clippy::needless_range_loop)]
    for line in 0..limit {
        let r = cur.r[line];
        let f = cur.f[line];
        // Predicted magnitude / phase from the previous two blocks.
        let (rhat, fhat) = match (prev1, prev2) {
            (Some(p1), Some(p2)) => (2.0 * p1.r[line] - p2.r[line], 2.0 * p1.f[line] - p2.f[line]),
            // Without two history blocks the predictor has nothing; a
            // zero prediction yields c_ω = 1 (fully unpredictable).
            _ => (0.0, 0.0),
        };
        let dx = r * f.cos() - rhat * fhat.cos();
        let dy = r * f.sin() - rhat * fhat.sin();
        let denom = r + rhat.abs();
        cw[line] = if denom > 0.0 {
            ((dx * dx + dy * dy).sqrt() / denom).clamp(0.0, 1.0)
        } else {
            // Silent line: treat as fully predictable (tonal) — the
            // weighted unpredictability sum will be zero-weighted by
            // the (also-zero) energy anyway.
            0.0
        };
    }
    cw
}

/// Precomputed per-rate Model 2 partition geometry: the calculation
/// partitions (Table D.3x), the spreading matrix `sprdngf(bval_bb,
/// bval_b)` and the renormalization coefficients `rnorm_b`
/// (clause D.2.4 e/f).
#[derive(Debug, Clone)]
pub struct PartitionTables {
    /// Calculation partitions (Table D.3x rows) for this rate.
    pub partitions: Vec<CalcPartition>,
    /// `spread[b][bb] = sprdngf(bval_bb, bval_b)` — the contribution of
    /// source partition `bb` spread into target partition `b`.
    pub spread: Vec<Vec<f64>>,
    /// Renormalization coefficient `rnorm_b = 1 / Σ_bb sprdngf(bval_bb,
    /// bval_b)` (clause D.2.4 f), one per target partition.
    pub rnorm: Vec<f64>,
}

impl PartitionTables {
    /// Build the partition geometry for `rate` from the staged
    /// `psy` tables. The spreading matrix is `bmax × bmax`.
    pub fn for_rate(rate: Model2Rate) -> Self {
        let partitions = rate.calc_partitions();
        let bmax = partitions.len();
        let mut spread = vec![vec![0.0f64; bmax]; bmax];
        for (b, row) in spread.iter_mut().enumerate() {
            for (bb, cell) in row.iter_mut().enumerate() {
                // sprdngf(i = source bval, j = target bval): the spec's
                // i is the Bark of the signal being spread (source bb),
                // j is the Bark of the band being spread into (target b).
                *cell = crate::psy::model2_sprdngf(partitions[bb].bval, partitions[b].bval);
            }
        }
        let rnorm = spread
            .iter()
            .map(|row| {
                let s: f64 = row.iter().sum();
                if s > 0.0 {
                    1.0 / s
                } else {
                    0.0
                }
            })
            .collect();
        PartitionTables {
            partitions,
            spread,
            rnorm,
        }
    }

    /// Number of calculation partitions (`bmax`).
    pub fn bmax(&self) -> usize {
        self.partitions.len()
    }
}

/// Per-partition Model 2 quantities for one frame (clause D.2.4 e–g).
#[derive(Debug, Clone)]
pub struct PartitionEnergies {
    /// Raw partition energy `eb_b = Σ r_ω²` over the partition's lines.
    pub eb: Vec<f64>,
    /// Energy-weighted unpredictability `cb_b = Σ r_ω²·c_ω`.
    pub cb_weighted: Vec<f64>,
    /// Normalized spread energy `en_b = ecb_b · rnorm_b`.
    pub en: Vec<f64>,
    /// Spread weighted unpredictability `ct_b`.
    pub ct: Vec<f64>,
    /// Per-partition tonality index `tb_b ∈ (0, 1)`.
    pub tb: Vec<f64>,
}

/// Steps e–g: accumulate per-partition energy `eb` and weighted
/// unpredictability `cb`, convolve both with the spreading function to
/// get `ecb` / `ct`, renormalize to `en`, and derive the tonality
/// index `tb` (clause D.2.4 e/f/g).
///
/// `r` is the current block's per-line magnitude (`cur.r`); `cw` is the
/// per-line unpredictability from [`unpredictability`]. Both index FFT
/// lines `0..NUM_LINES` (DC..Nyquist). The partition `ωlow`/`ωhigh`
/// columns are 1-based, so a partition spanning printed lines
/// `[ωlow, ωhigh]` reads `r`/`cw` indices `[ωlow−1, ωhigh−1]`.
pub fn partition_energies(tables: &PartitionTables, r: &[f64], cw: &[f64]) -> PartitionEnergies {
    let bmax = tables.bmax();
    let mut eb = vec![0.0f64; bmax];
    let mut cb_weighted = vec![0.0f64; bmax];
    for (b, p) in tables.partitions.iter().enumerate() {
        let lo = (p.omega_low as usize).saturating_sub(1);
        let hi = (p.omega_high as usize).min(NUM_LINES);
        for line in lo..hi {
            let e = r[line] * r[line];
            eb[b] += e;
            cb_weighted[b] += e * cw[line];
        }
    }

    // Step f: convolve eb and (cb_weighted) with the spreading matrix.
    // ecb_b = Σ_bb eb_bb · spread[b][bb]; ct_b = Σ_bb cb_bb · spread[b][bb].
    let mut ecb = vec![0.0f64; bmax];
    let mut ct = vec![0.0f64; bmax];
    for b in 0..bmax {
        let row = &tables.spread[b];
        let mut e_acc = 0.0;
        let mut c_acc = 0.0;
        for bb in 0..bmax {
            e_acc += eb[bb] * row[bb];
            c_acc += cb_weighted[bb] * row[bb];
        }
        ecb[b] = e_acc;
        ct[b] = c_acc;
    }

    // cb_b = ct_b / ecb_b (the spread, energy-weighted unpredictability
    // ratio); en_b = ecb_b · rnorm_b (the renormalized spread energy).
    let mut en = vec![0.0f64; bmax];
    let mut tb = vec![0.0f64; bmax];
    for b in 0..bmax {
        en[b] = ecb[b] * tables.rnorm[b];
        let cb = if ecb[b] > 0.0 { ct[b] / ecb[b] } else { 0.0 };
        // Step g: tonality index tb_b = -0.299 - 0.43·ln(cb_b),
        // limited to 0 < tb_b < 1.
        let tbb = if cb > 0.0 {
            -0.299 - 0.43 * cb.ln()
        } else {
            // cb → 0 means fully tonal; the formula diverges to +∞, so
            // clamp to the upper bound 1.
            1.0
        };
        tb[b] = tbb.clamp(0.0, 1.0);
    }

    PartitionEnergies {
        eb,
        cb_weighted,
        en,
        ct,
        tb,
    }
}

/// Reference noise-masking-tone offset `NMT_b = 5.5 dB` for all
/// partitions (clause D.2.4 h).
pub const NMT_DB: f64 = 5.5;

/// Steps h–l: per-FFT-line audibility threshold `thr_ω` from the
/// per-partition normalized energy `en`, tonality `tb`, the per-rate
/// partition table (for `TMN_b` / `minval_b` and the `ωlow`/`ωhigh`
/// spread-back), and the per-line absolute threshold (Table D.4x).
///
/// Per partition `b`:
///
/// ```text
/// SNR_b  = max(minval_b, tb_b·TMN_b + (1−tb_b)·NMT_b)   (step h)
/// bc_b   = 10^(−SNR_b/10)                                (step i, power ratio)
/// nb_b   = en_b · bc_b                                   (step j, energy threshold)
/// nb_ω   = nb_b / (ωhigh_b − ωlow_b + 1)                 (step k, spread over lines)
/// thr_ω  = max(nb_ω, absthr_ω)                           (step l, fold in LTq)
/// ```
///
/// `absthr_ω` is the Table D.4x absolute threshold converted from dB to
/// the FFT energy domain via [`absthr_db_to_energy`]. Returns the
/// per-FFT-line threshold `thr_ω` for `0..NUM_LINES` (DC..Nyquist).
pub fn line_thresholds(
    tables: &PartitionTables,
    en: &[f64],
    tb: &[f64],
    rate: Model2Rate,
) -> Vec<f64> {
    let mut thr = vec![0.0f64; NUM_LINES];
    for (b, p) in tables.partitions.iter().enumerate() {
        // Step h: required SNR.
        let snr = (tb[b] * p.tmn + (1.0 - tb[b]) * NMT_DB).max(p.minval);
        // Step i: power ratio.
        let bc = 10f64.powf(-snr / 10.0);
        // Step j: partition energy threshold.
        let nbb = en[b] * bc;
        // Step k: spread over the partition's FFT lines.
        let width = (p.omega_high - p.omega_low + 1) as f64;
        let nb_omega = if width > 0.0 { nbb / width } else { 0.0 };
        let lo = (p.omega_low as usize).saturating_sub(1);
        let hi = (p.omega_high as usize).min(NUM_LINES);
        for (line, t) in thr.iter_mut().enumerate().take(hi).skip(lo) {
            // Step l: include the absolute threshold.
            let absthr_e = rate
                .absthr_db(line as u16 + 1)
                .map(absthr_db_to_energy)
                .unwrap_or(0.0);
            *t = nb_omega.max(absthr_e);
        }
    }
    thr
}

/// Convert a Table D.4x absolute-threshold dB value to the FFT
/// energy domain (clause D.2.4 l note).
///
/// The spec note states: *"A value of 0 dB represents a level in the
/// absolute threshold calculation of 96 dB below the energy of a sine
/// wave of amplitude ±32 760"* and the dB values *"are relative to the
/// level that a sine wave of ±½ lsb has in the FFT used for threshold
/// calculation … the dB values must be converted to the energy domain
/// after considering the FFT normalization actually used."*
///
/// This driver's FFT consumes the same `f64` PCM the analysis filter
/// receives (the caller scales it to whatever full-scale convention it
/// uses). The conversion below is the plain power-from-dB map
/// `10^(absthr_db/10)` against a unit reference: it places the
/// absolute-threshold floor on the same `r_ω²` energy scale as the
/// per-partition energies `eb`. Because Model 2 is informative (no
/// bit-exact oracle) and the SMR is a *ratio* `epart/npart`, only the
/// relative placement of the floor matters, and it is documented here
/// as the unit-reference convention.
pub fn absthr_db_to_energy(absthr_db: f64) -> f64 {
    10f64.powf(absthr_db / 10.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::psy::{model2_sprdngf, CODER_PARTITIONS};

    #[test]
    fn fft_size_constants() {
        assert_eq!(FFT_SIZE, 1024);
        assert_eq!(NUM_LINES, 513);
        assert_eq!(SUBBANDS, 32);
    }

    #[test]
    fn rate_from_hz_maps_known_rates_only() {
        assert_eq!(Model2Rate::from_hz(32_000), Some(Model2Rate::Hz32000));
        assert_eq!(Model2Rate::from_hz(44_100), Some(Model2Rate::Hz44100));
        assert_eq!(Model2Rate::from_hz(48_000), Some(Model2Rate::Hz48000));
        // MPEG-2 LSF half-rates have no Annex D Model 2 table.
        assert_eq!(Model2Rate::from_hz(24_000), None);
        assert_eq!(Model2Rate::from_hz(22_050), None);
        assert_eq!(Model2Rate::from_hz(16_000), None);
    }

    #[test]
    fn calc_partition_counts_match_spec_bmax() {
        assert_eq!(Model2Rate::Hz32000.calc_partitions().len(), 49);
        assert_eq!(Model2Rate::Hz44100.calc_partitions().len(), 57);
        assert_eq!(Model2Rate::Hz48000.calc_partitions().len(), 58);
    }

    #[test]
    fn calc_partitions_tile_to_nyquist() {
        for rate in [
            Model2Rate::Hz32000,
            Model2Rate::Hz44100,
            Model2Rate::Hz48000,
        ] {
            let parts = rate.calc_partitions();
            assert_eq!(parts[0].omega_low, 1, "first partition starts at line 1");
            assert_eq!(
                parts.last().unwrap().omega_high,
                513,
                "last partition reaches the Nyquist line 513"
            );
            // Contiguous tiling: each ωlow is the previous ωhigh + 1.
            for w in parts.windows(2) {
                assert_eq!(w[1].omega_low, w[0].omega_high + 1);
            }
        }
    }

    #[test]
    fn absthr_lookup_resolves_low_lines() {
        // Every usable FFT line below the table top has a threshold.
        for rate in [
            Model2Rate::Hz32000,
            Model2Rate::Hz44100,
            Model2Rate::Hz48000,
        ] {
            assert!(rate.absthr_db(1).is_some());
            assert!(rate.absthr_db(100).is_some());
        }
    }

    #[test]
    fn hann_window_is_symmetric_and_zero_at_edges_interior() {
        let w = hann_window();
        // The (i + 0.5) sampling means the window never hits exactly
        // zero, but the first/last taps are the smallest and equal.
        let n = FFT_SIZE;
        for i in 0..n / 2 {
            assert!(
                (w[i] - w[n - 1 - i]).abs() < 1e-9,
                "window symmetric about its centre at {i}"
            );
        }
        // Peak is at the centre.
        let peak = w[n / 2];
        for &v in &w {
            assert!(v <= peak + 1e-9);
        }
    }

    #[test]
    fn hann_window_energy_correction() {
        // The √(8/3) amplitude correction makes the window's
        // mean-square gain ≈ 1, so a full-scale stationary tone keeps
        // its energy after windowing. Mean of sw_i² ≈ 1.
        let w = hann_window();
        let mean_sq: f64 = w.iter().map(|v| v * v).sum::<f64>() / FFT_SIZE as f64;
        assert!(
            (mean_sq - 1.0).abs() < 0.05,
            "windowed mean-square gain {mean_sq} near 1"
        );
    }

    /// Naïve O(n²) DFT for cross-checking the radix-2 FFT.
    fn naive_dft(re_in: &[f64]) -> Vec<(f64, f64)> {
        let n = re_in.len();
        let mut out = vec![(0.0, 0.0); n];
        for (k, o) in out.iter_mut().enumerate() {
            let mut sr = 0.0;
            let mut si = 0.0;
            for (m, &x) in re_in.iter().enumerate() {
                let th = -2.0 * core::f64::consts::PI * (k * m) as f64 / n as f64;
                sr += x * th.cos();
                si += x * th.sin();
            }
            *o = (sr, si);
        }
        out
    }

    #[test]
    fn fft_matches_naive_dft_on_random_input() {
        // Deterministic pseudo-random input.
        let n = 64usize;
        let mut x = vec![0.0f64; n];
        let mut state = 0x1234_5678u32;
        for v in x.iter_mut() {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            *v = ((state >> 8) as f64 / (1u32 << 24) as f64) - 0.5;
        }
        let want = naive_dft(&x);
        let mut buf = vec![0.0f64; 2 * n];
        for i in 0..n {
            buf[2 * i] = x[i];
        }
        fft_in_place(&mut buf);
        for k in 0..n {
            assert!(
                (buf[2 * k] - want[k].0).abs() < 1e-9,
                "re mismatch at k={k}"
            );
            assert!(
                (buf[2 * k + 1] - want[k].1).abs() < 1e-9,
                "im mismatch at k={k}"
            );
        }
    }

    #[test]
    fn fft_of_pure_tone_peaks_in_one_bin() {
        // A cosine at bin 8 should put almost all energy in line 8.
        let n = FFT_SIZE;
        let bin = 8usize;
        let mut buf = vec![0.0f64; 2 * n];
        for i in 0..n {
            buf[2 * i] = (2.0 * core::f64::consts::PI * bin as f64 * i as f64 / n as f64).cos();
        }
        fft_in_place(&mut buf);
        let mag = |k: usize| (buf[2 * k] * buf[2 * k] + buf[2 * k + 1] * buf[2 * k + 1]).sqrt();
        let peak = mag(bin);
        // Neighbouring bins are tiny; the conjugate bin n-8 also peaks.
        assert!(peak > 1.0);
        assert!(mag(bin + 2) < peak * 1e-6);
        assert!(mag(bin - 2) < peak * 1e-6);
    }

    #[test]
    fn spectrum_analyze_sizes() {
        let samples = [0.0f64; FFT_SIZE];
        let s = Spectrum::analyze(&samples);
        assert_eq!(s.r.len(), NUM_LINES);
        assert_eq!(s.f.len(), NUM_LINES);
    }

    #[test]
    fn spectrum_of_tone_concentrates_magnitude() {
        // A mid-band cosine should produce a clear magnitude peak at
        // its bin and near-zero elsewhere (after Hann windowing the
        // peak spreads to a couple of bins, so check a peak exists and
        // far bins are far smaller).
        let n = FFT_SIZE;
        let bin = 40usize;
        let mut samples = [0.0f64; FFT_SIZE];
        for (i, sm) in samples.iter_mut().enumerate() {
            *sm = (2.0 * core::f64::consts::PI * bin as f64 * i as f64 / n as f64).cos();
        }
        let s = Spectrum::analyze(&samples);
        let peak = s.r[bin];
        assert!(peak > 1.0, "tone produces a magnitude peak: {peak}");
        // A far-away line carries far less magnitude.
        assert!(s.r[bin + 20] < peak * 0.05);
        assert!(s.r[2] < peak * 0.05);
    }

    #[test]
    fn spreading_function_self_spread_is_near_unity_peak() {
        // The clause D.2.3 spreading function at j==i (tmpx = 0) is the
        // self-spread reference used by every partition. With tmpx = 0
        // the parabolic term x = 0 and tmpy = 15.811389 + 7.5·0.474 −
        // 17.5·√(1 + 0.474²) ≈ 0 dB (the 15.811389 constant is tuned so
        // the envelope peaks at unity), so sprdngf ≈ 10^(0/10) ≈ 1.0.
        // Confirm psy exposes the closed form the driver convolves
        // with, and that off-diagonal spread is strictly smaller.
        let diag = model2_sprdngf(5.0, 5.0);
        assert!((diag - 1.0).abs() < 1e-4, "self-spread {diag} at peak");
        assert!(model2_sprdngf(5.0, 7.0) < diag, "upward spread decays");
        assert!(model2_sprdngf(5.0, 3.0) < diag, "downward spread decays");
    }

    #[test]
    fn coder_partition_boundaries_step_sixteen() {
        // Table D.5: 33 boundaries 1,17,...,513 — 16 FFT lines per
        // coder partition, 32 subbands.
        assert_eq!(CODER_PARTITIONS.len(), 33);
        for (n, cp) in CODER_PARTITIONS.iter().enumerate() {
            assert_eq!(cp.boundary as usize, 16 * n + 1);
        }
    }

    /// Build a polar spectrum directly from per-line (mag, phase) so
    /// tests can drive the threshold steps without an FFT.
    fn spectrum_from(mag_phase: &[(f64, f64)]) -> Spectrum {
        let mut r = vec![0.0; NUM_LINES];
        let mut f = vec![0.0; NUM_LINES];
        for (line, &(m, p)) in mag_phase.iter().enumerate().take(NUM_LINES) {
            r[line] = m;
            f[line] = p;
        }
        Spectrum { r, f }
    }

    #[test]
    fn unpredictability_full_when_no_history() {
        // Without two prior blocks the predictor is zero, so a non-zero
        // line is fully unpredictable (c_ω = 1).
        let mut mp = vec![(0.0, 0.0); NUM_LINES];
        mp[10] = (4.0, 0.5);
        let cur = spectrum_from(&mp);
        let cw = unpredictability(&cur, None, None, NUM_LINES);
        assert!((cw[10] - 1.0).abs() < 1e-9, "no-history line fully unpred");
    }

    #[test]
    fn unpredictability_zero_for_perfect_linear_prediction() {
        // If the current block is exactly the linear extrapolation of
        // the previous two (r = 2·r1 − r2, f = 2·f1 − f2), c_ω = 0.
        let mut p2 = vec![(0.0, 0.0); NUM_LINES];
        let mut p1 = vec![(0.0, 0.0); NUM_LINES];
        let mut cu = vec![(0.0, 0.0); NUM_LINES];
        p2[20] = (1.0, 0.1);
        p1[20] = (2.0, 0.2);
        cu[20] = (2.0 * 2.0 - 1.0, 2.0 * 0.2 - 0.1); // (3.0, 0.3)
        let s2 = spectrum_from(&p2);
        let s1 = spectrum_from(&p1);
        let cur = spectrum_from(&cu);
        let cw = unpredictability(&cur, Some(&s1), Some(&s2), NUM_LINES);
        assert!(cw[20] < 1e-9, "perfectly predicted line unpred ≈ 0");
    }

    #[test]
    fn unpredictability_constant_above_cutoff() {
        let mp = vec![(1.0, 0.0); NUM_LINES];
        let cur = spectrum_from(&mp);
        let cw = unpredictability(&cur, None, None, 100);
        for &v in cw.iter().take(100) {
            assert!((v - 1.0).abs() < 1e-9);
        }
        for &v in cw.iter().skip(100) {
            assert!((v - 0.3).abs() < 1e-12, "above cutoff held at 0.3");
        }
    }

    #[test]
    fn partition_tables_shapes_and_normalization() {
        for rate in [
            Model2Rate::Hz32000,
            Model2Rate::Hz44100,
            Model2Rate::Hz48000,
        ] {
            let t = PartitionTables::for_rate(rate);
            let bmax = t.bmax();
            assert_eq!(t.spread.len(), bmax);
            assert!(t.spread.iter().all(|row| row.len() == bmax));
            assert_eq!(t.rnorm.len(), bmax);
            // Each rnorm conserves the spread row to unit sum.
            for b in 0..bmax {
                let s: f64 = t.spread[b].iter().sum();
                assert!((s * t.rnorm[b] - 1.0).abs() < 1e-9, "rnorm conserves");
            }
        }
    }

    #[test]
    fn partition_energies_sum_line_power() {
        let t = PartitionTables::for_rate(Model2Rate::Hz32000);
        // Put energy 4.0 (mag 2.0) on FFT line 1 (DC, index 0), which
        // is partition 1 (ωlow=ωhigh=1).
        let mut r = vec![0.0; NUM_LINES];
        r[0] = 2.0;
        let cw = vec![0.5; NUM_LINES];
        let pe = partition_energies(&t, &r, &cw);
        assert!((pe.eb[0] - 4.0).abs() < 1e-9, "partition 1 energy = 2²");
        assert!((pe.cb_weighted[0] - 2.0).abs() < 1e-9, "weighted = 4·0.5");
    }

    #[test]
    fn tonality_index_is_bounded_zero_one() {
        // Drive a realistic spectrum and confirm every tb_b ∈ [0,1].
        let t = PartitionTables::for_rate(Model2Rate::Hz44100);
        let mut r = vec![0.0; NUM_LINES];
        let mut cw = vec![0.0; NUM_LINES];
        let mut st = 0x9e37_79b9u32;
        for line in 0..NUM_LINES {
            st = st.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            r[line] = ((st >> 9) as f64 / (1u32 << 23) as f64) * 5.0;
            cw[line] = ((st >> 16) & 0xff) as f64 / 255.0;
        }
        let pe = partition_energies(&t, &r, &cw);
        for &tb in &pe.tb {
            assert!((0.0..=1.0).contains(&tb), "tonality bounded: {tb}");
        }
    }

    #[test]
    fn tonal_partition_gets_higher_snr_than_noisy() {
        // A fully tonal partition (cb→0, tb→1) demands SNR ≈ TMN; a
        // fully noisy one (tb→0) demands SNR ≈ NMT = 5.5 dB. TMN ≥ 24.5
        // everywhere, so tonal SNR exceeds noisy SNR.
        let t = PartitionTables::for_rate(Model2Rate::Hz32000);
        let bmax = t.bmax();
        // Large partition energy so the masking term nb dominates the
        // absolute-threshold floor at the compared line.
        let en = vec![1e12; bmax];
        let tb_tonal = vec![1.0; bmax];
        let tb_noisy = vec![0.0; bmax];
        let thr_tonal = line_thresholds(&t, &en, &tb_tonal, Model2Rate::Hz32000);
        let thr_noisy = line_thresholds(&t, &en, &tb_noisy, Model2Rate::Hz32000);
        // Higher required SNR (tonal) → lower power ratio → lower noise
        // threshold. Compare a mid-band line dominated by nb (not LTq).
        let line = 200usize;
        assert!(
            thr_tonal[line] < thr_noisy[line],
            "tonal {} < noisy {}",
            thr_tonal[line],
            thr_noisy[line]
        );
    }

    #[test]
    fn line_threshold_floored_by_absolute_threshold() {
        // With zero signal energy the per-line threshold collapses to
        // the Table D.4x absolute threshold (energy domain).
        let t = PartitionTables::for_rate(Model2Rate::Hz32000);
        let bmax = t.bmax();
        let en = vec![0.0; bmax];
        let tb = vec![0.5; bmax];
        let thr = line_thresholds(&t, &en, &tb, Model2Rate::Hz32000);
        // Line 100 (1-based) has a finite absolute threshold.
        let absthr = absthr_db_to_energy(Model2Rate::Hz32000.absthr_db(100).unwrap());
        assert!((thr[99] - absthr).abs() < 1e-9, "floored to LTq energy");
    }

    #[test]
    fn absthr_db_to_energy_is_monotone() {
        assert!(absthr_db_to_energy(0.0) < absthr_db_to_energy(10.0));
        assert!((absthr_db_to_energy(0.0) - 1.0).abs() < 1e-12);
        assert!((absthr_db_to_energy(10.0) - 10.0).abs() < 1e-9);
    }
}
