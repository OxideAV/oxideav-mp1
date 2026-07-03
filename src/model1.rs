//! ISO/IEC 11172-3 (1993) Annex D **Psychoacoustic Model 1** per-frame
//! driver (clause D.1).
//!
//! [`crate::psy`] stages every *static* Model 1 ingredient — the
//! threshold-in-quiet tables (D.1a–f), the critical-band boundary
//! tables (D.2a–f), the Step 6 masking-index / masking-function closed
//! forms ([`crate::psy::masking_index_tonal`],
//! [`crate::psy::masking_function`]) and the Step 7 power-domain
//! combiner ([`crate::psy::global_threshold_db`]). This module
//! assembles those ingredients into the **nine-step per-frame
//! procedure** of clause D.1, producing the per-subband
//! signal-to-mask ratios `SMR_sb(n)` the §C.1.5.1.6 / §C.1.5.2.7 bit
//! allocator needs:
//!
//! - **Step 1** — Hann-windowed FFT (512-point for Layer I, 1024-point
//!   for Layer II) to the power density spectrum `X(k)`, normalized to
//!   the 96 dB SPL reference ([`power_spectrum`]).
//! - **Step 2** — sound pressure level `Lsb(n)` per subband from the
//!   spectral maximum and the scalefactor term ([`spl_per_subband`]).
//! - **Step 3** — threshold in quiet `LTq(i)` from Tables D.1a–f with
//!   the per-channel bit-rate offset (already staged in `psy`).
//! - **Step 4** — tonal / non-tonal component identification.
//! - **Step 5** — decimation of irrelevant maskers.
//! - **Steps 6/7** — individual and global masking thresholds on the
//!   subsampled frequency lines of Tables D.1a–f.
//! - **Step 8** — minimum masking threshold `LTmin(n)` per subband.
//! - **Step 9** — `SMR_sb(n) = Lsb(n) − LTmin(n)`.
//!
//! Provenance: every formula here is the closed form printed in
//! ISO/IEC 11172-3 (1993) Annex D clause D.1 (PDF pages 115–121,
//! printed 109–115 of the in-repo
//! `docs/audio/mp3/ISO_IEC_11172-3-MP3-1993.pdf`), read from page
//! renders, plus the verbatim Step 3/6/7 lines already staged in
//! `docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md`. The model
//! is **informative** in 11172-3 — an example encoder model, not a
//! normative decoder requirement — so there is no bit-exact oracle;
//! the tests assert the documented analytic properties of each step.

use crate::header::Layer;
use crate::model2::fft_in_place;

/// Number of Layer I / II subbands.
pub const SUBBANDS: usize = 32;

/// The Model 1 analysis-FFT length for `layer` (clause D.1 Step 1):
/// 512 samples for Layer I, 1 024 for Layer II.
pub fn fft_size(layer: Layer) -> usize {
    match layer {
        Layer::I => 512,
        Layer::II => 1024,
    }
}

/// Number of usable FFT lines (`k = 0…N/2`) for `layer`: 257 for
/// Layer I, 513 for Layer II.
pub fn num_lines(layer: Layer) -> usize {
    fft_size(layer) / 2 + 1
}

/// Clause D.1 Step 1 — the power-spectrum calibration offset in dB.
///
/// The spec requires "a normalization to the reference level of 96 dB
/// SPL … in such a way that the maximum value corresponds to 96 dB".
/// This crate reads that as a **fixed full-scale calibration**: a
/// full-scale sinusoid (amplitude 1,0 in this crate's PCM convention,
/// i.e. ±32 768 in the S16 domain) produces a 96 dB spectral peak.
/// Two spec-internal anchors corroborate the fixed reading over a
/// per-frame `max → 96 dB` gain: the Step 2 scalefactor term
/// `20·log10(scf_max·32768) − 10` compares against `X(k)` on an
/// *absolute* scale, and the Model 2 Table D.4 note calibrates its
/// 0 dB "96 dB below the energy of a sine wave of amplitude
/// ±32 760" — both are meaningless if `X(k)` were re-gained per
/// frame.
///
/// Derivation: a bin-centred sinusoid of amplitude `A` through the
/// Step 1 Hann window `h(i) = √(8/3)·0,5·(1 − cos(2πi/N))` yields
/// `|X| = A · (1/N)·Σh · ½ = A·√(8/3)/4`, i.e. per-line power
/// `A²/6`. The offset is therefore `96 − 10·log10(1/6) ≈ 103,78 dB`
/// so that `A = 1` peaks at exactly 96 dB.
pub fn calibration_offset_db() -> f64 {
    96.0 - 10.0 * (1.0f64 / 6.0).log10()
}

/// Numerical squelch floor for the Step 1 spectrum, in dB.
///
/// In exact arithmetic a bin-centred sinusoid excites exactly three
/// FFT lines through the Hann window and every other line is zero
/// (`−∞` dB). Double-precision FFT round-off instead leaves crumbs
/// around −190 dB and below, which would otherwise register as
/// spurious Step 4 "local maxima" (they beat their neighbours by
/// far more than 7 dB). Lines below this floor are therefore reported
/// as `−∞`, restoring the exact-arithmetic picture. The floor is
/// semantically inert: it sits 30 dB below 16-bit quantization
/// silence (1 LSB ≈ −90 dB against the 96 dB reference) and more than
/// 100 dB below the lowest Annex D threshold-in-quiet value
/// (−4,97 dB pre-offset), so no representable signal or threshold
/// interaction is affected.
pub const SPECTRUM_SQUELCH_DB: f64 = -120.0;

/// Clause D.1 Step 1 — Hann-windowed power density spectrum `X(k)` in
/// dB, `k = 0…N/2`, normalized to the 96 dB SPL reference.
///
/// `samples` is one analysis window of PCM in `[-1, 1)`, exactly
/// [`fft_size`]`(layer)` long (512 for Layer I, 1 024 for Layer II);
/// returns `None` on a length mismatch. The window is the printed
/// `h(i) = √(8/3) · 0,5 · (1 − cos(2π·i/N))`, `0 <= i <= N−1`, and the
/// spectrum the printed
/// `X(k) = 10·log10 |(1/N)·Σ h(l)·s(l)·e^(−j·2πkl/N)|²` plus the
/// [`calibration_offset_db`] normalization. A zero line comes back as
/// `−∞` dB (silence has no spectral content, matching the Step 4
/// "set to −∞" convention).
pub fn power_spectrum(samples: &[f64], layer: Layer) -> Option<Vec<f64>> {
    let n = fft_size(layer);
    if samples.len() != n {
        return None;
    }
    let inv_n = 1.0 / n as f64;
    let norm = (8.0f64 / 3.0).sqrt();
    // Interleaved [re, im] buffer; the window and the spec's 1/N are
    // folded into the real part before the transform.
    let mut buf = vec![0.0f64; 2 * n];
    for (i, &s) in samples.iter().enumerate() {
        let h = norm * 0.5 * (1.0 - (2.0 * core::f64::consts::PI * i as f64 * inv_n).cos());
        buf[2 * i] = h * s * inv_n;
    }
    fft_in_place(&mut buf);
    let offset = calibration_offset_db();
    let mut x = Vec::with_capacity(n / 2 + 1);
    for k in 0..=n / 2 {
        let (re, im) = (buf[2 * k], buf[2 * k + 1]);
        let power = re * re + im * im;
        // log10(0) is -inf in IEEE-754; -inf + offset stays -inf, so a
        // silent line needs no special casing. Sub-squelch numerical
        // crumbs are reported as -inf too (see SPECTRUM_SQUELCH_DB).
        let v = 10.0 * power.log10() + offset;
        x.push(if v < SPECTRUM_SQUELCH_DB {
            f64::NEG_INFINITY
        } else {
            v
        });
    }
    Some(x)
}

/// Number of FFT lines per subband for `layer`: the `N/2` usable lines
/// tile the 32 subbands with `N/64` lines each (8 for Layer I, 16 for
/// Layer II). The Nyquist line `k = N/2` sits on the upper edge of
/// subband 31 and belongs to no subband.
pub fn lines_per_subband(layer: Layer) -> usize {
    fft_size(layer) / 64
}

/// Clause D.1 Step 2 — sound pressure level `Lsb(n)` per subband, in
/// dB:
///
/// ```text
/// Lsb(n) = MAX[ X(k), 20·log10(scf_max(n)·32768) − 10 ]  dB
///          X(k) in subband n
/// ```
///
/// `x_db` is the Step 1 spectrum ([`power_spectrum`]); `scf_max[n]` is
/// the subband's scalefactor **multiplier** (Table 3-B.1 value — the
/// Layer I scalefactor, or in Layer II the maximum of the three
/// scalefactors of the frame). The `−10 dB` term is the spec's
/// peak-to-RMS correction. Returns `None` when `x_db` does not have
/// the [`num_lines`]`(layer)` length.
pub fn spl_per_subband(
    x_db: &[f64],
    scf_max: &[f64; SUBBANDS],
    layer: Layer,
) -> Option<[f64; SUBBANDS]> {
    spl_per_subband_impl(x_db, scf_max, layer, false)
}

/// Clause D.1 Step 2 — the spec's **alternative** `Lsb(n)`, replacing
/// the per-subband spectral maximum with the power sum
/// `X_spl(n) = 10·log10( Σ 10^(X(k)/10) )` over the subband's lines.
///
/// The spec offers this variant as "a potential for better encoder
/// performance" (it counts all of a subband's spectral energy, not
/// just its loudest line) while noting it has not been subjected to a
/// formal audio quality test. Same contract as [`spl_per_subband`].
pub fn spl_per_subband_alt(
    x_db: &[f64],
    scf_max: &[f64; SUBBANDS],
    layer: Layer,
) -> Option<[f64; SUBBANDS]> {
    spl_per_subband_impl(x_db, scf_max, layer, true)
}

fn spl_per_subband_impl(
    x_db: &[f64],
    scf_max: &[f64; SUBBANDS],
    layer: Layer,
    power_sum: bool,
) -> Option<[f64; SUBBANDS]> {
    if x_db.len() != num_lines(layer) {
        return None;
    }
    let lines = lines_per_subband(layer);
    let mut out = [f64::NEG_INFINITY; SUBBANDS];
    for (sb, o) in out.iter_mut().enumerate() {
        let band = &x_db[sb * lines..(sb + 1) * lines];
        let x_term = if power_sum {
            let acc: f64 = band.iter().map(|&v| 10f64.powf(v / 10.0)).sum();
            10.0 * acc.log10()
        } else {
            band.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        };
        // 20·log10(0) is -inf: an all-zero scalefactor floor simply
        // never wins the MAX.
        let scf_term = 20.0 * (scf_max[sb] * 32768.0).log10() - 10.0;
        *o = x_term.max(scf_term);
    }
    Some(out)
}

// -----------------------------------------------------------------
// Clause D.1 Step 4 — Finding of tonal and non-tonal components
// -----------------------------------------------------------------

/// A Step 4 **tonal** (sinusoid-like) masking component: a local
/// spectral maximum that beats every examined neighbour by at least
/// 7 dB, carrying the three-line power sum
/// `X_tm(k) = 10·log10(10^(X(k−1)/10) + 10^(X(k)/10) + 10^(X(k+1)/10))`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TonalComponent {
    /// FFT-line index `k` of the local maximum (0-based DC..Nyquist,
    /// matching [`power_spectrum`]'s indexing).
    pub k: usize,
    /// Sound pressure level `X_tm(k)` in dB.
    pub spl_db: f64,
}

/// A Step 4 **non-tonal** (noise-like) masking component: the power
/// sum of one critical band's remaining spectral lines, placed at the
/// line nearest the band's geometric mean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NonTonalComponent {
    /// Representative FFT-line index `k` — the line nearest the
    /// geometric mean of the critical band.
    pub k: usize,
    /// Sound pressure level `X_nm(k)` in dB (`−∞` for a band whose
    /// every line was consumed by tonal extraction or silent).
    pub spl_db: f64,
    /// 0-based critical-band number (row of the matching Table D.2x).
    pub band: usize,
}

// The examined-neighbour offset sets `j` of clause D.1 Step 4 b). The
// spec lists them per layer and per FFT-line region; ±1 is never
// listed because the local-maximum labelling (Step 4 a) already
// covers the immediate neighbours.
const J2: &[isize] = &[-2, 2];
const J3: &[isize] = &[-3, -2, 2, 3];
const J6: &[isize] = &[-6, -5, -4, -3, -2, 2, 3, 4, 5, 6];
const J12: &[isize] = &[
    -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
];

/// Clause D.1 Step 4 b) — the examined-neighbour offsets `j` for a
/// local maximum at FFT line `k`, or `None` when `k` lies outside the
/// examinable region (no line there can be labelled tonal):
///
/// ```text
/// Layer I:  j = −2, +2            for   2 < k < 63
///           j = −3…−2, +2…+3      for  63 <= k < 127
///           j = −6…−2, +2…+6      for 127 <= k <= 250
/// Layer II: (same first three regions, region 3 ending at k < 255)
///           j = −12…−2, +2…+12    for 255 <= k <= 500
/// ```
pub fn tonal_search_offsets(layer: Layer, k: usize) -> Option<&'static [isize]> {
    match layer {
        Layer::I => match k {
            3..=62 => Some(J2),
            63..=126 => Some(J3),
            127..=250 => Some(J6),
            _ => None,
        },
        Layer::II => match k {
            3..=62 => Some(J2),
            63..=126 => Some(J3),
            127..=254 => Some(J6),
            255..=500 => Some(J12),
            _ => None,
        },
    }
}

/// Clause D.1 Step 4 a)+b) — label the local maxima of `x_db` and
/// extract the **tonal** components.
///
/// Returns the tonal list plus the *residual* spectrum the Step 4 c)
/// non-tonal accumulation consumes: for every extracted tonal
/// component, all lines within the examined frequency range
/// (`k ± max|j|`, which covers the three lines summed into `X_tm`)
/// are set to `−∞` dB, per the spec's "all spectral lines within the
/// examined frequency range are set to −∞ dB". Local-maximum
/// labelling and the 7 dB criterion are evaluated on the *input*
/// spectrum (the spec presents Step 4 as list operations over the
/// computed spectrum; the close-tonal interactions this leaves are
/// resolved by the Step 5 b) 0,5-Bark decimation). Returns `None` on
/// a spectrum-length mismatch.
pub fn find_tonal_components(
    x_db: &[f64],
    layer: Layer,
) -> Option<(Vec<TonalComponent>, Vec<f64>)> {
    let len = num_lines(layer);
    if x_db.len() != len {
        return None;
    }
    let mut residual = x_db.to_vec();
    let mut tonal = Vec::new();
    for k in 1..len - 1 {
        // Step 4 a): X(k) > X(k−1) and X(k) >= X(k+1).
        if !(x_db[k] > x_db[k - 1] && x_db[k] >= x_db[k + 1]) {
            continue;
        }
        let Some(offsets) = tonal_search_offsets(layer, k) else {
            continue;
        };
        // Step 4 b): X(k) − X(k+j) >= 7 dB for every examined j.
        let is_tonal = offsets.iter().all(|&j| {
            let idx = k as isize + j;
            match usize::try_from(idx).ok().and_then(|i| x_db.get(i)) {
                Some(&neighbor) => x_db[k] - neighbor >= 7.0,
                // Beyond the spectrum edge there is no neighbour to
                // out-mask; treat as satisfied.
                None => true,
            }
        });
        if !is_tonal {
            continue;
        }
        // X_tm(k): power sum of the maximum and its two neighbours.
        let p = 10f64.powf(x_db[k - 1] / 10.0)
            + 10f64.powf(x_db[k] / 10.0)
            + 10f64.powf(x_db[k + 1] / 10.0);
        tonal.push(TonalComponent {
            k,
            spl_db: 10.0 * p.log10(),
        });
        // Zero the examined frequency range in the residual.
        let jmax = offsets.iter().map(|j| j.unsigned_abs()).max().unwrap_or(2);
        let lo = k.saturating_sub(jmax);
        let hi = (k + jmax).min(len - 1);
        for r in &mut residual[lo..=hi] {
            *r = f64::NEG_INFINITY;
        }
    }
    Some((tonal, residual))
}

/// Clause D.1 Step 4 c) — accumulate the **non-tonal** components
/// from the residual spectrum ([`find_tonal_components`]'s second
/// return): within each critical band of the matching Table D.2x, the
/// powers of the remaining spectral lines are summed to one non-tonal
/// component `X_nm(k)`, listed at the index `k` of the spectral line
/// nearest to the geometric mean of the critical band.
///
/// Lines above the last critical-band boundary (beyond the Table D.1x
/// / D.2x coverage, e.g. above 15 kHz at 32 kHz Layer I) belong to no
/// band and are ignored — the Annex D tables define no threshold
/// there. Returns `None` for a `(layer, sampling_frequency)` pair
/// without a Table D.2x (the MPEG-2 LSF rates) or on a length
/// mismatch.
pub fn find_non_tonal_components(
    residual: &[f64],
    layer: Layer,
    sampling_frequency_hz: u32,
) -> Option<Vec<NonTonalComponent>> {
    let len = num_lines(layer);
    if residual.len() != len {
        return None;
    }
    let bands = crate::psy::critical_band_table(layer, sampling_frequency_hz)?;
    let n = fft_size(layer) as f64;
    let fs = sampling_frequency_hz as f64;
    let mut out = Vec::with_capacity(bands.len());
    let mut prev_top_hz = 0.0f64;
    for (b, band) in bands.iter().enumerate() {
        // Lines with prev_top < k·Fs/N <= top. The D.2x tops sit
        // exactly on the line grid but are printed truncated to three
        // decimals (e.g. 258,398 Hz for the exact 3·Fs/512 =
        // 258,3984375 Hz at 44,1 kHz), so the boundary can land up to
        // ~1e-5 line units *below* its integer; the +1e-3 epsilon
        // absorbs that print truncation (the next line is a full 1,0
        // away, so it can never overshoot).
        let k_lo = ((prev_top_hz * n / fs) + 1e-3).floor() as usize + 1;
        let k_hi = (((band.top_freq_hz * n / fs) + 1e-3).floor() as usize).min(len - 1);
        prev_top_hz = band.top_freq_hz;
        if k_lo > k_hi {
            continue;
        }
        let power: f64 = residual[k_lo..=k_hi]
            .iter()
            .map(|&v| 10f64.powf(v / 10.0))
            .sum();
        // Geometric mean of the band, expressed on the line grid
        // (frequency is proportional to line index).
        let k_gm = ((k_lo as f64 * k_hi as f64).sqrt().round() as usize).clamp(k_lo, k_hi);
        out.push(NonTonalComponent {
            k: k_gm,
            spl_db: 10.0 * power.log10(),
            band: b,
        });
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Full-scale bin-centred sine at 1-based FFT line `k0`.
    fn sine(n: usize, k0: usize, amplitude: f64) -> Vec<f64> {
        (0..n)
            .map(|i| {
                amplitude * (2.0 * core::f64::consts::PI * k0 as f64 * i as f64 / n as f64).sin()
            })
            .collect()
    }

    #[test]
    fn fft_sizes_match_step1_technical_data() {
        // Clause D.1 Step 1: 512-point FFT for Layer I, 1024-point for
        // Layer II; frequency resolution Fs/512 and Fs/1024.
        assert_eq!(fft_size(Layer::I), 512);
        assert_eq!(fft_size(Layer::II), 1024);
        assert_eq!(num_lines(Layer::I), 257);
        assert_eq!(num_lines(Layer::II), 513);
        assert_eq!(lines_per_subband(Layer::I), 8);
        assert_eq!(lines_per_subband(Layer::II), 16);
    }

    #[test]
    fn power_spectrum_rejects_wrong_length() {
        assert!(power_spectrum(&[0.0; 511], Layer::I).is_none());
        assert!(power_spectrum(&[0.0; 1024], Layer::I).is_none());
        assert!(power_spectrum(&[0.0; 512], Layer::II).is_none());
    }

    #[test]
    fn full_scale_sine_peaks_at_96_db() {
        for (layer, k0) in [(Layer::I, 40usize), (Layer::II, 100usize)] {
            let s = sine(fft_size(layer), k0, 1.0);
            let x = power_spectrum(&s, layer).unwrap();
            // The 96 dB SPL normalization: full scale == 96 dB at the
            // sine's own line. The negative-frequency image leaks only
            // through the Hann sidelobes at 2·k0 lines distance.
            assert!(
                (x[k0] - 96.0).abs() < 1e-3,
                "layer {layer:?}: X({k0}) = {} dB, want 96",
                x[k0]
            );
            // And that line is the global maximum.
            let argmax = x
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                .unwrap()
                .0;
            assert_eq!(argmax, k0);
        }
    }

    #[test]
    fn half_amplitude_sine_is_6_db_down() {
        let k0 = 64;
        let s = sine(512, k0, 0.5);
        let x = power_spectrum(&s, Layer::I).unwrap();
        assert!(
            (x[k0] - (96.0 - 20.0 * 2.0f64.log10())).abs() < 1e-3,
            "X = {}",
            x[k0]
        );
    }

    #[test]
    fn silence_is_neg_infinity_everywhere() {
        let x = power_spectrum(&[0.0; 512], Layer::I).unwrap();
        assert!(x.iter().all(|&v| v == f64::NEG_INFINITY));
    }

    #[test]
    fn hann_window_suppresses_far_leakage() {
        // A bin-centred sine's energy is confined to its own line and
        // the two adjacent Hann-spread lines; four lines away the level
        // is far below the peak.
        let k0 = 100;
        let s = sine(512, k0, 1.0);
        let x = power_spectrum(&s, Layer::I).unwrap();
        assert!(x[k0 + 4] < x[k0] - 60.0, "leak {} vs {}", x[k0 + 4], x[k0]);
        // The adjacent lines carry the Hann skirt (−6 dB each for a
        // perfectly centred bin).
        assert!((x[k0 + 1] - (96.0 - 6.0206)).abs() < 0.1);
        assert!((x[k0 - 1] - (96.0 - 6.0206)).abs() < 0.1);
    }

    #[test]
    fn calibration_offset_matches_closed_form() {
        // 96 − 10·log10(1/6) = 96 + 10·log10 6.
        assert!((calibration_offset_db() - (96.0 + 10.0 * 6.0f64.log10())).abs() < 1e-12);
    }

    #[test]
    fn spl_rejects_wrong_spectrum_length() {
        let scf = [1.0; SUBBANDS];
        assert!(spl_per_subband(&vec![0.0; 256], &scf, Layer::I).is_none());
        assert!(spl_per_subband(&vec![0.0; 257], &scf, Layer::II).is_none());
    }

    #[test]
    fn spl_picks_up_tone_in_its_own_subband() {
        // Line 44 (Layer I) sits in subband 5 (lines 40..48).
        let k0 = 44;
        let s = sine(512, k0, 1.0);
        let x = power_spectrum(&s, Layer::I).unwrap();
        let scf = [0.0; SUBBANDS];
        let l = spl_per_subband(&x, &scf, Layer::I).unwrap();
        assert!((l[5] - 96.0).abs() < 1e-3, "Lsb(5) = {}", l[5]);
        // Far subbands see only the FFT's numerical noise floor and a
        // zero scalefactor floor: far, far below audibility.
        assert!(l[20] < -180.0, "Lsb(20) = {}", l[20]);
    }

    #[test]
    fn spl_scalefactor_term_floors_silence() {
        // Silence: Lsb(n) = 20·log10(scf·32768) − 10 exactly.
        let x = power_spectrum(&[0.0; 512], Layer::I).unwrap();
        let mut scf = [0.0; SUBBANDS];
        scf[3] = 2.0;
        scf[7] = 0.0157;
        let l = spl_per_subband(&x, &scf, Layer::I).unwrap();
        for (sb, &s) in scf.iter().enumerate() {
            if s > 0.0 {
                let want = 20.0 * (s * 32768.0).log10() - 10.0;
                assert!((l[sb] - want).abs() < 1e-12, "sb {sb}");
            } else {
                assert_eq!(l[sb], f64::NEG_INFINITY);
            }
        }
    }

    #[test]
    fn spl_takes_max_of_spectrum_and_scalefactor_term() {
        // A quiet tone (well below the scalefactor term) loses the MAX.
        let k0 = 44;
        let s = sine(512, k0, 1e-4); // 96 − 80 = 16 dB peak
        let x = power_spectrum(&s, Layer::I).unwrap();
        let mut scf = [0.0; SUBBANDS];
        scf[5] = 2.0; // 86.3 dB floor
        let l = spl_per_subband(&x, &scf, Layer::I).unwrap();
        let want = 20.0 * (2.0 * 32768.0f64).log10() - 10.0;
        assert!((l[5] - want).abs() < 1e-9);
        // A loud tone wins it.
        let s = sine(512, k0, 1.0);
        let x = power_spectrum(&s, Layer::I).unwrap();
        let l = spl_per_subband(&x, &scf, Layer::I).unwrap();
        assert!((l[5] - 96.0).abs() < 1e-3);
    }

    #[test]
    fn spl_alt_power_sum_dominates_max_variant() {
        // The alternative Xspl(n) sums the subband's line powers, so it
        // is >= the per-line maximum everywhere (equality only for a
        // one-line subband).
        let mut s = sine(512, 44, 0.5);
        let s2 = sine(512, 46, 0.5);
        for (a, b) in s.iter_mut().zip(s2.iter()) {
            *a += *b;
        }
        let x = power_spectrum(&s, Layer::I).unwrap();
        let scf = [0.0; SUBBANDS];
        let lmax = spl_per_subband(&x, &scf, Layer::I).unwrap();
        let lsum = spl_per_subband_alt(&x, &scf, Layer::I).unwrap();
        for sb in 0..SUBBANDS {
            if lmax[sb].is_finite() {
                assert!(
                    lsum[sb] >= lmax[sb] - 1e-12,
                    "sb {sb}: {} < {}",
                    lsum[sb],
                    lmax[sb]
                );
            }
        }
        // Two equal tones in subband 5: the power sum sits ≈ 3 dB above
        // the single-line max (plus Hann-skirt energy).
        assert!(lsum[5] > lmax[5] + 2.5);
    }

    #[test]
    fn layer2_spectrum_has_finer_grid() {
        // The same physical frequency lands on twice the line index in
        // the Layer II 1024-point analysis.
        let k1 = 40;
        let s1 = sine(512, k1, 1.0);
        let s2 = sine(1024, 2 * k1, 1.0);
        let x1 = power_spectrum(&s1, Layer::I).unwrap();
        let x2 = power_spectrum(&s2, Layer::II).unwrap();
        assert!((x1[k1] - x2[2 * k1]).abs() < 1e-6);
    }

    // ------------------------- Step 4 -------------------------

    #[test]
    fn tonal_offsets_follow_step4_regions() {
        // Region boundaries, Layer I: 2 < k < 63 / 63..127 / 127..=250.
        assert_eq!(tonal_search_offsets(Layer::I, 2), None);
        assert_eq!(tonal_search_offsets(Layer::I, 3), Some(J2));
        assert_eq!(tonal_search_offsets(Layer::I, 62), Some(J2));
        assert_eq!(tonal_search_offsets(Layer::I, 63), Some(J3));
        assert_eq!(tonal_search_offsets(Layer::I, 126), Some(J3));
        assert_eq!(tonal_search_offsets(Layer::I, 127), Some(J6));
        assert_eq!(tonal_search_offsets(Layer::I, 250), Some(J6));
        assert_eq!(tonal_search_offsets(Layer::I, 251), None);
        // Layer II adds the fourth region 255..=500 with j to ±12.
        assert_eq!(tonal_search_offsets(Layer::II, 127), Some(J6));
        assert_eq!(tonal_search_offsets(Layer::II, 254), Some(J6));
        assert_eq!(tonal_search_offsets(Layer::II, 255), Some(J12));
        assert_eq!(tonal_search_offsets(Layer::II, 500), Some(J12));
        assert_eq!(tonal_search_offsets(Layer::II, 501), None);
        // ±1 is never an examined offset; every set is symmetric.
        for set in [J2, J3, J6, J12] {
            assert!(!set.contains(&1) && !set.contains(&-1));
            assert!(set.iter().all(|&j| set.contains(&-j)));
        }
    }

    #[test]
    fn single_sine_yields_one_tonal_component() {
        let k0 = 44;
        let s = sine(512, k0, 1.0);
        let x = power_spectrum(&s, Layer::I).unwrap();
        let (tonal, residual) = find_tonal_components(&x, Layer::I).unwrap();
        assert_eq!(tonal.len(), 1, "{tonal:?}");
        assert_eq!(tonal[0].k, k0);
        // X_tm = the 96 dB peak plus its two −6,02 dB Hann skirts:
        // 96 + 10·log10(1 + 2·0,25) ≈ 97,76 dB.
        assert!(
            (tonal[0].spl_db - (96.0 + 10.0 * 1.5f64.log10())).abs() < 0.05,
            "X_tm = {}",
            tonal[0].spl_db
        );
        // The examined range (±2 in this region) is erased from the
        // residual; lines beyond it are untouched (here already −∞:
        // a bin-centred sine excites exactly three Hann lines and the
        // squelch restores the exact-arithmetic zeros elsewhere).
        for (k, &r) in residual.iter().enumerate().skip(k0 - 2).take(5) {
            assert_eq!(r, f64::NEG_INFINITY, "line {k}");
        }
        assert_eq!(residual[k0 + 3], x[k0 + 3]);
        assert_eq!(x[k0 + 3], f64::NEG_INFINITY);
    }

    #[test]
    fn two_distant_sines_yield_two_tonal_components() {
        let (ka, kb) = (44, 200);
        let mut s = sine(512, ka, 0.5);
        for (a, b) in s.iter_mut().zip(sine(512, kb, 0.25)) {
            *a += b;
        }
        let x = power_spectrum(&s, Layer::I).unwrap();
        let (tonal, _) = find_tonal_components(&x, Layer::I).unwrap();
        let lines: Vec<usize> = tonal.iter().map(|t| t.k).collect();
        assert_eq!(lines, vec![ka, kb]);
    }

    #[test]
    fn adjacent_equal_partials_are_not_tonal() {
        // Two equal-amplitude sines two lines apart: each local maximum
        // fails the 7 dB criterion against the other (X(k) − X(k±2) ≈ 0).
        let k0 = 100;
        let mut s = sine(512, k0, 0.5);
        for (a, b) in s.iter_mut().zip(sine(512, k0 + 2, 0.5)) {
            *a += b;
        }
        let x = power_spectrum(&s, Layer::I).unwrap();
        let (tonal, _) = find_tonal_components(&x, Layer::I).unwrap();
        assert!(
            !tonal
                .iter()
                .any(|t| (t.k as isize - k0 as isize).abs() <= 2),
            "{tonal:?}"
        );
    }

    #[test]
    fn tonal_rejects_wrong_length() {
        assert!(find_tonal_components(&[0.0; 256], Layer::I).is_none());
        assert!(find_tonal_components(&[0.0; 257], Layer::II).is_none());
    }

    #[test]
    fn non_tonal_components_cover_every_band_once() {
        // Property check at every supported (layer, rate): the per-band
        // line ranges tile lines 1..=top contiguously and each geometric
        // mean lands inside its own band.
        for layer in [Layer::I, Layer::II] {
            for fs in [32_000u32, 44_100, 48_000] {
                let x = vec![20.0; num_lines(layer)];
                let nt = find_non_tonal_components(&x, layer, fs).unwrap();
                let bands = crate::psy::critical_band_table(layer, fs).unwrap();
                assert_eq!(nt.len(), bands.len(), "{layer:?} {fs}");
                for (b, c) in nt.iter().enumerate() {
                    assert_eq!(c.band, b);
                    assert!(c.spl_db.is_finite());
                }
                // Representative lines are strictly increasing.
                for w in nt.windows(2) {
                    assert!(w[0].k < w[1].k);
                }
                // Reconstruct the total power: the band power sums must
                // add up to the power of all covered lines exactly.
                let n = fft_size(layer) as f64;
                let top_line =
                    ((bands.last().unwrap().top_freq_hz * n / fs as f64) + 1e-9).floor() as usize;
                let covered = top_line.min(num_lines(layer) - 1);
                let want: f64 = covered as f64 * 10f64.powf(2.0);
                let got: f64 = nt.iter().map(|c| 10f64.powf(c.spl_db / 10.0)).sum();
                assert!(
                    (got - want).abs() < 1e-6 * want,
                    "{layer:?} {fs}: {got} vs {want}"
                );
            }
        }
    }

    #[test]
    fn non_tonal_geometric_mean_placement() {
        // Layer I / 32 kHz band 8 spans lines 16..18 (Table D.2a: tops
        // at index 15 and 18); the geometric mean sqrt(16·18) ≈ 16,97
        // rounds to line 17.
        let x = vec![30.0; 257];
        let nt = find_non_tonal_components(&x, Layer::I, 32_000).unwrap();
        assert_eq!(nt[8].k, 17);
        // Band 0 is the single line 1.
        assert_eq!(nt[0].k, 1);
        // Its power is that one line's power.
        assert!((nt[0].spl_db - 30.0).abs() < 1e-9);
    }

    #[test]
    fn non_tonal_ignores_tonal_extracted_lines() {
        // A pure tone: after tonal extraction its band's remaining
        // power is only the far Hann-leakage crumbs, way below X_tm.
        let k0 = 44;
        let s = sine(512, k0, 1.0);
        let x = power_spectrum(&s, Layer::I).unwrap();
        let (tonal, residual) = find_tonal_components(&x, Layer::I).unwrap();
        let nt = find_non_tonal_components(&residual, Layer::I, 32_000).unwrap();
        // Line 44 at 32 kHz is 2750 Hz — band 14 of Table D.2a.
        let band14 = nt.iter().find(|c| c.band == 14).unwrap();
        assert!(
            band14.spl_db < tonal[0].spl_db - 60.0,
            "non-tonal {} vs tonal {}",
            band14.spl_db,
            tonal[0].spl_db
        );
    }

    #[test]
    fn non_tonal_rejects_lsf_rates_and_bad_lengths() {
        let x = vec![0.0; 257];
        assert!(find_non_tonal_components(&x, Layer::I, 24_000).is_none());
        assert!(find_non_tonal_components(&x, Layer::I, 22_050).is_none());
        assert!(find_non_tonal_components(&x, Layer::I, 16_000).is_none());
        assert!(find_non_tonal_components(&x, Layer::II, 32_000).is_none());
    }
}
