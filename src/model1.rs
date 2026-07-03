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
        // silent line needs no special casing.
        x.push(10.0 * power.log10() + offset);
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
}
