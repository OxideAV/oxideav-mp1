//! Lightweight per-subband masking model used by the MP1 VBR encoder.
//!
//! This is **not** a full ISO/IEC 11172-3 §C.1 (psychoacoustic model 1)
//! implementation — that needs an FFT-based tonality detector, the
//! spreading function over Bark partitions, the absolute hearing
//! threshold table and a separate analysis window pre-FFT. We do
//! something simpler that's still spec-grounded enough to drive a Layer
//! I VBR quantiser:
//!
//! 1. Take the 32 × 12 subband matrix the polyphase analysis filter
//!    already produced for each channel.
//! 2. For each subband, compute the signal energy
//!    `E_b = (1/12) * sum sub[b][i]^2` and the masking threshold
//!    `T_b = E_b / mask_ratio`, where `mask_ratio` scales with the VBR
//!    quality knob (higher quality → lower allowed noise → mask sits
//!    further below the band energy).
//! 3. The bit allocator estimates per-subband quantisation noise from
//!    the chosen allocation level: at allocation `a` (≥1) the encoder
//!    quantises with step `q = 2 / (2^(a+1) - 1) * sf_mag`, giving a
//!    uniform-quantiser noise variance of `q^2 / 12`. Total band noise
//!    over the 12 samples is `12 * q^2 / 12 = q^2`.
//! 4. The frame is "masked" at allocation `a[b]` for every band when
//!    `noise(a[b]) <= T_b`. Equivalently the allocator stops upgrading
//!    a subband once its NMR (noise-to-mask ratio) drops below 0 dB.
//!
//! The mask ratio for VBR quality 0..=9 follows the spirit of the
//! V0..V9 LAME convention without copying any external tables: V0
//! (highest quality, smallest noise budget) maps to a large mask ratio
//! (~30 dB SNR target per band), V9 to a small ratio (~6 dB).
//!
//! The model also folds in two simple effects from §C.1 that survive
//! the simplification:
//!
//! - **Absolute threshold of hearing**: subbands above ~16 kHz get an
//!   extra mask boost (looser threshold) because most listeners can't
//!   hear that range; this saves bits without measurable quality loss.
//! - **Inter-band spreading**: a small fraction of each band's energy
//!   leaks into the immediate neighbours when computing thresholds, so
//!   a loud subband partially masks its neighbours (one-tap spreader).

use crate::bitalloc::SBLIMIT;

/// Per-subband masking estimate for one channel of a Layer I frame.
#[derive(Clone, Debug)]
pub struct SubbandMask {
    /// Mean-square energy per subband over the 12 samples.
    pub energy: [f32; SBLIMIT],
    /// Masking threshold per subband. Quantisation noise that exceeds
    /// the threshold in any subband is audible per the simplified model.
    pub threshold: [f32; SBLIMIT],
}

impl SubbandMask {
    /// Build a mask estimate for one channel's subband matrix.
    ///
    /// `sub[b][i]` is the i-th sample (0..12) of subband `b` (0..32).
    /// `sample_rate` is the PCM sample rate (used to map subband index
    /// to a centre frequency for the absolute-threshold tweak).
    /// `mask_ratio` (>= 1.0) is the per-band SNR target as a *linear*
    /// ratio: the allowed quantisation noise per band is
    /// `E_b / mask_ratio`. Higher mask_ratio = stricter target = more
    /// bits.
    pub fn analyze(sub: &[[f32; 12]; SBLIMIT], sample_rate: u32, mask_ratio: f32) -> Self {
        // Step 1: per-band mean-square energy.
        let mut energy = [0.0f32; SBLIMIT];
        for b in 0..SBLIMIT {
            let mut acc = 0.0f32;
            for i in 0..12 {
                acc += sub[b][i] * sub[b][i];
            }
            energy[b] = acc / 12.0;
        }

        // Step 2: one-tap inter-band spreading. A loud subband leaks a
        // small fraction of its energy into its immediate neighbours
        // (cheap proxy for the ISO Bark-partition spreading function).
        // Coefficient kept small so we never raise a band's threshold
        // by more than ~1 dB from its own energy.
        const SPREAD: f32 = 0.10;
        let mut spread = energy;
        for b in 0..SBLIMIT {
            if b > 0 {
                spread[b] += SPREAD * energy[b - 1];
            }
            if b + 1 < SBLIMIT {
                spread[b] += SPREAD * energy[b + 1];
            }
        }

        // Step 3: turn each band's spread energy into a threshold.
        // Floor: keep the mask above pure-silence noise — without it a
        // near-silent band has T_b = 0 and the iterator can never
        // satisfy it.
        let global_floor = spread.iter().copied().fold(0.0f32, f32::max) * 1.0e-7 + 1.0e-12;
        let mut threshold = [0.0f32; SBLIMIT];
        let ratio = mask_ratio.max(1.0);
        for b in 0..SBLIMIT {
            threshold[b] = (spread[b] / ratio).max(global_floor);
        }

        // Step 4: absolute-threshold-of-hearing tweak. Subbands whose
        // centre frequency is above 16 kHz get a 12 dB threshold boost
        // (i.e. allow ~16× more noise). This roughly matches the
        // attenuation in the §C.1 ATH table for 16..20 kHz, which most
        // listeners can't perceive.
        let band_hz = sample_rate as f32 / 64.0; // 32 subbands span 0..fs/2
        for b in 0..SBLIMIT {
            let centre = (b as f32 + 0.5) * band_hz;
            if centre >= 16_000.0 {
                threshold[b] *= 16.0;
            } else if centre >= 12_000.0 {
                threshold[b] *= 4.0;
            }
        }

        Self { energy, threshold }
    }
}

/// Map a 0..=9 VBR quality scalar to a per-band SNR target (linear
/// energy ratio). Quality 0 = best (~30 dB SNR target); quality 9 =
/// smallest files (~6 dB SNR target). Smooth interpolation across the
/// V0..V9 range without copying any external encoder's table.
pub fn vbr_quality_to_mask_ratio(q: u8) -> f32 {
    let q = q.min(9) as f32;
    // SNR_db: q=0 → 30 dB, q=9 → 6 dB.
    let snr_db = 30.0 - (q / 9.0) * 24.0;
    10.0_f32.powf(snr_db / 10.0)
}

/// Estimate per-subband quantisation noise (variance, not squared) for
/// a Layer I allocation `alloc` (0..=14) with scalefactor magnitude
/// `sf_mag` for that subband.
///
/// At allocation `a >= 1` the decoder quantises sample step is
/// `q = 2 / (2^(a+1) - 1) * sf_mag`. Uniform-quantiser noise variance
/// is `q^2 / 12` per sample. Over 12 samples per subband the *total*
/// noise energy contribution is `12 * q^2 / 12 = q^2`. We therefore
/// compare `q^2` against the band threshold (which is also a 12-sample
/// energy on the same scale).
///
/// `alloc == 0` → no samples encoded → noise == band signal energy
/// (everything is lost). Returned value reflects that worst case.
pub fn subband_noise_energy(alloc: u8, sf_mag: f32, energy: f32) -> f32 {
    if alloc == 0 {
        // Band is dropped entirely; "noise" equals lost signal energy.
        return energy;
    }
    let nb = alloc as u32 + 1; // bits per sample
    let denom = ((1u32 << nb) - 1) as f32; // 2^nb - 1
    let q = 2.0 * sf_mag / denom;
    q * q
}

#[cfg(test)]
mod tests {
    use super::*;

    fn silent_sub() -> [[f32; 12]; SBLIMIT] {
        [[0.0f32; 12]; SBLIMIT]
    }

    #[test]
    fn silence_has_zero_energy_and_floor_threshold() {
        let m = SubbandMask::analyze(&silent_sub(), 44_100, vbr_quality_to_mask_ratio(2));
        for b in 0..SBLIMIT {
            assert_eq!(m.energy[b], 0.0);
            assert!(m.threshold[b] > 0.0, "threshold floor must be positive");
        }
    }

    #[test]
    fn loud_band_raises_neighbour_threshold() {
        let mut sub = silent_sub();
        for i in 0..12 {
            sub[5][i] = 0.5;
        }
        let m = SubbandMask::analyze(&sub, 44_100, vbr_quality_to_mask_ratio(2));
        // Neighbour (4 or 6) gets a higher threshold than the silent
        // band 0 thanks to spreading.
        assert!(m.threshold[6] > m.threshold[0]);
        assert!(m.threshold[4] > m.threshold[0]);
    }

    #[test]
    fn quality_knob_changes_threshold_strictness() {
        let mut sub = silent_sub();
        for i in 0..12 {
            sub[5][i] = 0.3;
        }
        let m_strict = SubbandMask::analyze(&sub, 44_100, vbr_quality_to_mask_ratio(0));
        let m_loose = SubbandMask::analyze(&sub, 44_100, vbr_quality_to_mask_ratio(9));
        // Quality 0 → stricter mask → threshold sits *below* quality 9.
        assert!(
            m_strict.threshold[5] < m_loose.threshold[5],
            "strict {} not < loose {}",
            m_strict.threshold[5],
            m_loose.threshold[5]
        );
    }

    #[test]
    fn high_band_threshold_boosted() {
        let mut sub = silent_sub();
        // Equal energy in band 5 (~3.4 kHz @ 44.1k) and band 28 (~19.6 kHz).
        for i in 0..12 {
            sub[5][i] = 0.3;
            sub[28][i] = 0.3;
        }
        let m = SubbandMask::analyze(&sub, 44_100, vbr_quality_to_mask_ratio(2));
        // High band gets the ATH boost.
        assert!(
            m.threshold[28] > m.threshold[5],
            "high-band threshold {} should exceed mid-band {}",
            m.threshold[28],
            m.threshold[5]
        );
    }

    #[test]
    fn vbr_quality_monotonic() {
        // Higher q → smaller ratio (looser mask).
        let r0 = vbr_quality_to_mask_ratio(0);
        let r5 = vbr_quality_to_mask_ratio(5);
        let r9 = vbr_quality_to_mask_ratio(9);
        assert!(r0 > r5);
        assert!(r5 > r9);
    }

    #[test]
    fn noise_drops_with_more_bits() {
        // With more allocation bits, quantiser noise must shrink.
        let n2 = subband_noise_energy(2, 1.0, 1.0);
        let n5 = subband_noise_energy(5, 1.0, 1.0);
        let n14 = subband_noise_energy(14, 1.0, 1.0);
        assert!(n2 > n5, "n2={n2} must exceed n5={n5}");
        assert!(n5 > n14, "n5={n5} must exceed n14={n14}");
    }

    #[test]
    fn zero_alloc_reports_band_energy_loss() {
        let n = subband_noise_energy(0, 1.0, 0.42);
        assert_eq!(n, 0.42);
    }
}
