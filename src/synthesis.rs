//! The §2.4.3.2 polyphase **synthesis subband filter** for MPEG-1
//! Audio Layer I.
//!
//! Implements the reconstruction operation of 3-Annex A Figure 3-A.2
//! "SYNTHESIS SUBBAND FILTER FLOW CHART" exactly as drawn in the
//! staged ISO/IEC 11172-3 (1993) PDF. For every set of 32 subband
//! samples (one per subband, for one channel and one sample-slot) the
//! filter emits 32 reconstructed PCM samples. A Layer I frame has 12
//! sample-slots per channel, so the per-frame output is
//! `12 * 32 = 384` PCM samples per channel.
//!
//! ## Algorithm (Figure 3-A.2)
//!
//! Per-channel state is a 1024-element FIFO `V` (initialised to zero
//! at startup — Figure 3-A.2 footnote 1). For each slot of 32 input
//! subband samples `S[0..32]`:
//!
//! 1. **Shifting.** `for i = 1023 down to 64: V[i] = V[i-64]`.
//! 2. **Matrixing.** `for i = 0..64: V[i] = Σ_{k=0}^{31} N_ik · S[k]`
//!    with `N_ik = cos[(16 + i)(2k + 1) · π / 64]` (the matrixing
//!    coefficients given in §2.4.3.2).
//! 3. **Build U (512 values).**
//!    `for i in 0..8: for j in 0..32:`
//!    `U[i*64 + j]      = V[i*128 + j]` and
//!    `U[i*64 + 32 + j] = V[i*128 + 96 + j]`.
//! 4. **Window.** `for i in 0..512: W[i] = U[i] · D[i]`, with `D`
//!    the [`crate::tables::SYNTHESIS_WINDOW`] coefficients of Table
//!    3-B.3.
//! 5. **Sum.** `for j in 0..32: out[j] = Σ_{i=0}^{15} W[j + 32i]`.
//!
//! The matrixing coefficients `N_ik` are computed here from the
//! closed-form cosine the standard gives; the window taps `D[i]`
//! come from the transcribed Table 3-B.3.

use crate::tables::SYNTHESIS_WINDOW;

/// Number of subbands fed into the filter per slot (§2.4.1.5).
const SUBBANDS: usize = 32;
/// Length of the per-channel FIFO `V` (Figure 3-A.2 "Shifting": the
/// loop runs `i = 1023 down to 64`, so `V` has 1024 elements).
const V_LEN: usize = 1024;
/// Length of the windowing vector `U`/`W` and Table 3-B.3 (512 taps).
const WINDOW_LEN: usize = 512;

/// Per-channel synthesis filterbank state: the 1024-element `V` FIFO
/// (Figure 3-A.2) plus the precomputed `N_ik` matrixing cosines.
///
/// The cosine table is shared by every channel and never mutated; the
/// `V` buffer is the only carry-over state, so [`reset`](Self::reset)
/// zeroing it returns the bank to its startup condition.
#[derive(Debug, Clone)]
pub struct SynthesisFilter {
    /// The 1024-element FIFO `V`, initialised to zero (Figure 3-A.2
    /// footnote 1: "V to be initialized with zeros during startup").
    v: Vec<f64>,
}

/// The §2.4.3.2 matrixing coefficients `N_ik = cos[(16+i)(2k+1)π/64]`
/// for `i in 0..64`, `k in 0..32`, flattened row-major as
/// `N[i*32 + k]`. Computed once, lazily, and shared by all filters.
fn matrix_coeffs() -> &'static [f64; 64 * SUBBANDS] {
    use std::sync::OnceLock;
    static N: OnceLock<[f64; 64 * SUBBANDS]> = OnceLock::new();
    N.get_or_init(|| {
        let mut n = [0.0f64; 64 * SUBBANDS];
        for i in 0..64 {
            for k in 0..SUBBANDS {
                // N_ik = cos[(16 + i)(2k + 1) * pi / 64].
                let angle = (16 + i) as f64 * (2 * k + 1) as f64 * std::f64::consts::PI / 64.0;
                n[i * SUBBANDS + k] = angle.cos();
            }
        }
        n
    })
}

impl SynthesisFilter {
    /// A fresh filterbank with its `V` FIFO zeroed (startup state,
    /// Figure 3-A.2 footnote 1).
    pub fn new() -> SynthesisFilter {
        SynthesisFilter {
            v: vec![0.0; V_LEN],
        }
    }

    /// Drop all carry-over state: zero the `V` FIFO so the next
    /// [`synthesize`](Self::synthesize) starts as if from a fresh
    /// stream (used after a container seek).
    pub fn reset(&mut self) {
        for x in &mut self.v {
            *x = 0.0;
        }
    }

    /// Run one slot of the synthesis filter: consume 32 subband
    /// samples `samples[0..32]` (one per subband for this channel and
    /// slot) and produce 32 reconstructed PCM samples (Figure 3-A.2).
    ///
    /// The returned values are the linear, unclamped reconstruction
    /// `out[j]` from the final summation; callers convert them to an
    /// integer PCM representation (see [`to_s16`]).
    pub fn synthesize(&mut self, samples: &[f64; SUBBANDS]) -> [f64; SUBBANDS] {
        // 1. Shifting: V[i] = V[i-64] for i = 1023 down to 64.
        // Copying high-to-low (largest index first) preserves the
        // values being read, matching the spec's descending loop.
        for i in (64..V_LEN).rev() {
            self.v[i] = self.v[i - 64];
        }

        // 2. Matrixing: V[i] = sum_{k=0..31} N_ik * S[k], for i in 0..64.
        let n = matrix_coeffs();
        for i in 0..64 {
            let row = &n[i * SUBBANDS..i * SUBBANDS + SUBBANDS];
            let mut acc = 0.0f64;
            for k in 0..SUBBANDS {
                acc += row[k] * samples[k];
            }
            self.v[i] = acc;
        }

        // 3. Build U (512 values).
        let mut u = [0.0f64; WINDOW_LEN];
        for i in 0..8 {
            for j in 0..SUBBANDS {
                u[i * 64 + j] = self.v[i * 128 + j];
                u[i * 64 + 32 + j] = self.v[i * 128 + 96 + j];
            }
        }

        // 4. Window: W[i] = U[i] * D[i].
        // 5. Sum: out[j] = sum_{i=0..15} W[j + 32*i].
        // Fused: accumulate the windowed value into the output bin as
        // it is produced (W is never needed in full).
        let mut out = [0.0f64; SUBBANDS];
        for (idx, (&uval, &dval)) in u.iter().zip(SYNTHESIS_WINDOW.iter()).enumerate() {
            out[idx % SUBBANDS] += uval * dval;
        }
        out
    }
}

impl Default for SynthesisFilter {
    fn default() -> SynthesisFilter {
        SynthesisFilter::new()
    }
}

/// Convert one reconstructed sample to signed 16-bit PCM.
///
/// The §2.4.3.2 reconstruction produces values in the nominal range
/// `[-1, 1)`; this scales by `32768`, rounds to nearest, and clamps to
/// the `i16` range so the rare over-range excursions a lossy stream
/// can produce do not wrap.
#[inline]
pub fn to_s16(sample: f64) -> i16 {
    let scaled = (sample * 32768.0).round();
    if scaled >= 32767.0 {
        i16::MAX
    } else if scaled <= -32768.0 {
        i16::MIN
    } else {
        scaled as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn matrix_coeff_spot_values() {
        let n = matrix_coeffs();
        // N_ik = cos[(16+i)(2k+1) pi / 64].
        // i=0,k=0: cos(16*1*pi/64) = cos(pi/4).
        assert!((n[0] - (PI / 4.0).cos()).abs() < 1e-12);
        // i=0,k=1: cos(16*3*pi/64) = cos(3pi/4).
        assert!((n[1] - (3.0 * PI / 4.0).cos()).abs() < 1e-12);
        // i=48,k=0: (16+48)*1*pi/64 = pi -> cos(pi) = -1.
        assert!((n[48 * SUBBANDS] - (-1.0)).abs() < 1e-12);
    }

    #[test]
    fn silence_in_yields_silence_out() {
        // Zero subband input must produce zero PCM, regardless of
        // history (V starts zeroed).
        let mut f = SynthesisFilter::new();
        let zeros = [0.0f64; SUBBANDS];
        for _ in 0..20 {
            let out = f.synthesize(&zeros);
            for &v in &out {
                assert_eq!(v, 0.0);
            }
        }
    }

    #[test]
    fn reset_restores_startup_state() {
        // Drive the filter with a non-trivial input, reset, then a run
        // of silence must immediately yield silence again (no lingering
        // V history).
        let mut f = SynthesisFilter::new();
        let mut input = [0.0f64; SUBBANDS];
        input[0] = 0.8;
        input[5] = -0.4;
        for _ in 0..4 {
            let _ = f.synthesize(&input);
        }
        f.reset();
        let out = f.synthesize(&[0.0f64; SUBBANDS]);
        // First slot after reset with zero input: matrixing of zeros
        // gives V[0..64]=0; the windowing of an all-zero V is zero.
        for &v in &out {
            assert_eq!(v, 0.0, "state lingered after reset");
        }
    }

    #[test]
    fn overlap_add_window_length() {
        // Sanity: the fused window/sum touches exactly 512 taps and
        // distributes them across 32 output bins, 16 taps each.
        // Verify by feeding a single unit impulse in subband 0 on the
        // very first slot and checking the output is the expected
        // linear combination of D[] and the matrix column.
        let mut f = SynthesisFilter::new();
        let mut input = [0.0f64; SUBBANDS];
        input[0] = 1.0;
        let out = f.synthesize(&input);
        // Recompute the expected output independently from the same
        // documented steps (V history is all zero on the first slot).
        let n = matrix_coeffs();
        let mut v = [0.0f64; V_LEN];
        for (i, vi) in v.iter_mut().enumerate().take(64) {
            *vi = n[i * SUBBANDS]; // sum over k of N_ik * input[k]; only k=0 nonzero
        }
        let mut u = [0.0f64; WINDOW_LEN];
        for i in 0..8 {
            for j in 0..SUBBANDS {
                u[i * 64 + j] = v[i * 128 + j];
                u[i * 64 + 32 + j] = v[i * 128 + 96 + j];
            }
        }
        let mut expected = [0.0f64; SUBBANDS];
        for (idx, (&uv, &dv)) in u.iter().zip(SYNTHESIS_WINDOW.iter()).enumerate() {
            expected[idx % SUBBANDS] += uv * dv;
        }
        for j in 0..SUBBANDS {
            assert!(
                (out[j] - expected[j]).abs() < 1e-12,
                "bin {j}: got {}, want {}",
                out[j],
                expected[j]
            );
        }
    }

    #[test]
    fn to_s16_clamps_and_rounds() {
        assert_eq!(to_s16(0.0), 0);
        assert_eq!(to_s16(0.5), 16384);
        assert_eq!(to_s16(-0.5), -16384);
        // Over-range excursions clamp rather than wrap.
        assert_eq!(to_s16(2.0), i16::MAX);
        assert_eq!(to_s16(-2.0), i16::MIN);
        assert_eq!(to_s16(1.0), i16::MAX);
    }
}
