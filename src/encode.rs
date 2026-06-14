//! MPEG-1 / MPEG-2 LSF Audio **Layer I** encode (analysis
//! filterbank, scalefactor selection, bit allocation, uniform
//! quantization).
//!
//! Everything in this module is derived solely from ISO/IEC 11172-3
//! (1993), the informative Annex C "The encoding process" plus the
//! normative Layer I clauses, with the ISO/IEC 13818-3 §2.4.2.3 LSF
//! extension that adds the 16 / 22.05 / 24 kHz sampling rates and a
//! distinct Layer I bitrate ladder when the `ID` header bit is `0`:
//!
//! * §C.1.3 / figure C.4 "Analysis Subband Filter Flow Chart" — the
//!   32-band polyphase analysis filterbank: shift 512-element input
//!   vector `X` by 32, window by the Table C.1 coefficients `C[i]`
//!   (`Z[i] = C[i]·X[i]`), partial-sum to 64 values
//!   `Y[i] = Σ_{j=0}^{7} Z[i+64j]`, and matrix to 32 subband samples
//!   `S[i] = Σ_{k=0}^{63} M[i][k]·Y[k]` with
//!   `M[i][k] = cos[(2i+1)(k-16)π/64]`.
//! * §C.1.5.1.4 "Scalefactor calculation" — per subband, every 12
//!   subband samples, take the maximum absolute value and pick the
//!   *lowest* Table 3-B.1 scalefactor that is larger than it.
//! * §C.1.5.1.6 "Bit allocation" — iterative allocation: each step
//!   raises the bit count of the subband with the minimal mask-to-noise
//!   ratio until the frame's bit budget is spent. This clean-room
//!   encoder uses a **non-psychoacoustic** allocator: the per-subband
//!   "signal-to-mask ratio" is replaced by the subband's log-energy, so
//!   the loop spends bits on the loudest subbands first. No
//!   psychoacoustic model (Annex D) is implemented.
//! * §C.1.5.1.7 "Quantization and encoding of subband samples" — a
//!   linear quantizer with symmetric zero: normalize by the scalefactor
//!   to obtain `X`, compute `A·X + B` from the Table C.3 coefficients,
//!   take the `nb` most-significant bits, and invert the MSB (to avoid
//!   the all-`1` sync pattern).
//! * §C.1.5.1.10 / figure C.2 — Layer I frame format: HEADER, ALLOC,
//!   SCALEFACTORS, SAMPLES, ANC, packed MSB-first.
//!
//! The encoder is validated by **self-roundtrip**: encode a known PCM
//! signal, decode it with this crate's own decoder, and assert a
//! bounded RMS / SNR error. ffmpeg ships no MP1 encoder, so there is no
//! external bit-exact oracle; the analysis-filter math, the
//! scalefactor pick, the quantizer, and the allocator are each covered
//! by direct unit tests against the spec formulas.

use crate::decode::{SAMPLES_PER_SUBBAND, SUBBANDS};
use crate::header::{Bitrate, FrameHeader, Mode};
use crate::tables::{ANALYSIS_WINDOW, QUANT_A, QUANT_B, SCALEFACTORS, SNR_DB};

/// Length of the analysis input FIFO `X` (figure C.4: the shift loop
/// runs `i = 511 down to 32`, so `X` has 512 elements).
const X_LEN: usize = 512;

/// The §C.1.3 analysis matrixing coefficients
/// `M[i][k] = cos[(2i+1)(k-16)π/64]` for `i in 0..32`, `k in 0..64`,
/// flattened row-major as `M[i*64 + k]`. Computed once, lazily.
fn analysis_matrix() -> &'static [f64; SUBBANDS * 64] {
    use std::sync::OnceLock;
    static M: OnceLock<[f64; SUBBANDS * 64]> = OnceLock::new();
    M.get_or_init(|| {
        let mut m = [0.0f64; SUBBANDS * 64];
        for i in 0..SUBBANDS {
            for k in 0..64 {
                // M_ik = cos[(2i + 1)(k - 16) * pi / 64].
                let angle = (2 * i + 1) as f64 * (k as f64 - 16.0) * std::f64::consts::PI / 64.0;
                m[i * 64 + k] = angle.cos();
            }
        }
        m
    })
}

/// Per-channel analysis subband filterbank (figure C.4): a 512-element
/// input FIFO `X` that carries the windowing history across slots.
///
/// One [`analyze`](Self::analyze) call consumes 32 fresh input samples
/// (most-recent first per the flow chart) and emits the 32 subband
/// samples for that slot. A Layer I frame is built from 12 slots per
/// channel.
#[derive(Debug, Clone)]
pub struct AnalysisFilter {
    /// The 512-element input FIFO `X`, zero at startup (so the first
    /// slots ramp in, mirroring the decoder's zeroed `V`).
    x: Vec<f64>,
}

impl AnalysisFilter {
    /// A fresh analysis bank with its input FIFO zeroed.
    pub fn new() -> AnalysisFilter {
        AnalysisFilter {
            x: vec![0.0; X_LEN],
        }
    }

    /// Zero the input FIFO (startup state).
    pub fn reset(&mut self) {
        for v in &mut self.x {
            *v = 0.0;
        }
    }

    /// Run one slot of the analysis filter (figure C.4): consume the 32
    /// input PCM samples in `input` and produce 32 subband samples
    /// `S[0..32]`.
    ///
    /// `input` is the next 32 time-domain samples in natural (increasing
    /// time) order; per the flow chart they are shifted into positions
    /// `31..=0` with the most recent at position `0`.
    pub fn analyze(&mut self, input: &[f64; SUBBANDS]) -> [f64; SUBBANDS] {
        // 1. Shift: X[i] = X[i-32] for i = 511 down to 32.
        for i in (32..X_LEN).rev() {
            self.x[i] = self.x[i - 32];
        }
        // 2. Input: X[i] = next_input_audio_sample for i = 31 down to 0,
        //    i.e. the most recent sample lands at X[0]. `input` is in
        //    increasing-time order, so input[31] is the newest.
        for i in (0..SUBBANDS).rev() {
            self.x[i] = input[SUBBANDS - 1 - i];
        }
        // 3. Window: Z[i] = C[i] * X[i] for i in 0..512.
        // 4. Partial calculation: Y[i] = sum_{j=0..7} Z[i + 64j].
        let mut y = [0.0f64; 64];
        for (i, yi) in y.iter_mut().enumerate() {
            let mut acc = 0.0f64;
            for j in 0..8 {
                let idx = i + 64 * j;
                acc += ANALYSIS_WINDOW[idx] * self.x[idx];
            }
            *yi = acc;
        }
        // 5. Matrixing: S[i] = sum_{k=0..63} M[i][k] * Y[k], i in 0..32.
        let m = analysis_matrix();
        let mut s = [0.0f64; SUBBANDS];
        for (i, si) in s.iter_mut().enumerate() {
            let row = &m[i * 64..i * 64 + 64];
            let mut acc = 0.0f64;
            for k in 0..64 {
                acc += row[k] * y[k];
            }
            *si = acc;
        }
        s
    }
}

impl Default for AnalysisFilter {
    fn default() -> AnalysisFilter {
        AnalysisFilter::new()
    }
}

/// The number of standard-defined Table 3-B.1 scalefactor indices
/// (0..=62); index 63 is undefined and never emitted by the encoder.
const MAX_SCF_INDEX: usize = 62;

/// Pick the Table 3-B.1 scalefactor index for a subband (§C.1.5.1.4).
///
/// "The maximum of the absolute value of these 12 samples is
/// determined. The lowest value in table B.1 … which is larger than
/// this maximum is used as the scalefactor." The table is monotonically
/// decreasing (index 0 is the largest multiplier, `2.0`), so the
/// *lowest value larger than `peak`* is the *highest index* whose
/// multiplier still exceeds `peak`.
///
/// Returns the chosen 6-bit index in `0..=62`. When `peak` exceeds the
/// largest table value (`2.0`), index `0` (the largest multiplier) is
/// returned — the sample is at full scale and will clip slightly, which
/// a conformant `[-1, 1)` PCM input cannot reach.
pub fn select_scalefactor(peak: f64) -> u8 {
    // Find the highest index i (0..=62) with SCALEFACTORS[i] > peak.
    // SCALEFACTORS is strictly decreasing across 0..=62, so scan from
    // the smallest multiplier upward and keep the last one that still
    // exceeds peak.
    let mut chosen = 0u8;
    for (i, &v) in SCALEFACTORS.iter().enumerate().take(MAX_SCF_INDEX + 1) {
        if v > peak {
            chosen = i as u8;
        } else {
            break;
        }
    }
    chosen
}

/// Quantize one normalized subband sample to an `nb`-bit code
/// (§C.1.5.1.7).
///
/// `value` is the raw subband sample `s'` and `scf` the chosen Table
/// 3-B.1 multiplier. The sample is normalized `X = value / scf`, then
/// `A·X + B` is computed (Table C.3), the `nb` most-significant bits are
/// taken, and the MSB is inverted. `nb` must be in `2..=15`.
///
/// This is the exact inverse of [`crate::decode::requantize`]: feeding
/// the returned code back through `requantize` recovers `X` to within
/// one quantizer step.
pub fn quantize(value: f64, scf: f64, nb: u8) -> u32 {
    debug_assert!((2..=15).contains(&nb), "Layer I sample width 2..=15");
    let nb_u = nb as usize;
    let x = value / scf;
    // §C.1.5.1.7 step 1: AX + B, a signed fractional value in
    // [-1, 1 - 2·2^-nb] for X in [-1, 1).
    let axb = QUANT_A[nb_u] * x + QUANT_B[nb_u];
    // §C.1.5.1.7 step 2: "take the N most significant bits". With AX+B
    // viewed as a fractional number whose MSB has weight -1 (the same
    // convention §2.4.3.2 uses for the decoder's `s'''`), scaling by
    // 2^(nb-1) and flooring yields an `nb`-bit signed integer in
    // [-2^(nb-1), 2^(nb-1) - 1]. Re-bias by +2^(nb-1) for an unsigned
    // value in [0, 2^nb - 1]. The +bias step is also exactly what
    // "invert the MSB" produces on a two's-complement nb-bit word, so
    // steps 2 and 3 combine into a single offset+floor here.
    let half = 1u32 << (nb - 1);
    let scaled = (axb * half as f64).floor();
    (scaled + half as f64).clamp(0.0, ((1u32 << nb) - 1) as f64) as u32
}

/// A bit sink that packs values MSB-first into a byte vector
/// (§2.3 / §2.4.1.5: the Layer I bitstream is written most-significant
/// bit first within each byte).
#[derive(Debug, Default)]
pub struct BitWriter {
    bytes: Vec<u8>,
    acc: u32,
    nbits: u8,
}

impl BitWriter {
    /// A fresh, empty writer.
    pub fn new() -> BitWriter {
        BitWriter::default()
    }

    /// Append the low `n` bits of `val`, most-significant first.
    pub fn put(&mut self, val: u32, n: u8) {
        for i in (0..n).rev() {
            let b = (val >> i) & 1;
            self.acc = (self.acc << 1) | b;
            self.nbits += 1;
            if self.nbits == 8 {
                self.bytes.push(self.acc as u8);
                self.acc = 0;
                self.nbits = 0;
            }
        }
    }

    /// Number of whole bytes already flushed.
    pub fn byte_len(&self) -> usize {
        self.bytes.len()
    }

    /// Flush any partial trailing byte (zero-padded on the low side) and
    /// return the packed bytes.
    pub fn finish(mut self) -> Vec<u8> {
        if self.nbits > 0 {
            self.acc <<= 8 - self.nbits;
            self.bytes.push(self.acc as u8);
            self.acc = 0;
            self.nbits = 0;
        }
        self.bytes
    }
}

/// The §C.1.5.1.6 bit-allocation result: the per-channel, per-subband
/// number of bits `nb` (`0`, or `2..=15`).
pub type Allocation = [[u8; SUBBANDS]; 2];

/// Compute the number of audio-data bits available in a Layer I frame
/// at `bitrate` (kbit/s) and `sampling_frequency` (Hz), after the
/// header and (optional) CRC.
///
/// §2.4.2.1: a Layer I frame spans `N = floor(12·bitrate/Fs)` slots of
/// four bytes each, carrying 384 samples. The header is 32 bits; the
/// 4-bit allocation fields and 6-bit scalefactors are *part of* the
/// audio data budget the allocator distributes, so this returns the
/// total frame payload bits minus the header (and CRC), i.e. the
/// `adb` budget the §C.1.5.1.6 loop draws `bbal + bscf + bspl` from.
fn frame_payload_bits(
    bitrate_kbps: u32,
    sampling_frequency: u32,
    has_crc: bool,
    nch: usize,
) -> usize {
    let slots = (12 * bitrate_kbps * 1000) / sampling_frequency;
    let total = slots as usize * 32; // 4 bytes/slot * 8 bits
    let header = 32 + if has_crc { 16 } else { 0 };
    // Each subband always spends 4 allocation bits per channel even when
    // unallocated, so the allocation field cost is fixed.
    let alloc_cost = SUBBANDS * nch * 4;
    total.saturating_sub(header + alloc_cost)
}

/// Run the §C.1.5.1.6 iterative bit allocation for one frame.
///
/// `energy[ch][sb]` is a per-subband signal-energy proxy (the maximum
/// absolute subband sample, the same quantity used to pick the
/// scalefactor). `budget_bits` is the number of bits available for
/// scalefactors + samples after the header, CRC and the fixed
/// allocation field (`adb` of §C.1.5.1.6).
///
/// The loop, faithful to §C.1.5.1.6 but **without** a psychoacoustic
/// model: each iteration finds the subband with the minimal
/// mask-to-noise ratio (here `MNR = SNR(nb) − signal_level_dB`) among
/// the subbands that can still afford one more bit-step, and raises its
/// allocation to the next level. A subband moving from 0→2 bits also
/// costs 6 scalefactor bits. Iteration stops when no further increase
/// fits the budget.
// The allocator scans the (ch, sb) grid every iteration and updates
// `alloc[ch][sb]` by index; an iterator rewrite would obscure the
// §C.1.5.1.6 "find the subband with the minimal MNR" loop.
#[allow(clippy::needless_range_loop)]
pub fn allocate_bits(energy: &[[f64; SUBBANDS]; 2], nch: usize, budget_bits: usize) -> Allocation {
    let mut alloc: Allocation = [[0u8; SUBBANDS]; 2];
    // Per-subband target "level" in dB: louder subbands have a higher
    // signal level and so a higher MNR demand. Use 20·log10(peak).
    let mut level_db = [[f64::NEG_INFINITY; SUBBANDS]; 2];
    for ch in 0..nch {
        for sb in 0..SUBBANDS {
            let p = energy[ch][sb];
            level_db[ch][sb] = if p > 0.0 {
                20.0 * p.log10()
            } else {
                f64::NEG_INFINITY
            };
        }
    }

    // Bits spent so far on scalefactors + samples.
    let mut spent = 0usize;
    // Cost (in bits) of raising (ch, sb) from its current nb to the next
    // legal nb, plus the one-time +6 scalefactor cost on the 0→2 jump.
    let next_nb = |nb: u8| -> u8 {
        if nb == 0 {
            2
        } else if nb < 15 {
            nb + 1
        } else {
            15
        }
    };
    let step_cost = |nb: u8| -> usize {
        let nn = next_nb(nb);
        // 12 samples per subband: extra sample bits = 12 * (nn - nb).
        let sample_bits = 12 * (nn as usize - nb as usize);
        let scf_bits = if nb == 0 { 6 } else { 0 };
        sample_bits + scf_bits
    };

    loop {
        // Find the subband with the minimal MNR that can still grow and
        // afford its next step within the remaining budget.
        let mut best: Option<(usize, usize, f64)> = None;
        for ch in 0..nch {
            for sb in 0..SUBBANDS {
                let nb = alloc[ch][sb];
                if nb >= 15 {
                    continue;
                }
                // A silent subband never needs bits.
                if level_db[ch][sb] == f64::NEG_INFINITY {
                    continue;
                }
                if spent + step_cost(nb) > budget_bits {
                    continue;
                }
                // MNR after this band's *current* allocation: the noise
                // the listener still hears. Lower MNR == more urgent.
                let snr = SNR_DB[nb as usize];
                let mnr = snr - level_db[ch][sb];
                let better = match best {
                    None => true,
                    Some((_, _, bmnr)) => mnr < bmnr,
                };
                if better {
                    best = Some((ch, sb, mnr));
                }
            }
        }
        match best {
            Some((ch, sb, _)) => {
                let nb = alloc[ch][sb];
                spent += step_cost(nb);
                alloc[ch][sb] = next_nb(nb);
            }
            None => break,
        }
    }
    alloc
}

/// Map an `nb` (bits per sample) back to the 4-bit `allocation` code
/// (§2.4.2.5, the inverse of [`crate::decode::allocation_bits`]).
///
/// `0` bits → code `0`; `2..=15` bits → code `nb - 1` (so 2→1, …,
/// 15→14). `nb == 1` is not a legal Layer I allocation and never
/// produced by [`allocate_bits`].
fn allocation_code(nb: u8) -> u8 {
    if nb == 0 {
        0
    } else {
        nb - 1
    }
}

// ---- Layer II scalefactor extraction (§C.1.5.1.4 per part) -----

/// Per-(ch, sb, part) Table 3-B.1 scalefactor *indices* the §2.4.1.6
/// scalefactor field consumes. One index per scalefactor part: part 0
/// covers slot range `0..12`, part 1 `12..24`, part 2 `24..36`
/// (§2.4.2.6 / §2.4.3.3.2 — "three equal parts of 12 subband samples").
///
/// `indices[ch][sb][part]` is in `0..=62`; index `63` is the §2.4.2.6
/// "undefined" reserved code and is never produced.
pub type Layer2ScalefactorIndices = [[[u8; 3]; SUBBANDS]; 2];

/// Per-(ch, sb, part) peak (maximum absolute analyzed sub-band sample)
/// across the 12 sample-slots belonging to a single §2.4.2.6
/// scalefactor part. The shape matches [`Layer2ScalefactorIndices`].
///
/// `peaks[ch][sb][part]` is the input quantity §C.1.5.1.4 calls
/// "the maximum of the absolute value of these 12 samples", taken over
/// the part-`part` granules (`gr in part*4 .. (part+1)*4`, three
/// samples per granule).
pub type Layer2SubbandPeaks = [[[f64; 3]; SUBBANDS]; 2];

/// Compute the §C.1.5.1.4 per-part peak amplitudes for one Layer II
/// frame.
///
/// `subbands[ch][sb]` is the 36-slot analysed sub-band trace for one
/// (ch, sb), laid out `[s_0, s_1, …, s_35]` to match the decoder-side
/// `Layer2Subband::samples` storage and the encoder analysis-filter
/// output order (slot-major: slot 0 is the oldest, slot 35 the newest
/// in the frame). Only the first `nch` rows are read; only the
/// `0..sblimit` columns receive a non-zero peak (subbands above
/// `sblimit` are §2.4.3.3.1 "forced to zero" by both sides and never
/// carry a scalefactor).
///
/// The 36 slots are split into three §2.4.2.6 scalefactor parts of 12
/// slots each: part `p` covers `slot in p*12 .. (p+1)*12`. The returned
/// peak is the maximum of `|s_slot|` over that range. Above-`sblimit`
/// and above-`nch` cells are zero.
// The (ch, sb, part, slot) walk is naturally index-driven (the §2.4.2.6
// part windowing is "slot range part*12..(part+1)*12"); an iterator
// rewrite would obscure the spec mapping.
#[allow(clippy::needless_range_loop)]
pub fn layer2_subband_peak_per_part(
    subbands: &[[[f64; 36]; SUBBANDS]; 2],
    nch: usize,
    sblimit: usize,
) -> Layer2SubbandPeaks {
    let mut peaks: Layer2SubbandPeaks = [[[0.0f64; 3]; SUBBANDS]; 2];
    let nch = nch.min(2);
    let sblimit = sblimit.min(SUBBANDS);
    for ch in 0..nch {
        for sb in 0..sblimit {
            for part in 0..3 {
                let mut m = 0.0f64;
                for slot in part * 12..(part + 1) * 12 {
                    let a = subbands[ch][sb][slot].abs();
                    if a > m {
                        m = a;
                    }
                }
                peaks[ch][sb][part] = m;
            }
        }
    }
    peaks
}

/// Extract the per-(ch, sb, part) Table 3-B.1 scalefactor indices for
/// one Layer II frame (§C.1.5.1.4 applied to each §2.4.2.6 scalefactor
/// part independently).
///
/// For every (ch, sb, part) the maximum of the absolute value of the
/// 12 sub-band samples belonging to that part is determined, then
/// [`select_scalefactor`] picks the *lowest Table 3-B.1 value larger
/// than this maximum* (equivalently, the *highest* index in `0..=62`
/// whose multiplier still exceeds the peak — the table is monotonically
/// decreasing). For all-zero parts the all-largest-index `62` is
/// returned, mirroring the Layer I helper's "tiniest scalefactor"
/// fallback.
///
/// The returned indices are intended to populate
/// [`Layer2ScalefactorFieldInput::scalefactor_indices`] directly; the
/// SCFSI selection that may collapse them to one or two values is a
/// separate step (Table C.4 — DOCS-GAP) and is the caller's
/// responsibility. Writing `scfsi == 0b00` (three independent
/// scalefactors) consumes all three indices as produced.
///
/// `subbands[ch][sb]` carries the 36 analysed sub-band sample values
/// the encoder's [`AnalysisFilter`] has produced for the frame, in
/// slot-major order; the slot-to-part split is `part = slot / 12`.
/// Only the first `nch` rows and the `0..sblimit` columns of the
/// returned `[[[u8; 3]; SUBBANDS]; 2]` are meaningful (the rest stay at
/// the "tiniest scalefactor" `62` index but the §2.4.1.6 writer will
/// not emit them for `alloc == 0` / above-sblimit subbands).
// Same (ch, sb, part) index loop as `layer2_subband_peak_per_part`; an
// iterator rewrite would lose the spec mapping.
#[allow(clippy::needless_range_loop)]
pub fn select_layer2_scalefactors(
    subbands: &[[[f64; 36]; SUBBANDS]; 2],
    nch: usize,
    sblimit: usize,
) -> Layer2ScalefactorIndices {
    let peaks = layer2_subband_peak_per_part(subbands, nch, sblimit);
    let mut out: Layer2ScalefactorIndices = [[[MAX_SCF_INDEX as u8; 3]; SUBBANDS]; 2];
    let nch = nch.min(2);
    let sblimit = sblimit.min(SUBBANDS);
    for ch in 0..nch {
        for sb in 0..sblimit {
            for part in 0..3 {
                out[ch][sb][part] = select_scalefactor(peaks[ch][sb][part]);
            }
        }
    }
    out
}

// ---- Layer II bit allocation (§C.1.5.2.7) ----------------------

use crate::header::{Emphasis, ModeExtension};
use crate::tables_layer2::{layer2_snr_db, AllocationTable, QuantClass};

/// Per-frame, per-channel, per-subband **Layer II** allocation indices.
///
/// `alloc[ch][sb]` is the raw `nbal`-bit `allocation[ch][sb]` value the
/// bitstream stores (§2.4.1.6) — `0` means "no samples transferred", and
/// `1..=(1<<nbal)-1` index into the row's Table 3-B.2x level list
/// (after subtracting one). `None` cells in that row are never selected
/// by [`allocate_bits_layer2`].
pub type Layer2Allocation = [[u8; SUBBANDS]; 2];

/// Cost in bits of one (sb, ch) sample region under a Layer II
/// quantization class, per frame (§C.1.5.2.7 `bspl` increment for one
/// subband). Layer II carries 36 sub-band samples per (sb, ch); for a
/// grouped class three samples share one `bits_per_codeword`-bit
/// codeword (12 codewords per frame), and for a non-grouped class each
/// sample is its own codeword (36 codewords per frame).
fn layer2_class_cost_bits(class: &QuantClass) -> usize {
    use crate::decode_layer2::LAYER2_SAMPLES_PER_SUBBAND;
    let n_codewords = LAYER2_SAMPLES_PER_SUBBAND / class.samples_per_codeword as usize;
    n_codewords * class.bits_per_codeword as usize
}

/// One-time per-(sb, ch) bookkeeping cost charged when a subband first
/// receives a non-zero allocation: the 2-bit `scfsi` field (§2.4.1.6)
/// plus, conservatively, three 6-bit scalefactor indices (the maximum
/// the §C.1.5.2.5 / Table C.4 selection logic can choose).
///
/// The Table C.4 SCFSI selection table is rendered as an image in the
/// PDF that the staging text layer cannot extract reliably, so this
/// encoder writes the worst-case `scfsi == '00'` (three scalefactors)
/// for every allocated subband. That keeps the encoder's allocation
/// fit-check sound at the cost of slightly underspending the budget on
/// signals whose successive scalefactors collapse — see the per-frame
/// "bsel + bscf" terms in the §C.1.5.2.7 budget formula.
const LAYER2_PER_SUBBAND_OVERHEAD_BITS: usize = 2 + 3 * 6;

/// Per-channel sum of `nbal[sb]` for subbands `0..sblimit` under one
/// Table B.2x — the fixed cost of the `allocation` field per channel,
/// independent of which raw allocation values are written.
pub fn sum_nbal_per_channel(table: &AllocationTable) -> usize {
    let mut s = 0usize;
    for sb in 0..table.sblimit() {
        s += table.nbal(sb) as usize;
    }
    s
}

/// Total Layer II audio-data budget for one frame (`adb` of §C.1.5.2.7),
/// in bits, after subtracting the §2.4.1.6 header, the optional CRC,
/// and the `bbal` allocation-field cost.
///
/// §2.4.2.1 sizes a Layer II frame as `N = floor(144 · bitrate / Fs)`
/// bytes (1152 samples / 8 = 144). The header is 32 bits; an optional
/// §2.4.1.4 CRC is 16 bits. `bbal = nch · Σ nbal[sb]`.
pub fn layer2_frame_payload_bits(
    bitrate_kbps: u32,
    sampling_frequency: u32,
    has_crc: bool,
    nch: usize,
    table: &AllocationTable,
) -> usize {
    let bytes = (144 * bitrate_kbps * 1000) / sampling_frequency;
    let total = bytes as usize * 8;
    let header = 32 + if has_crc { 16 } else { 0 };
    let bbal = nch * sum_nbal_per_channel(table);
    total.saturating_sub(header + bbal)
}

/// Step the per-(sb, ch) allocation up to the next legal Table 3-B.2x
/// column for that subband, skipping any `None` (`-`) cells. Returns the
/// new allocation index and the new quantization class, or `None` if
/// the subband has already reached the last column on its row.
fn layer2_next_alloc(
    table: &AllocationTable,
    sb: usize,
    current: u8,
) -> Option<(u8, &'static QuantClass)> {
    let max_alloc = 1u8 << table.nbal(sb);
    let mut next = current + 1;
    while next < max_alloc {
        if let Some(cls) = table.quant_class(sb, next) {
            return Some((next, cls));
        }
        next += 1;
    }
    None
}

/// Run the §C.1.5.2.7 iterative bit allocation for one Layer II frame.
///
/// `energy[ch][sb]` is the per-subband signal-energy proxy (the maximum
/// absolute analysed subband sample over the frame's 36 sample-slots),
/// the same quantity that drives the Layer I allocator
/// [`allocate_bits`]. `budget_bits` is the `adb` budget after the
/// header, CRC and `bbal` have been subtracted; pass the result of
/// [`layer2_frame_payload_bits`].
///
/// The loop is the spec algorithm:
///
/// > Determination of the minimal MNR of all subbands. The accuracy of
/// > the quantization of the subband with the minimal MNR is increased
/// > by using the next higher entry in the relevant table B.2 […]. bspl
/// > is updated according to the additional number of bits required. If
/// > a non-zero number of bits is assigned to a subband for the first
/// > time, bsel has to be updated, and bscf has to be updated.
///
/// Since the perceptual SMR depends on the Annex D model that this
/// crate documents as a DOCS-GAP, the encoder substitutes a signal-
/// energy proxy: `MNR = SNR(nlevels) − 20·log10(peak)`. The §C.1.5.2.7
/// "first non-zero allocation" overhead is the 2-bit scfsi plus three
/// 6-bit scalefactor indices (the Table C.4 SCFSI selection is a
/// DOCS-GAP; the encoder writes `scfsi == '00'` so the cost is
/// independent of the input signal).
///
/// Subbands at or above `table.sblimit()` are forced to allocation `0`.
// The §C.1.5.2.7 allocator is naturally an index-driven (ch, sb) double
// loop; rewriting as iterator chains would obscure the "find min-MNR
// (ch, sb)" loop body the spec literally describes.
#[allow(clippy::needless_range_loop)]
pub fn allocate_bits_layer2(
    energy: &[[f64; SUBBANDS]; 2],
    nch: usize,
    table: &AllocationTable,
    budget_bits: usize,
) -> Layer2Allocation {
    let sblimit = table.sblimit();
    let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];

    // Pre-compute the per-(ch, sb) signal level in dB. Silent subbands
    // skip allocation entirely.
    let mut level_db = [[f64::NEG_INFINITY; SUBBANDS]; 2];
    for ch in 0..nch {
        for sb in 0..sblimit {
            let p = energy[ch][sb];
            level_db[ch][sb] = if p > 0.0 {
                20.0 * p.log10()
            } else {
                f64::NEG_INFINITY
            };
        }
    }

    // Current quantization-class cost per (ch, sb); for `allocation = 0`
    // this is `None` (no codewords transmitted).
    let mut current_cost: [[usize; SUBBANDS]; 2] = [[0; SUBBANDS]; 2];

    let mut spent = 0usize;

    loop {
        // §C.1.5.2.7 step 1: find the subband with the minimal MNR that
        // can still grow to a next legal allocation under the remaining
        // budget.
        let mut best: Option<(usize, usize, f64, u8, &'static QuantClass, usize)> = None;
        for ch in 0..nch {
            for sb in 0..sblimit {
                if level_db[ch][sb] == f64::NEG_INFINITY {
                    continue;
                }
                let (next_idx, next_cls) = match layer2_next_alloc(table, sb, alloc[ch][sb]) {
                    Some(p) => p,
                    None => continue,
                };
                let new_cost = layer2_class_cost_bits(next_cls);
                let delta_sample_bits = new_cost - current_cost[ch][sb];
                let overhead = if alloc[ch][sb] == 0 {
                    LAYER2_PER_SUBBAND_OVERHEAD_BITS
                } else {
                    0
                };
                let step_cost = delta_sample_bits + overhead;
                if spent + step_cost > budget_bits {
                    continue;
                }
                // SNR after the proposed step: the quieter the subband
                // for that SNR, the smaller (more negative) the MNR, the
                // more urgent the next bit.
                let snr = layer2_snr_db(next_cls.nlevels)
                    .unwrap_or_else(|| panic!("nlevels {} missing SNR row", next_cls.nlevels));
                let mnr = snr - level_db[ch][sb];
                let better = match best {
                    None => true,
                    Some((_, _, bmnr, _, _, _)) => mnr < bmnr,
                };
                if better {
                    best = Some((ch, sb, mnr, next_idx, next_cls, step_cost));
                }
            }
        }
        match best {
            Some((ch, sb, _, next_idx, next_cls, step_cost)) => {
                spent += step_cost;
                alloc[ch][sb] = next_idx;
                current_cost[ch][sb] = layer2_class_cost_bits(next_cls);
            }
            None => break,
        }
    }
    alloc
}

// ---- Layer II frame-header writer (§2.4.2.3) -------------------

/// Errors raised while packing a Layer II frame header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer2HeaderError {
    /// `sampling_frequency` is not on the MPEG-1 (44.1 / 48 / 32 kHz)
    /// nor the MPEG-2 LSF (16 / 22.05 / 24 kHz) Layer II tables.
    UnsupportedSamplingFrequency(u32),
    /// `bitrate` is not a fixed value on the §2.4.2.3 Layer II ladder
    /// matching the implied `ID` bit (MPEG-1 vs LSF).
    UnsupportedBitrate(u16),
    /// `bitrate` is a `Free` / `Forbidden` enum rather than a fixed
    /// value; the writer only emits fixed-rate Layer II headers.
    NonFixedBitrate,
}

impl core::fmt::Display for Layer2HeaderError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Layer2HeaderError::UnsupportedSamplingFrequency(hz) => {
                write!(f, "Layer II header: unsupported sampling frequency {hz} Hz")
            }
            Layer2HeaderError::UnsupportedBitrate(kbps) => {
                write!(f, "Layer II header: unsupported bitrate {kbps} kbit/s")
            }
            Layer2HeaderError::NonFixedBitrate => {
                write!(f, "Layer II header: non-fixed bitrate (Free / Forbidden)")
            }
        }
    }
}

impl std::error::Error for Layer2HeaderError {}

/// The §2.4.2.3 Layer II `bitrate_index` for a fixed bitrate in kbit/s,
/// given the `ID` bit (`1` → MPEG-1 ladder, `0` → MPEG-2 LSF ladder),
/// or `None` when the bitrate is not on that ladder.
///
/// MPEG-1 Layer II ladder (11172-3 §2.4.2.3): `32 / 48 / 56 / 64 / 80 /
/// 96 / 112 / 128 / 160 / 192 / 224 / 256 / 320 / 384` kbit/s, encoded
/// at indices `0b0001..=0b1110`. LSF Layer II/III ladder (13818-3
/// §2.4.2.3, shared between Layer II and Layer III for the LSF
/// edition): `8 / 16 / 24 / 32 / 40 / 48 / 56 / 64 / 80 / 96 / 112 /
/// 128 / 144 / 160` kbit/s, encoded at indices `0b0001..=0b1110`.
///
/// Index `0b0000` (free format) and `0b1111` (forbidden) are never
/// returned: callers stage free format separately, and forbidden is
/// not a writable value.
pub fn bitrate_index_layer2(kbps: u16, id_bit: u8) -> Option<u8> {
    const L2_MPEG1: [u16; 14] = [
        32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384,
    ];
    const L2_LSF: [u16; 14] = [8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160];
    let ladder: &[u16] = if id_bit == 1 { &L2_MPEG1 } else { &L2_LSF };
    ladder
        .iter()
        .position(|&k| k == kbps)
        .map(|p| (p + 1) as u8)
}

/// Parameters required to write a Layer II frame header (§2.4.2.3).
///
/// Mirrors the thirteen fields in the §2.4.1.3 syntax verbatim. The
/// caller chooses every field; the writer never invents a default
/// beyond what the round-tripping `FrameHeader::parse` requires.
///
/// `sampling_frequency` and `bitrate_kbps` together imply the `ID`
/// bit:
///
/// * `44_100 / 48_000 / 32_000` Hz → `ID == 1` (MPEG-1), and
///   `bitrate_kbps` must be on the MPEG-1 Layer II ladder.
/// * `22_050 / 24_000 / 16_000` Hz → `ID == 0` (MPEG-2 LSF), and
///   `bitrate_kbps` must be on the LSF Layer II/III ladder.
///
/// `padding_bit`, `private_bit`, `copyright`, `original` and
/// `mode_extension` are carried verbatim; the writer does not
/// recompute padding from the per-frame byte budget.
#[derive(Debug, Clone, Copy)]
pub struct Layer2HeaderParams {
    /// Sampling frequency in Hz; must be one of the six allowed
    /// values listed above.
    pub sampling_frequency: u32,
    /// Layer II bitrate in kbit/s; must be on the §2.4.2.3 ladder
    /// matching the implied `ID` bit.
    pub bitrate_kbps: u16,
    /// Channel mode (`mode` field).
    pub mode: Mode,
    /// Raw two-bit `mode_extension` value; only meaningful when
    /// `mode == Mode::JointStereo`.
    pub mode_extension: ModeExtension,
    /// `padding_bit`: `true` adds one extra slot to the frame.
    pub padding: bool,
    /// `private_bit`: reserved for private use, never assigned by ISO.
    pub private: bool,
    /// `copyright`: `true` if copyright-protected.
    pub copyright: bool,
    /// `original/copy`: `true` if this is the original (not a copy).
    pub original: bool,
    /// `emphasis` field (de-emphasis type).
    pub emphasis: Emphasis,
    /// `true` if the optional §2.4.1.4 `error_check()` CRC field is
    /// present in the frame; controls the `protection_bit`. The CRC
    /// word itself is **not** written by this function — callers
    /// reserve a placeholder after the header and patch it once the
    /// allocation/scfsi fields have been emitted (see
    /// [`Mp1FrameEncoder::encode_frame`] for the Layer I template).
    pub has_crc: bool,
}

impl Layer2HeaderParams {
    /// Build a minimal Layer II header parameter set: the three
    /// "mandatory" fields (sampling frequency, bitrate, channel mode)
    /// and `emphasis = None`. Every other field defaults to its zero
    /// value (no padding, not private, not copyright, original = 1,
    /// no CRC, mode_extension = 0).
    pub fn new(sampling_frequency: u32, bitrate_kbps: u16, mode: Mode) -> Layer2HeaderParams {
        Layer2HeaderParams {
            sampling_frequency,
            bitrate_kbps,
            mode,
            mode_extension: ModeExtension(0),
            padding: false,
            private: false,
            copyright: false,
            original: true,
            emphasis: Emphasis::None,
            has_crc: false,
        }
    }
}

/// The two-bit code for an [`Emphasis`] (§2.4.2.3).
fn emphasis_code(e: Emphasis) -> u8 {
    match e {
        Emphasis::None => 0b00,
        Emphasis::Ms5015 => 0b01,
        Emphasis::Reserved => 0b10,
        Emphasis::CcittJ17 => 0b11,
    }
}

/// The two-bit code for a [`Mode`] (§2.4.2.3).
fn mode_code(m: Mode) -> u8 {
    match m {
        Mode::Stereo => 0b00,
        Mode::JointStereo => 0b01,
        Mode::DualChannel => 0b10,
        Mode::SingleChannel => 0b11,
    }
}

/// Pack a Layer II frame header (§2.4.2.3) into a four-byte big-endian
/// word.
///
/// Returns the four header bytes ready to write at the start of a
/// Layer II frame. The order of fields and their bit widths come
/// **directly** from §2.4.1.3:
///
/// | bits | field                | width |
/// |------|----------------------|-------|
/// | 31:20 | syncword            | 12    |
/// | 19    | ID                  | 1     |
/// | 18:17 | layer (`0b10` = II) | 2     |
/// | 16    | protection_bit      | 1     |
/// | 15:12 | bitrate_index       | 4     |
/// | 11:10 | sampling_frequency  | 2     |
/// | 9     | padding_bit         | 1     |
/// | 8     | private_bit         | 1     |
/// | 7:6   | mode                | 2     |
/// | 5:4   | mode_extension      | 2     |
/// | 3     | copyright           | 1     |
/// | 2     | original/copy       | 1     |
/// | 1:0   | emphasis            | 2     |
///
/// The `ID` and `sampling_frequency` codes are resolved from `params.
/// sampling_frequency` via the §2.4.2.3 table; the `bitrate_index` is
/// resolved from `params.bitrate_kbps` against the Layer II ladder
/// matching the chosen `ID` (see [`bitrate_index_layer2`]). The
/// `protection_bit` is `0` when `params.has_crc` is true, `1`
/// otherwise — per §2.4.2.3 the bit indicates redundancy *has* been
/// added (`'0'`) or has not (`'1'`).
///
/// Errors:
///
/// * [`Layer2HeaderError::UnsupportedSamplingFrequency`] — `params.
///   sampling_frequency` is not on the §2.4.2.3 table.
/// * [`Layer2HeaderError::UnsupportedBitrate`] — `params.bitrate_kbps`
///   is not on the Layer II ladder for the implied `ID`.
pub fn pack_layer2_header(params: &Layer2HeaderParams) -> Result<[u8; 4], Layer2HeaderError> {
    let (id_bit, samp_code) = sampling_code(params.sampling_frequency).ok_or(
        Layer2HeaderError::UnsupportedSamplingFrequency(params.sampling_frequency),
    )?;
    let brate_idx = bitrate_index_layer2(params.bitrate_kbps, id_bit)
        .ok_or(Layer2HeaderError::UnsupportedBitrate(params.bitrate_kbps))?;

    // Pack MSB-first into a 32-bit big-endian word per §2.4.1.3. The
    // syncword sits in bits 31..20 (top 12 bits = 0xFFF). Every
    // subsequent field shifts in at its (32 - top_bit) position.
    let word: u32 = (0xFFFu32 << 20)
        | ((id_bit as u32 & 0x1) << 19)
        | (0b10u32 << 17) // layer II
        | (if params.has_crc { 0 } else { 1u32 } << 16)
        | ((brate_idx as u32 & 0xF) << 12)
        | ((samp_code as u32 & 0x3) << 10)
        | ((u32::from(params.padding)) << 9)
        | ((u32::from(params.private)) << 8)
        | ((mode_code(params.mode) as u32 & 0x3) << 6)
        | ((params.mode_extension.0 as u32 & 0x3) << 4)
        | ((u32::from(params.copyright)) << 3)
        | ((u32::from(params.original)) << 2)
        | (emphasis_code(params.emphasis) as u32 & 0x3);
    Ok(word.to_be_bytes())
}

/// Append a packed Layer II frame header (§2.4.2.3) to `bw` MSB-first.
///
/// Convenience wrapper around [`pack_layer2_header`] for callers that
/// are already streaming bits into a [`BitWriter`]; the four header
/// bytes are pushed exactly as if `pack_layer2_header(params).unwrap()`
/// had been written byte-by-byte. Bit width: 32.
pub fn write_layer2_header(
    bw: &mut BitWriter,
    params: &Layer2HeaderParams,
) -> Result<(), Layer2HeaderError> {
    let bytes = pack_layer2_header(params)?;
    for b in bytes {
        bw.put(b as u32, 8);
    }
    Ok(())
}

// ---- Layer II allocation-field writer (§2.4.1.6) ---------------

/// Errors raised while writing the §2.4.1.6 Layer II allocation field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer2AllocationFieldError {
    /// `nch` was not `1` or `2`.
    UnsupportedChannelCount(usize),
    /// `bound > table.sblimit()`. The intensity-stereo bound cannot
    /// exceed the in-stream subband count.
    BoundExceedsSblimit {
        /// The supplied `bound`.
        bound: usize,
        /// `table.sblimit()`.
        sblimit: usize,
    },
    /// `bound < sblimit` with `nch == 1`. Mono frames never share an
    /// allocation field across "two" channels; callers that build a mono
    /// header must pass `bound == sblimit`.
    MonoBoundBelowSblimit {
        /// The supplied `bound`.
        bound: usize,
        /// `table.sblimit()`.
        sblimit: usize,
    },
    /// A non-zero allocation was set for a subband at or above
    /// `table.sblimit()`. The §2.4.1.6 syntax never emits a bit for
    /// those subbands.
    NonZeroAllocationAboveSblimit {
        /// Subband index that carried a non-zero allocation.
        subband: usize,
        /// `table.sblimit()`.
        sblimit: usize,
        /// Channel that carried the non-zero allocation.
        channel: usize,
    },
    /// A per-(ch, sb) allocation does not fit in `nbal` bits, or selects
    /// a `-` cell of the Table 3-B.2x row. Either case is rejected by
    /// the decoder's `quant_class` lookup as well.
    InvalidAllocationCode {
        /// Channel of the offending allocation.
        channel: usize,
        /// Subband of the offending allocation.
        subband: usize,
        /// The supplied allocation value.
        allocation: u8,
    },
    /// In the intensity_stereo upper band `[bound, sblimit)` the
    /// §2.4.1.6 syntax stores **one** allocation value per subband
    /// (shared between channels). The caller passed differing
    /// `alloc[0][sb]` and `alloc[1][sb]`, which the writer refuses to
    /// silently collapse.
    UpperBandChannelsDisagree {
        /// Subband index in the shared upper band.
        subband: usize,
        /// `alloc[0][sb]`.
        left: u8,
        /// `alloc[1][sb]`.
        right: u8,
    },
}

impl core::fmt::Display for Layer2AllocationFieldError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Layer2AllocationFieldError::UnsupportedChannelCount(n) => {
                write!(
                    f,
                    "Layer II allocation field: unsupported channel count {n}"
                )
            }
            Layer2AllocationFieldError::BoundExceedsSblimit { bound, sblimit } => {
                write!(
                    f,
                    "Layer II allocation field: bound {bound} exceeds sblimit {sblimit}"
                )
            }
            Layer2AllocationFieldError::MonoBoundBelowSblimit { bound, sblimit } => {
                write!(
                    f,
                    "Layer II allocation field: mono bound {bound} < sblimit {sblimit}"
                )
            }
            Layer2AllocationFieldError::NonZeroAllocationAboveSblimit {
                subband,
                sblimit,
                channel,
            } => {
                write!(
                    f,
                    "Layer II allocation field: non-zero allocation at \
                     sb={subband} ch={channel} (sblimit={sblimit})"
                )
            }
            Layer2AllocationFieldError::InvalidAllocationCode {
                channel,
                subband,
                allocation,
            } => {
                write!(
                    f,
                    "Layer II allocation field: invalid allocation code \
                     {allocation} at sb={subband} ch={channel}"
                )
            }
            Layer2AllocationFieldError::UpperBandChannelsDisagree {
                subband,
                left,
                right,
            } => {
                write!(
                    f,
                    "Layer II allocation field: upper-band sb={subband} \
                     allocations differ between channels ({left} vs {right})"
                )
            }
        }
    }
}

impl std::error::Error for Layer2AllocationFieldError {}

/// Write the §2.4.1.6 Layer II `allocation[ch][sb]` field into `bw`,
/// MSB-first.
///
/// The §2.4.1.6 syntax sizes one `nbal[sb]`-bit `allocation` slot per
/// channel for subbands `[0, bound)` and one shared `nbal[sb]`-bit slot
/// for subbands `[bound, sblimit)` (intensity_stereo: both channels
/// share one upper-band allocation). Subbands at or above `sblimit` are
/// never written. The total bit width is
/// `nch · Σ_{sb < bound} nbal[sb] + Σ_{bound ≤ sb < sblimit} nbal[sb]`
/// — `bbal` in the §C.1.5.2.7 budget breakdown.
///
/// `bound` is the intensity_stereo bound: `sblimit` for mono / stereo /
/// dual_channel, the `mode_extension` bound (`{4, 8, 12, 16}`) clamped
/// to `sblimit` for joint_stereo. Callers that already parsed a
/// [`FrameHeader`] can compute it with [`layer2_stereo_bound`].
///
/// The function validates:
/// * `nch ∈ {1, 2}`.
/// * `bound ≤ table.sblimit()`.
/// * For `nch == 1`, `bound == sblimit` (a mono frame has no shared
///   upper band).
/// * Every `alloc[ch][sb]` fits in `nbal[sb]` bits and either is `0` or
///   resolves to a valid Table 3-B.2x quantization class (so the
///   decoder's [`AllocationTable::quant_class`] lookup will accept it).
/// * For sb in `[bound, sblimit)`, `alloc[0][sb] == alloc[1][sb]`.
/// * `alloc[ch][sb] == 0` for `sb ≥ sblimit` (a non-zero value there
///   would be silently dropped without this check).
///
/// On error nothing is written to `bw`: the function buffers all checks
/// before any `put` call.
// The §2.4.1.6 syntax is a sb-major / ch-minor double loop with sb-only
// upper-band sharing; rewriting as iterator chains hides the spec
// structure (the bound-driven branch shape is the explicit assertion).
#[allow(clippy::needless_range_loop)]
pub fn write_layer2_allocation_field(
    bw: &mut BitWriter,
    table: &AllocationTable,
    alloc: &Layer2Allocation,
    nch: usize,
    bound: usize,
) -> Result<(), Layer2AllocationFieldError> {
    if !(nch == 1 || nch == 2) {
        return Err(Layer2AllocationFieldError::UnsupportedChannelCount(nch));
    }
    let sblimit = table.sblimit();
    if bound > sblimit {
        return Err(Layer2AllocationFieldError::BoundExceedsSblimit { bound, sblimit });
    }
    if nch == 1 && bound != sblimit {
        return Err(Layer2AllocationFieldError::MonoBoundBelowSblimit { bound, sblimit });
    }

    // Pre-flight: validate every cell before writing a single bit.
    // Low band (per-channel allocations).
    for sb in 0..bound {
        let nbal = table.nbal(sb);
        let max = 1u8 << nbal;
        for ch in 0..nch {
            let a = alloc[ch][sb];
            if a >= max {
                return Err(Layer2AllocationFieldError::InvalidAllocationCode {
                    channel: ch,
                    subband: sb,
                    allocation: a,
                });
            }
            if a != 0 && table.quant_class(sb, a).is_none() {
                return Err(Layer2AllocationFieldError::InvalidAllocationCode {
                    channel: ch,
                    subband: sb,
                    allocation: a,
                });
            }
        }
    }
    // Upper band (shared allocations).
    for sb in bound..sblimit {
        let nbal = table.nbal(sb);
        let max = 1u8 << nbal;
        // nch is in {1, 2} and bound == sblimit for nch == 1, so this
        // upper-band loop only ever runs for nch == 2.
        let a0 = alloc[0][sb];
        let a1 = alloc[1][sb];
        if a0 != a1 {
            return Err(Layer2AllocationFieldError::UpperBandChannelsDisagree {
                subband: sb,
                left: a0,
                right: a1,
            });
        }
        if a0 >= max {
            return Err(Layer2AllocationFieldError::InvalidAllocationCode {
                channel: 0,
                subband: sb,
                allocation: a0,
            });
        }
        if a0 != 0 && table.quant_class(sb, a0).is_none() {
            return Err(Layer2AllocationFieldError::InvalidAllocationCode {
                channel: 0,
                subband: sb,
                allocation: a0,
            });
        }
    }
    // Subbands at or above sblimit must not carry an allocation. The
    // §2.4.1.6 syntax never writes a bit for those, so a non-zero entry
    // would be silently dropped — refuse it.
    for sb in sblimit..SUBBANDS {
        for ch in 0..nch {
            if alloc[ch][sb] != 0 {
                return Err(Layer2AllocationFieldError::NonZeroAllocationAboveSblimit {
                    subband: sb,
                    sblimit,
                    channel: ch,
                });
            }
        }
    }

    // Pre-flight passed: emit the bits.
    for sb in 0..bound {
        let nbal = table.nbal(sb);
        for ch in 0..nch {
            bw.put(alloc[ch][sb] as u32, nbal);
        }
    }
    for sb in bound..sblimit {
        let nbal = table.nbal(sb);
        bw.put(alloc[0][sb] as u32, nbal);
    }
    Ok(())
}

/// The §2.4.1.6 scfsi-and-scalefactor write input: per-(ch, sb) `scfsi`
/// 2-bit codes (one per non-zero allocation), and per-(ch, sb, part) the
/// three 6-bit Table 3-B.1 scalefactor indices the decoder reads
/// according to the `scfsi` schedule.
///
/// `scfsi[ch][sb]` is meaningful only when `alloc[ch][sb] != 0`; the
/// writer ignores its value otherwise. Likewise `scalefactor_indices[ch][sb]`
/// is only emitted for non-zero allocations. The three parts cover the
/// three §2.4.3.3.2 scalefactor groups (granules 0..4, 4..8, 8..12).
///
/// `scfsi` selects the §2.4.2.6 emission schedule:
///
/// * `0b00`: three separate scalefactors are written (parts 0, 1, 2).
/// * `0b01`: two scalefactors written; `scalefactor_indices[ch][sb][0]`
///   then `scalefactor_indices[ch][sb][2]`. Caller must set parts 0
///   and 1 equal (one value covers both).
/// * `0b10`: one scalefactor written
///   (`scalefactor_indices[ch][sb][0]`). Caller must set all three
///   parts equal.
/// * `0b11`: two scalefactors written;
///   `scalefactor_indices[ch][sb][0]` then
///   `scalefactor_indices[ch][sb][1]`. Caller must set parts 1 and 2
///   equal (one value covers both).
#[derive(Debug, Clone, Copy)]
pub struct Layer2ScalefactorFieldInput {
    /// `scfsi[ch][sb]`: the 2-bit §2.4.1.6 SCFSI code for each
    /// allocated (ch, sb).
    pub scfsi: [[u8; SUBBANDS]; 2],
    /// `scalefactor_indices[ch][sb][part]`: the 6-bit Table 3-B.1
    /// indices for parts 0/1/2 of each allocated (ch, sb).
    pub scalefactor_indices: [[[u8; 3]; SUBBANDS]; 2],
}

impl Default for Layer2ScalefactorFieldInput {
    fn default() -> Self {
        Layer2ScalefactorFieldInput {
            scfsi: [[0u8; SUBBANDS]; 2],
            scalefactor_indices: [[[0u8; 3]; SUBBANDS]; 2],
        }
    }
}

/// Errors raised by [`write_layer2_scalefactor_field`].
///
/// All shapes are validated before any bit is written; on error nothing
/// is appended to the writer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer2ScalefactorFieldError {
    /// `nch` was not `1` or `2`.
    UnsupportedChannelCount(usize),
    /// `bound > table.sblimit()`. The intensity-stereo bound cannot
    /// exceed the in-stream subband count.
    BoundExceedsSblimit {
        /// The supplied `bound`.
        bound: usize,
        /// `table.sblimit()`.
        sblimit: usize,
    },
    /// `bound < sblimit` with `nch == 1`. Mono frames must use
    /// `bound == sblimit` (no shared upper band exists).
    MonoBoundBelowSblimit {
        /// The supplied `bound`.
        bound: usize,
        /// `table.sblimit()`.
        sblimit: usize,
    },
    /// A `scfsi[ch][sb]` value did not fit in two bits (i.e. was `≥ 4`).
    InvalidScfsiCode {
        /// Channel of the offending value.
        channel: usize,
        /// Subband of the offending value.
        subband: usize,
        /// The supplied scfsi value.
        scfsi: u8,
    },
    /// A `scalefactor_indices[ch][sb][part]` value was `≥ 63`. The
    /// six-bit field encodes 0..=63, but the standard reserves `63` —
    /// conformant encoders must not emit it.
    InvalidScalefactorIndex {
        /// Channel of the offending index.
        channel: usize,
        /// Subband of the offending index.
        subband: usize,
        /// Part (0, 1 or 2) of the offending index.
        part: usize,
        /// The supplied 6-bit value.
        index: u8,
    },
    /// `scfsi == 0b01` but `scalefactor_indices[ch][sb][0] !=
    /// scalefactor_indices[ch][sb][1]`. Under that SCFSI code the
    /// decoder reads one value and replays it across parts 0 and 1, so
    /// the caller's per-part array must already have those two parts
    /// equal — otherwise the encode would silently lose information.
    ScfsiPartsInconsistent01 {
        /// Channel of the offending entry.
        channel: usize,
        /// Subband of the offending entry.
        subband: usize,
        /// `scalefactor_indices[ch][sb][0]`.
        part0: u8,
        /// `scalefactor_indices[ch][sb][1]`.
        part1: u8,
    },
    /// `scfsi == 0b10` but the three parts are not all equal. Under
    /// that SCFSI code the decoder reads one value and replays it
    /// across all three parts.
    ScfsiPartsInconsistent10 {
        /// Channel of the offending entry.
        channel: usize,
        /// Subband of the offending entry.
        subband: usize,
        /// `scalefactor_indices[ch][sb]`.
        parts: [u8; 3],
    },
    /// `scfsi == 0b11` but `scalefactor_indices[ch][sb][1] !=
    /// scalefactor_indices[ch][sb][2]`. Under that SCFSI code the
    /// decoder reads one value and replays it across parts 1 and 2.
    ScfsiPartsInconsistent11 {
        /// Channel of the offending entry.
        channel: usize,
        /// Subband of the offending entry.
        subband: usize,
        /// `scalefactor_indices[ch][sb][1]`.
        part1: u8,
        /// `scalefactor_indices[ch][sb][2]`.
        part2: u8,
    },
}

impl core::fmt::Display for Layer2ScalefactorFieldError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Layer2ScalefactorFieldError::UnsupportedChannelCount(n) => {
                write!(
                    f,
                    "Layer II scalefactor field: unsupported channel count {n}"
                )
            }
            Layer2ScalefactorFieldError::BoundExceedsSblimit { bound, sblimit } => {
                write!(
                    f,
                    "Layer II scalefactor field: bound {bound} exceeds sblimit {sblimit}"
                )
            }
            Layer2ScalefactorFieldError::MonoBoundBelowSblimit { bound, sblimit } => {
                write!(
                    f,
                    "Layer II scalefactor field: mono bound {bound} < sblimit {sblimit}"
                )
            }
            Layer2ScalefactorFieldError::InvalidScfsiCode {
                channel,
                subband,
                scfsi,
            } => {
                write!(
                    f,
                    "Layer II scalefactor field: scfsi={scfsi} at sb={subband} ch={channel} \
                     does not fit in two bits"
                )
            }
            Layer2ScalefactorFieldError::InvalidScalefactorIndex {
                channel,
                subband,
                part,
                index,
            } => {
                write!(
                    f,
                    "Layer II scalefactor field: scalefactor index {index} at sb={subband} \
                     ch={channel} part={part} is out of range (must be < 63)"
                )
            }
            Layer2ScalefactorFieldError::ScfsiPartsInconsistent01 {
                channel,
                subband,
                part0,
                part1,
            } => {
                write!(
                    f,
                    "Layer II scalefactor field: scfsi=01 at sb={subband} ch={channel} but \
                     parts 0/1 differ ({part0} vs {part1})"
                )
            }
            Layer2ScalefactorFieldError::ScfsiPartsInconsistent10 {
                channel,
                subband,
                parts,
            } => {
                write!(
                    f,
                    "Layer II scalefactor field: scfsi=10 at sb={subband} ch={channel} but \
                     parts are not all equal ({}, {}, {})",
                    parts[0], parts[1], parts[2]
                )
            }
            Layer2ScalefactorFieldError::ScfsiPartsInconsistent11 {
                channel,
                subband,
                part1,
                part2,
            } => {
                write!(
                    f,
                    "Layer II scalefactor field: scfsi=11 at sb={subband} ch={channel} but \
                     parts 1/2 differ ({part1} vs {part2})"
                )
            }
        }
    }
}

impl std::error::Error for Layer2ScalefactorFieldError {}

/// Write the §2.4.1.6 Layer II `scfsi` + `scalefactor` fields into `bw`,
/// MSB-first — the bitstream region that immediately follows the
/// `allocation` field (see [`write_layer2_allocation_field`]) and
/// precedes the §2.4.1.6 `sample` triplets.
///
/// The §2.4.1.6 syntax has two phases:
///
/// 1. **SCFSI**: for every `(ch, sb)` with `alloc[ch][sb] != 0`, write
///    a 2-bit `scfsi` code. The decoder loops sb-major / ch-minor over
///    `[0, sblimit)`, reading two bits whenever an allocation is non-
///    zero. In the intensity_stereo upper band `[bound, sblimit)` the
///    decoder copies the *allocation* across channels but still reads
///    one scfsi *per channel*, so the writer mirrors that order.
/// 2. **Scalefactors**: for every `(ch, sb)` with `alloc[ch][sb] != 0`,
///    write 1..3 six-bit Table 3-B.1 scalefactor indices per the
///    §2.4.2.6 SCFSI schedule (one for `scfsi == 0b10`, two for
///    `0b01`/`0b11`, three for `0b00`).
///
/// Pre-flight validation:
/// * `nch ∈ {1, 2}`.
/// * `bound ≤ table.sblimit()`.
/// * For `nch == 1`, `bound == sblimit`.
/// * Every scfsi value fits in two bits.
/// * Every scalefactor index is `< 63` (the spec leaves `63` undefined
///   and conformant encoders must not emit it).
/// * For each `scfsi` code, the parts the decoder will collapse to a
///   single value must already match in the caller's per-part array:
///   `0b01` → parts 0 == 1; `0b10` → all three parts equal; `0b11` →
///   parts 1 == 2. The writer refuses to silently drop information.
///
/// On error nothing is written to `bw`.
// The §2.4.1.6 syntax is a sb-major / ch-minor double loop with the
// 'first scfsi, then scalefactors' phase split; rewriting as iterator
// chains hides the spec structure.
#[allow(clippy::needless_range_loop)]
pub fn write_layer2_scalefactor_field(
    bw: &mut BitWriter,
    table: &AllocationTable,
    alloc: &Layer2Allocation,
    input: &Layer2ScalefactorFieldInput,
    nch: usize,
    bound: usize,
) -> Result<(), Layer2ScalefactorFieldError> {
    if !(nch == 1 || nch == 2) {
        return Err(Layer2ScalefactorFieldError::UnsupportedChannelCount(nch));
    }
    let sblimit = table.sblimit();
    if bound > sblimit {
        return Err(Layer2ScalefactorFieldError::BoundExceedsSblimit { bound, sblimit });
    }
    if nch == 1 && bound != sblimit {
        return Err(Layer2ScalefactorFieldError::MonoBoundBelowSblimit { bound, sblimit });
    }

    // Pre-flight: validate every cell before writing a single bit.
    for sb in 0..sblimit {
        for ch in 0..nch {
            if alloc[ch][sb] == 0 {
                continue;
            }
            let s = input.scfsi[ch][sb];
            if s >= 4 {
                return Err(Layer2ScalefactorFieldError::InvalidScfsiCode {
                    channel: ch,
                    subband: sb,
                    scfsi: s,
                });
            }
            let parts = input.scalefactor_indices[ch][sb];
            for part in 0..3 {
                if parts[part] >= 63 {
                    return Err(Layer2ScalefactorFieldError::InvalidScalefactorIndex {
                        channel: ch,
                        subband: sb,
                        part,
                        index: parts[part],
                    });
                }
            }
            match s {
                0b00 => {} // three parts: any combination is legal.
                0b01 => {
                    if parts[0] != parts[1] {
                        return Err(Layer2ScalefactorFieldError::ScfsiPartsInconsistent01 {
                            channel: ch,
                            subband: sb,
                            part0: parts[0],
                            part1: parts[1],
                        });
                    }
                }
                0b10 => {
                    if !(parts[0] == parts[1] && parts[1] == parts[2]) {
                        return Err(Layer2ScalefactorFieldError::ScfsiPartsInconsistent10 {
                            channel: ch,
                            subband: sb,
                            parts,
                        });
                    }
                }
                _ => {
                    // 0b11
                    if parts[1] != parts[2] {
                        return Err(Layer2ScalefactorFieldError::ScfsiPartsInconsistent11 {
                            channel: ch,
                            subband: sb,
                            part1: parts[1],
                            part2: parts[2],
                        });
                    }
                }
            }
        }
    }

    // Pre-flight passed: emit the SCFSI region first.
    for sb in 0..sblimit {
        for ch in 0..nch {
            if alloc[ch][sb] != 0 {
                bw.put(input.scfsi[ch][sb] as u32, 2);
            }
        }
    }

    // Then the scalefactor region, per the §2.4.2.6 SCFSI schedule.
    for sb in 0..sblimit {
        for ch in 0..nch {
            if alloc[ch][sb] == 0 {
                continue;
            }
            let parts = input.scalefactor_indices[ch][sb];
            match input.scfsi[ch][sb] {
                0b00 => {
                    bw.put(parts[0] as u32, 6);
                    bw.put(parts[1] as u32, 6);
                    bw.put(parts[2] as u32, 6);
                }
                0b01 => {
                    bw.put(parts[0] as u32, 6);
                    bw.put(parts[2] as u32, 6);
                }
                0b10 => {
                    bw.put(parts[0] as u32, 6);
                }
                _ => {
                    // 0b11
                    bw.put(parts[0] as u32, 6);
                    bw.put(parts[1] as u32, 6);
                }
            }
        }
    }

    Ok(())
}

// ============================================================================
// §2.4.1.6 / §2.4.3.3.4 Layer II SAMPLES region writer
// ============================================================================

/// Per-`(ch, sb, granule)` sample-triplet codes for the §2.4.1.6 Layer II
/// SAMPLES region.
///
/// Each entry `codes[ch][sb][gr]` is the three already-quantized
/// per-sample integers the writer emits as one triplet. They are the
/// MSB-inverted unsigned codes the §2.4.3.3.4 decoder will read; under
/// the decoder's "first bit inverted, then two's-complement fractional"
/// rule each value must lie in `[0, nlevels)` for the (ch, sb)
/// quantization class.
///
/// For a grouped class (`QuantClass::grouping == true`) the three codes
/// are packed into a single `bits_per_codeword`-wide field as
/// `samplecode = codes[0] + codes[1] * N + codes[2] * N²` where
/// `N = nlevels`. For a non-grouped class the three codes are emitted
/// as three separable `bits_per_codeword`-wide fields, MSB-first.
///
/// Subbands `[bound, sblimit)` in joint_stereo mode are shared between
/// channels: the writer reads the triplet from `codes[0][sb][gr]` and
/// emits it once; the decoder mirrors it into both channels.
#[derive(Debug, Clone, Copy)]
pub struct Layer2SamplesFieldInput {
    /// `codes[ch][sb][gr]` carries one triplet `[s0, s1, s2]` of
    /// per-sample integer codes for channel `ch`, subband `sb`,
    /// syntax-granule `gr` (`gr < 12`).
    pub codes: [[[[u32; 3]; SUBBANDS]; 12]; 2],
}

impl Default for Layer2SamplesFieldInput {
    fn default() -> Self {
        Layer2SamplesFieldInput {
            codes: [[[[0u32; 3]; SUBBANDS]; 12]; 2],
        }
    }
}

/// Errors raised by [`write_layer2_samples_field`].
///
/// All shapes are validated before any bit is written; on error nothing
/// is appended to the writer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer2SamplesFieldError {
    /// `nch` was not `1` or `2`.
    UnsupportedChannelCount(usize),
    /// `bound > table.sblimit()`. The intensity-stereo bound cannot
    /// exceed the in-stream subband count.
    BoundExceedsSblimit {
        /// The supplied `bound`.
        bound: usize,
        /// `table.sblimit()`.
        sblimit: usize,
    },
    /// `bound < sblimit` with `nch == 1`. Mono frames must use
    /// `bound == sblimit` (no shared upper band exists).
    MonoBoundBelowSblimit {
        /// The supplied `bound`.
        bound: usize,
        /// `table.sblimit()`.
        sblimit: usize,
    },
    /// `alloc[ch][sb]` is non-zero but points at an invalid (`-`) slot
    /// in the per-subband row of the chosen Tables 3-B.2x.
    InvalidAllocationCode {
        /// Channel of the offending allocation.
        channel: usize,
        /// Subband of the offending allocation.
        subband: usize,
        /// The supplied allocation index.
        allocation: u8,
    },
    /// A sample code did not fit in `[0, nlevels)` for the quantization
    /// class resolved at `(sb, alloc[ch][sb])`. The decoder's degrouping
    /// arithmetic and MSB-inversion rule require every code to be a
    /// valid `nlevels`-level value.
    SampleCodeOutOfRange {
        /// Channel of the offending code.
        channel: usize,
        /// Subband of the offending code.
        subband: usize,
        /// Syntax-granule of the offending code (`0..=11`).
        granule: usize,
        /// Sample-within-triplet of the offending code (`0..=2`).
        sample: usize,
        /// The supplied code value.
        code: u32,
        /// The `nlevels` of the (ch, sb) quantization class.
        nlevels: u16,
    },
}

impl core::fmt::Display for Layer2SamplesFieldError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Layer2SamplesFieldError::UnsupportedChannelCount(n) => {
                write!(f, "Layer II samples field: unsupported channel count {n}")
            }
            Layer2SamplesFieldError::BoundExceedsSblimit { bound, sblimit } => {
                write!(
                    f,
                    "Layer II samples field: bound {bound} exceeds sblimit {sblimit}"
                )
            }
            Layer2SamplesFieldError::MonoBoundBelowSblimit { bound, sblimit } => {
                write!(
                    f,
                    "Layer II samples field: mono bound {bound} < sblimit {sblimit}"
                )
            }
            Layer2SamplesFieldError::InvalidAllocationCode {
                channel,
                subband,
                allocation,
            } => {
                write!(
                    f,
                    "Layer II samples field: allocation {allocation} at sb={subband} \
                     ch={channel} is not a legal index in the Tables 3-B.2x row"
                )
            }
            Layer2SamplesFieldError::SampleCodeOutOfRange {
                channel,
                subband,
                granule,
                sample,
                code,
                nlevels,
            } => {
                write!(
                    f,
                    "Layer II samples field: code {code} at sb={subband} ch={channel} \
                     gr={granule} s={sample} is out of [0, {nlevels})"
                )
            }
        }
    }
}

impl std::error::Error for Layer2SamplesFieldError {}

/// Write the §2.4.1.6 Layer II SAMPLES region into `bw`, MSB-first —
/// the bitstream region that immediately follows the SCFSI +
/// scalefactor region emitted by [`write_layer2_scalefactor_field`].
///
/// The §2.4.1.6 syntax for this region is the outer
/// `for (gr=0; gr<12; gr++)` loop the decoder mirrors in
/// [`crate::decode_layer2::decode_layer2_audio_data`]: for each of the
/// 12 syntax-granules, walk the low band `[0, bound)` per channel and
/// emit one triplet per `(ch, sb)`, then walk the shared upper band
/// `[bound, sblimit)` and emit one triplet per `sb` (mirrored into
/// both channels by the decoder).
///
/// Each triplet is emitted via the per-subband [`QuantClass`] resolved
/// from `(sb, alloc[ch][sb])` against the per-frame [`AllocationTable`]:
///
/// * If `class.grouping == true`, write a single `bits_per_codeword`-bit
///   field whose value is `codes[0] + codes[1]·N + codes[2]·N²` where
///   `N = class.nlevels`. The decoder's degrouping arithmetic
///   (`for i in 0..3: s[i] = c % N; c /= N`) recovers `codes[0..3]`
///   exactly.
/// * If `class.grouping == false`, write three separable
///   `bits_per_codeword`-bit fields carrying `codes[0]`, `codes[1]`,
///   `codes[2]` in order.
///
/// In either case each code is a *raw* unsigned `nlevels`-level value
/// (i.e. the §2.4.3.3.4 "first bit has to be inverted" pre-image). A
/// roundtrip through the §2.4.3.3.4 decoder recovers the same triplet
/// of codes the encoder wrote.
///
/// Subbands with `alloc[ch][sb] == 0` emit zero bits — they are
/// silenced by the §2.4.3.3.5 "if a subband has no bits allocated to
/// it, the samples in that subband are set to zero" rule.
///
/// Subbands `[sblimit, 32)` are silently skipped (they cannot be
/// allocated by the §2.4.1.6 syntax).
///
/// Pre-flight validation:
/// * `nch ∈ {1, 2}`.
/// * `bound ≤ table.sblimit()`.
/// * For `nch == 1`, `bound == sblimit`.
/// * Every non-zero `alloc[ch][sb]` resolves to a legal [`QuantClass`]
///   via [`AllocationTable::quant_class`].
/// * Every emitted sample code is in `[0, nlevels)` for the resolved
///   quantization class.
///
/// On error nothing is written to `bw`.
// The §2.4.1.6 SAMPLES region is the same gr-major / sb-major /
// ch-minor triple loop the decoder uses, plus the per-triplet
// grouped-vs-separable code-emit split. Index-based loops are the
// faithful expression of the spec's nested syntax.
#[allow(clippy::needless_range_loop)]
pub fn write_layer2_samples_field(
    bw: &mut BitWriter,
    table: &AllocationTable,
    alloc: &Layer2Allocation,
    input: &Layer2SamplesFieldInput,
    nch: usize,
    bound: usize,
) -> Result<(), Layer2SamplesFieldError> {
    use crate::decode_layer2::SYNTAX_GRANULES;

    if !(nch == 1 || nch == 2) {
        return Err(Layer2SamplesFieldError::UnsupportedChannelCount(nch));
    }
    let sblimit = table.sblimit();
    if bound > sblimit {
        return Err(Layer2SamplesFieldError::BoundExceedsSblimit { bound, sblimit });
    }
    if nch == 1 && bound != sblimit {
        return Err(Layer2SamplesFieldError::MonoBoundBelowSblimit { bound, sblimit });
    }

    // Pre-flight: resolve every (ch, sb) quant class and validate every
    // code, before any bit is written. A failed allocation lookup is a
    // typed `InvalidAllocationCode`; an out-of-range code is a typed
    // `SampleCodeOutOfRange`.
    for sb in 0..sblimit {
        // Determine which channels need pre-flighting for this subband.
        // Low band: per-channel allocations. Upper band: shared (use
        // channel 0 only — that is where the encoder reads the triplet
        // from).
        let upper_band = sb >= bound;
        for ch in 0..nch {
            if upper_band && ch != 0 {
                // Upper-band cells are sourced from channel 0 only; the
                // decoder mirrors the single triplet into both channels.
                continue;
            }
            let a = alloc[ch][sb];
            if a == 0 {
                continue;
            }
            let class =
                table
                    .quant_class(sb, a)
                    .ok_or(Layer2SamplesFieldError::InvalidAllocationCode {
                        channel: ch,
                        subband: sb,
                        allocation: a,
                    })?;
            let n = class.nlevels as u32;
            for gr in 0..SYNTAX_GRANULES {
                let triplet = &input.codes[ch][gr][sb];
                for s in 0..3 {
                    if triplet[s] >= n {
                        return Err(Layer2SamplesFieldError::SampleCodeOutOfRange {
                            channel: ch,
                            subband: sb,
                            granule: gr,
                            sample: s,
                            code: triplet[s],
                            nlevels: class.nlevels,
                        });
                    }
                }
            }
        }
    }

    // Pre-flight passed: emit the region.
    for gr in 0..SYNTAX_GRANULES {
        // Low band [0, bound): per-channel triplet per (ch, sb).
        for sb in 0..bound {
            for ch in 0..nch {
                emit_layer2_triplet(bw, table, alloc[ch][sb], &input.codes[ch][gr][sb], sb);
            }
        }
        // Upper band [bound, sblimit): one shared triplet per sb,
        // sourced from channel 0; the decoder mirrors into both.
        for sb in bound..sblimit {
            emit_layer2_triplet(bw, table, alloc[0][sb], &input.codes[0][gr][sb], sb);
        }
    }

    Ok(())
}

/// Emit one §2.4.1.6 triplet for the given `(sb, alloc)` cell, per the
/// per-subband [`QuantClass`] grouping flag. Pre-flight validation in
/// [`write_layer2_samples_field`] has already ensured every code fits.
fn emit_layer2_triplet(
    bw: &mut BitWriter,
    table: &AllocationTable,
    alloc: u8,
    codes: &[u32; 3],
    sb: usize,
) {
    if alloc == 0 {
        return;
    }
    // Pre-flight guarantees the lookup succeeds.
    let class = table
        .quant_class(sb, alloc)
        .expect("pre-flight validated allocation");
    if class.grouping {
        // §2.4.3.3.4 inverse-degroup: pack three nlevels-level codes
        // into a single bits_per_codeword-wide field, low code first
        // (so the decoder's `c % N` step recovers `codes[0]`).
        let n = class.nlevels as u32;
        let mut packed = codes[0];
        packed = packed.wrapping_add(codes[1].wrapping_mul(n));
        packed = packed.wrapping_add(codes[2].wrapping_mul(n.wrapping_mul(n)));
        bw.put(packed, class.bits_per_codeword);
    } else {
        // Three separable sample fields, MSB-first per the §2.4.1.6
        // sample[ch][sb][3*gr+s] inner loop.
        let nb = class.bits_per_codeword;
        bw.put(codes[0], nb);
        bw.put(codes[1], nb);
        bw.put(codes[2], nb);
    }
}

/// The §2.4.1.6 intensity_stereo bound for a Layer II frame: the first
/// subband whose `allocation` field is shared between channels.
///
/// `joint_stereo` frames read the bound from `mode_extension`
/// (§2.4.2.3: `{4, 8, 12, 16}`); every other [`Mode`] uses `sblimit`
/// (no upper-band sharing). The result is clamped to `sblimit` so a
/// joint_stereo header whose `mode_extension` bound exceeds the
/// frame's `sblimit` collapses to "no shared band".
///
/// `sblimit` must come from the §2.4.1.6 allocation table for `header`
/// — typically `crate::tables_layer2::layer2_bit_allocation_table(header).sblimit()`.
pub fn layer2_stereo_bound(header: &FrameHeader, sblimit: usize) -> usize {
    let raw = match header.mode {
        Mode::JointStereo => header.mode_extension.bound() as usize,
        _ => SUBBANDS,
    };
    raw.min(sblimit)
}

// ---- Layer II per-sample quantization (§C.1.5.2 / §2.4.3.3.4) ---

/// Quantize one Layer II sub-band sample value into a raw `nlevels`-level
/// code for a given quantization class (§C.1.5.2 quantization step,
/// inverse of the §2.4.3.3.4 decoder requantization formula
/// `s'' = C · (s''' + D)`).
///
/// * `value` is the analysed sub-band sample `s'`.
/// * `scf` is the chosen Table 3-B.1 scalefactor multiplier for the
///   sample's `(ch, sb, part)` cell.
/// * `class` is the per-`(sb, alloc[ch][sb])` quantization class
///   resolved off the per-frame [`AllocationTable`].
///
/// The function inverts the §2.4.3.3.4 formula step-by-step:
///
/// 1. Normalise `X = value / scf`.
/// 2. Recover the spec's `s''' = X / C − D` (the two's-complement
///    fractional pre-image of the decoder's linear formula).
/// 3. Scale to a signed `bits_per_sample`-bit integer
///    `signed = round(s''' · 2^(nb−1))`.
/// 4. Reinterpret as unsigned `inverted = signed mod 2^nb` and apply the
///    §2.4.3.3.4 MSB-inversion `code = inverted XOR (1 << (nb−1))`.
/// 5. Clamp the result to `[0, nlevels)` so the §2.4.1.6 writer's
///    pre-flight always accepts it (the §2.4.3.3.4 inverse only uses
///    `nlevels` of the `2^bits_per_sample` codes; the extra ones the
///    raw arithmetic could pick are forbidden by the spec).
///
/// Feeding the returned code through the decoder's
/// [`crate::decode_layer2::decode_layer2_audio_data`] path
/// (after writing it through [`write_layer2_samples_field`])
/// recovers a sample value within one quantizer step of the chosen
/// scalefactor's grid — exactly the §C.1.5.2 reconstruction error
/// behaviour.
pub fn quantize_layer2_sample(value: f64, scf: f64, class: &QuantClass) -> u32 {
    let nb = class.bits_per_sample();
    debug_assert!(
        (2..=16).contains(&nb),
        "Layer II per-sample width 2..=16 per Table 3-B.4"
    );
    let nlevels = class.nlevels as u32;
    let msb: u32 = 1u32 << (nb - 1);
    let two_nb: u64 = 1u64 << nb;

    // §2.4.3.3.4 step backward: target s'' (the decoder's linear-formula
    // output) IS `value/scf` once the scalefactor rescale is undone, so
    // s''' = (value/scf) / C − D. The half-open `[-1, +1−2^(−(nb−1)))`
    // range is the natural domain of `s'''`; any out-of-range input gets
    // clamped to the closest representable two's-complement signed
    // integer below.
    let x = value / scf;
    let s_triple_prime = x / class.c - class.d;
    let scaled = (s_triple_prime * (msb as f64)).round();

    // Clamp `signed` to the nb-bit two's-complement range
    // [-2^(nb-1), 2^(nb-1) - 1] before the unsigned conversion.
    let lo = -(msb as i64);
    let hi = (msb as i64) - 1;
    let signed = (scaled as i64).clamp(lo, hi);

    // §2.4.3.3.4 "the first bit of each of the three codes has to be
    // inverted" — read backward: the unsigned `inverted` representation
    // of `signed` is `(signed mod 2^nb)`; XOR'ing the MSB recovers the
    // raw bitstream code.
    let inverted = if signed < 0 {
        (signed + two_nb as i64) as u32
    } else {
        signed as u32
    };
    let raw_code = inverted ^ msb;

    // Only the first `nlevels` of the `2^nb` codes are legal for a
    // class with `nlevels < 2^nb` (the spec lists exactly `nlevels`
    // quantizer steps); the §2.4.1.6 writer rejects anything in the
    // forbidden tail. Clamping to `nlevels - 1` keeps the encoder
    // consistent with the writer and matches the symmetric `±1` clip
    // the spec implies for over-range PCM.
    raw_code.min(nlevels - 1)
}

// ---- Layer II top-level frame encoder (§C.1.5.2 wiring) --------

/// Errors raised while encoding a Layer II frame at the top level
/// ([`encode_layer2_frame`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer2EncodeError {
    /// A Layer II frame requires `1` or `2` channels; everything else is
    /// out of the §2.4.2.3 syntax.
    UnsupportedChannelCount(usize),
    /// The header parameters do not represent a valid Layer II header
    /// (unsupported sampling frequency or off-ladder bitrate).
    Header(Layer2HeaderError),
    /// The header's channel mode disagrees with the supplied per-channel
    /// sub-band data (mono header with stereo sub-bands or vice versa).
    ChannelModeMismatch {
        /// Channel count implied by the header.
        header_channels: usize,
        /// Channel count carried by the sub-band data.
        data_channels: usize,
    },
    /// The §2.4.2.1 frame byte count derived from header fields is too
    /// small to hold the §2.4.1.6 control regions and the per-frame
    /// allocation field (i.e. the bitrate is below the minimum the
    /// allocator can place a single non-zero allocation at).
    FrameTooSmall {
        /// The computed `(144·bitrate/Fs)` byte count.
        frame_len: usize,
        /// The header + (optional) CRC + allocation field cost in bytes.
        overhead: usize,
    },
    /// The top-level [`Mp1Layer2FrameEncoder::encode_frame`] expects
    /// exactly [`LAYER2_SAMPLES_PER_FRAME`] (= 1152) PCM samples per
    /// channel per call (§2.4.2.1).
    ///
    /// [`LAYER2_SAMPLES_PER_FRAME`]: crate::decode_layer2::LAYER2_SAMPLES_PER_FRAME
    WrongSampleCount {
        /// The per-channel sample count actually received.
        got: usize,
    },
    /// The §2.4.1.8 `ancillary_data()` payload supplied to
    /// [`encode_layer2_frame_with_ancillary`] (or
    /// [`Mp1Layer2FrameEncoder::set_pending_ancillary`]) is larger than
    /// the space remaining after the §2.4.1.6 header / CRC / allocation
    /// / scalefactor / samples regions.
    ///
    /// `space` is the byte count the encoder reserved for ancillary
    /// (`floor(144·bitrate/Fs) + padding_bit` minus the number of bytes
    /// the §2.4.1.6 audio-data regions consumed); `got` is the caller's
    /// payload length.
    AncillaryTooLarge {
        /// The §2.4.1.8 tail capacity available in the current frame.
        space: usize,
        /// The caller's ancillary payload length in bytes.
        got: usize,
    },
}

impl core::fmt::Display for Layer2EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Layer2EncodeError::UnsupportedChannelCount(n) => {
                write!(f, "Layer II encode: unsupported channel count {n}")
            }
            Layer2EncodeError::Header(h) => {
                write!(f, "Layer II encode: header: {h}")
            }
            Layer2EncodeError::ChannelModeMismatch {
                header_channels,
                data_channels,
            } => {
                write!(
                    f,
                    "Layer II encode: header mode implies {header_channels}-channel \
                     but sub-band data carries {data_channels}-channel"
                )
            }
            Layer2EncodeError::FrameTooSmall {
                frame_len,
                overhead,
            } => {
                write!(
                    f,
                    "Layer II encode: §2.4.2.1 frame is {frame_len} bytes, below the \
                     {overhead}-byte header / CRC / allocation overhead"
                )
            }
            Layer2EncodeError::WrongSampleCount { got } => {
                write!(
                    f,
                    "Layer II encode: §2.4.2.1 frame granularity is 1152 PCM samples \
                     per channel; received {got} per channel"
                )
            }
            Layer2EncodeError::AncillaryTooLarge { space, got } => {
                write!(
                    f,
                    "Layer II encode: §2.4.1.8 ancillary_data() payload {got} bytes \
                     exceeds the {space}-byte tail capacity in this frame"
                )
            }
        }
    }
}

impl std::error::Error for Layer2EncodeError {}

impl From<Layer2HeaderError> for Layer2EncodeError {
    fn from(e: Layer2HeaderError) -> Layer2EncodeError {
        Layer2EncodeError::Header(e)
    }
}

/// Encode one §2.4.1.6 Layer II frame from analysed sub-band samples,
/// wiring together the four §2.4.1.6 writers
/// ([`write_layer2_header`], [`write_layer2_allocation_field`],
/// [`write_layer2_scalefactor_field`], [`write_layer2_samples_field`])
/// behind the encoder-side scalefactor / bit-allocation / quantization
/// helpers.
///
/// `params` carries every §2.4.1.3 header field plus the optional
/// §2.4.1.4 CRC opt-in (via [`Layer2HeaderParams::has_crc`]).
/// `subbands[ch][sb][slot]` is the analysed sub-band sample matrix for
/// the frame — 36 slots per (ch, sb), the same shape
/// [`select_layer2_scalefactors`] / [`layer2_subband_peak_per_part`]
/// consume.
///
/// The encoder chooses `scfsi == 0b00` for every allocated `(ch, sb)`:
/// three independent scalefactors per (ch, sb), as the §2.4.1.6
/// "three equal parts of 12 sub-band samples" syntax permits in the
/// absence of the §C.1.5.2.5 / Table C.4 perceptual SCFSI collapse
/// (Table C.4 is rendered as a PDF image the text layer cannot extract
/// reliably; the all-`0b00` choice is the conservative bookkeeping the
/// [`allocate_bits_layer2`] budget already assumes).
///
/// Returns the §2.4.2.1 frame bytes (header + optional CRC +
/// §2.4.1.6 alloc + scfsi/scf + samples regions + zero-padded
/// §2.4.1.8 ancillary tail), exactly `floor(144 · bitrate / Fs) +
/// padding_bit` bytes long. The ancillary tail is zero-filled; see
/// [`encode_layer2_frame_with_ancillary`] for the variant that lets
/// the caller supply the §2.4.1.8 `ancillary_data()` payload.
///
/// The §2.4.3.3.4 round-trip `encode → decode` lands within one Layer II
/// quantizer step plus the §2.4.3.3 grouping rounding; the decoder's
/// [`crate::decode_layer2::decode_layer2_audio_data`] reproduces the
/// per-(ch, sb, slot) sub-band sample to a tolerance that depends on the
/// class `nlevels` (small `nlevels` → coarser grid).
pub fn encode_layer2_frame(
    params: &Layer2HeaderParams,
    subbands: &[[[f64; 36]; SUBBANDS]; 2],
) -> Result<Vec<u8>, Layer2EncodeError> {
    encode_layer2_frame_inner(params, subbands, &[])
}

/// Encode a §2.4.1.6 Layer II frame with a caller-supplied §2.4.1.8
/// `ancillary_data()` payload.
///
/// Identical to [`encode_layer2_frame`] but `ancillary` is copied into
/// the §2.4.1.8 tail that begins immediately after the §2.4.1.6
/// audio-data region (the partial trailing byte of the samples region
/// is byte-aligned by [`BitWriter::finish`] before the ancillary tail
/// is written; ancillary therefore starts on a whole-byte boundary).
/// Any §2.4.2.1 frame bytes the caller does not fill are zero-padded.
///
/// Returns [`Layer2EncodeError::AncillaryTooLarge`] when `ancillary`
/// does not fit between the audio-data tail and the §2.4.2.1 frame
/// length `floor(144·bitrate/Fs) + padding_bit`; the §2.4.3.1 CRC (if
/// enabled) is patched after the ancillary copy and is unaffected by
/// the ancillary bytes (the §2.4.3.1 protected region is header bits
/// 16…31 plus the §2.4.1.6 allocation + scfsi field).
pub fn encode_layer2_frame_with_ancillary(
    params: &Layer2HeaderParams,
    subbands: &[[[f64; 36]; SUBBANDS]; 2],
    ancillary: &[u8],
) -> Result<Vec<u8>, Layer2EncodeError> {
    encode_layer2_frame_inner(params, subbands, ancillary)
}

// The §C.1.5.2 wiring is a strict (ch, sb, gr, sample) nested walk, the
// same structure the four writers expose; an iterator rewrite would
// obscure the spec mapping.
#[allow(clippy::needless_range_loop)]
fn encode_layer2_frame_inner(
    params: &Layer2HeaderParams,
    subbands: &[[[f64; 36]; SUBBANDS]; 2],
    ancillary: &[u8],
) -> Result<Vec<u8>, Layer2EncodeError> {
    use crate::decode_layer2::{LAYER2_SAMPLES_PER_SUBBAND, SYNTAX_GRANULES};
    use crate::tables_layer2::layer2_bit_allocation_table;

    let nch = params.mode.channels() as usize;
    if !(nch == 1 || nch == 2) {
        return Err(Layer2EncodeError::UnsupportedChannelCount(nch));
    }

    // Round-trip the params through `FrameHeader::parse` to obtain the
    // typed header the §2.4.1.6 allocation-table selector consumes. The
    // pack call validates the §2.4.2.3 sampling-frequency / bitrate
    // ladder eagerly, returning `Layer2HeaderError` on bad input.
    let header_bytes = pack_layer2_header(params)?;
    let header = FrameHeader::parse(&header_bytes).expect(
        "pack_layer2_header produced a valid four-byte Layer II header; \
         FrameHeader::parse must succeed",
    );
    let table = layer2_bit_allocation_table(&header);
    let sblimit = table.sblimit();
    let bound = layer2_stereo_bound(&header, sblimit);

    // §2.4.2.1 frame byte count: floor(144·bitrate/Fs) + padding_bit.
    // The header pack validated `bitrate_kbps` already so this is in
    // range.
    let bytes = (144u32 * (params.bitrate_kbps as u32) * 1000) / params.sampling_frequency;
    let padding = if params.padding { 1 } else { 0 };
    let frame_len = (bytes + padding) as usize;
    let bbal_bits = nch * sum_nbal_per_channel(table);
    let crc_bytes = if params.has_crc { 2 } else { 0 };
    let overhead_bits = 32 + crc_bytes * 8 + bbal_bits;
    let overhead_bytes = overhead_bits.div_ceil(8);
    if frame_len < overhead_bytes {
        return Err(Layer2EncodeError::FrameTooSmall {
            frame_len,
            overhead: overhead_bytes,
        });
    }
    let budget_bits = (frame_len * 8).saturating_sub(overhead_bits);

    // §C.1.5.1.4 (per-part) scalefactor selection — populates the
    // §2.4.2.6 SCFSI part-0/1/2 array directly.
    let scf_indices = select_layer2_scalefactors(subbands, nch, sblimit);
    // §C.1.5.2.7 bit allocation — the per-frame `(ch, sb) → alloc`
    // matrix. The allocator's "peak energy" proxy is the per-subband max
    // absolute sub-band sample over the 36 slots.
    let mut peak = [[0.0f64; SUBBANDS]; 2];
    for ch in 0..nch {
        for sb in 0..sblimit {
            let mut m = 0.0f64;
            for slot in 0..LAYER2_SAMPLES_PER_SUBBAND {
                let a = subbands[ch][sb][slot].abs();
                if a > m {
                    m = a;
                }
            }
            peak[ch][sb] = m;
        }
    }
    // §2.4.1.6 intensity-stereo: in `[bound, sblimit)` both channels MUST
    // carry the same `allocation` value (the bitstream writes one shared
    // `nbal`-bit field for the pair). Pre-mirror the per-(ch, sb) peaks
    // in the upper band before running the allocator so any step the
    // allocator places lands on the same allocation for both channels —
    // the per-channel scfsi+scalefactor bookkeeping is still accounted
    // for in the allocator's worst-case overhead. Post-fix the rare
    // races where the two channels diverged anyway (both peaks equal but
    // numerical ties resolve to different (ch, sb) winners) by mirroring
    // the larger allocation into the other channel.
    if nch == 2 {
        for sb in bound..sblimit {
            let m = peak[0][sb].max(peak[1][sb]);
            peak[0][sb] = m;
            peak[1][sb] = m;
        }
    }
    let mut alloc = allocate_bits_layer2(&peak, nch, table, budget_bits);
    if nch == 2 {
        for sb in bound..sblimit {
            let shared = alloc[0][sb].max(alloc[1][sb]);
            alloc[0][sb] = shared;
            alloc[1][sb] = shared;
        }
    }

    // §C.1.5.2 quantization — build the per-(ch, gr, sb) triplet of raw
    // codes the §2.4.1.6 writer consumes. The §2.4.2.6 SCFSI schedule
    // splits the 36 slots into three 12-slot scalefactor parts (part =
    // slot / 12); the chosen scalefactor for the corresponding part
    // normalises each sample before quantization.
    let mut samples_input = Layer2SamplesFieldInput::default();
    for ch in 0..nch {
        for sb in 0..sblimit {
            // The upper band sources its triplet from channel 0; channel
            // 1's codes for `sb >= bound` are unused by the §2.4.1.6
            // writer in shared mode.
            let upper_band = sb >= bound;
            if upper_band && ch != 0 {
                continue;
            }
            let a = alloc[ch][sb];
            if a == 0 {
                continue;
            }
            let class = table.quant_class(sb, a).expect(
                "allocate_bits_layer2 only selects legal Table 3-B.2x cells; \
                 quant_class must resolve",
            );
            for gr in 0..SYNTAX_GRANULES {
                for s in 0..3 {
                    let slot = gr * 3 + s;
                    let part = slot / 12;
                    let scf = SCALEFACTORS[scf_indices[ch][sb][part] as usize];
                    let value = subbands[ch][sb][slot];
                    samples_input.codes[ch][gr][sb][s] = quantize_layer2_sample(value, scf, class);
                }
            }
        }
    }

    // §2.4.2.6 SCFSI: every allocated (ch, sb) carries `0b00` (three
    // independent scalefactors per part). Table C.4's perceptual collapse
    // is a PDF-image DOCS-GAP; the worst-case bookkeeping
    // [`allocate_bits_layer2`] already assumes is exactly this
    // three-scalefactor cost, so the budget stays sound.
    let scalefactor_input = Layer2ScalefactorFieldInput {
        scfsi: [[0u8; SUBBANDS]; 2],
        scalefactor_indices: scf_indices,
    };

    // §C.1.5.1.10-style frame assembly: write each region MSB-first,
    // patch the CRC placeholder after the §2.4.1.6 alloc + scfsi fields
    // have been emitted (those plus header bits 16…31 are the Table
    // 3-B.5 protected region for Layer II).
    let mut bw = BitWriter::new();
    write_layer2_header(&mut bw, params).map_err(Layer2EncodeError::Header)?;
    if params.has_crc {
        // Reserve the 16-bit error_check() placeholder.
        bw.put(0, 16);
    }
    write_layer2_allocation_field(&mut bw, table, &alloc, nch, bound)
        .expect("allocate_bits_layer2 + layer2_stereo_bound produce shapes the writer accepts");
    write_layer2_scalefactor_field(&mut bw, table, &alloc, &scalefactor_input, nch, bound)
        .expect("select_layer2_scalefactors emits indices < 63 and scfsi=0b00 is consistent");
    write_layer2_samples_field(&mut bw, table, &alloc, &samples_input, nch, bound).expect(
        "quantize_layer2_sample clamps codes to [0, nlevels) so the samples writer accepts them",
    );

    let mut bytes = bw.finish();
    // §2.4.1.8 ancillary_data(): the bytes between the §2.4.1.6
    // audio-data tail and the §2.4.2.1 frame length are reserved for
    // the caller's `ancillary_bit` payload (bslbf, byte-aligned because
    // `BitWriter::finish` flushed the partial samples-region byte just
    // above). When the caller supplies no ancillary the tail is
    // zero-padded; an overlong payload errors out so the caller can
    // resize before the §2.4.3.1 CRC patch fires.
    let space = frame_len.saturating_sub(bytes.len());
    if ancillary.len() > space {
        return Err(Layer2EncodeError::AncillaryTooLarge {
            space,
            got: ancillary.len(),
        });
    }
    bytes.extend_from_slice(ancillary);
    if bytes.len() < frame_len {
        bytes.resize(frame_len, 0);
    }

    // §2.4.3.1 CRC patch: walk the just-written header + alloc + scfsi
    // region with the decoder-side `compute_layer2_crc`, which sizes the
    // §2.4.1.6 protected region per the Table 3-B.2x `nbal` widths and
    // the in-stream allocation values. The CRC word lives at bytes 4..6,
    // and the allocation field starts at byte 6.
    if params.has_crc {
        use crate::decode_layer2::compute_layer2_crc;
        let crc = compute_layer2_crc(&header, &bytes[0..4], &bytes[6..])
            .expect("encoder just wrote a CRC-shaped allocation + scfsi region");
        let crc_bytes = crc.to_be_bytes();
        bytes[4] = crc_bytes[0];
        bytes[5] = crc_bytes[1];
    }

    Ok(bytes)
}

/// The MPEG audio layer the top-level [`Mp1Encoder`](crate::Mp1Encoder)
/// should emit.
///
/// The choice selects which inner encoder is driven and consequently
/// the per-frame PCM granularity:
///
/// * [`LayerSelect::LayerI`] — §2.4.1.5 frame structure, 384 PCM
///   samples per channel per frame (`12 · 32`).
/// * [`LayerSelect::LayerII`] — §2.4.1.6 frame structure, 1152 PCM
///   samples per channel per frame (`36 · 32`).
///
/// Both branches share the same six sampling-frequency / two-channel
/// surface (MPEG-1 `ID == 1` plus MPEG-2 LSF `ID == 0`); the bitrate
/// ladders differ (§2.4.2.3 Layer I vs. Layer II columns), so a
/// Layer-II encoder built from an [`EncodeParams`] whose bitrate is
/// only on the Layer I ladder will surface the inner encoder's
/// ladder-rejection error on first encode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LayerSelect {
    /// MPEG-1 Audio Layer I (§2.4.1.5). The default, preserving the
    /// historical [`EncodeParams`] behaviour before this switch existed.
    #[default]
    LayerI,
    /// MPEG-1 Audio Layer II (§2.4.1.6) — or MPEG-2 LSF Layer II per
    /// 13818-3 §2.4.2.3 when the chosen sampling frequency is on the
    /// LSF ladder (16 / 22.05 / 24 kHz).
    LayerII,
}

/// Parameters for a single Layer I (or, when [`EncodeParams::layer`] is
/// set, Layer II) encode: the target bitrate and the stream's sampling
/// frequency and channel mode.
#[derive(Debug, Clone, Copy)]
pub struct EncodeParams {
    /// Target bitrate. Must be a fixed Layer I ladder value
    /// ([`Bitrate::Fixed`]) when [`EncodeParams::layer`] is
    /// [`LayerSelect::LayerI`], or a Layer II ladder value when it is
    /// [`LayerSelect::LayerII`]. Free format is selected by setting
    /// [`EncodeParams::free_format_kbps`] instead; when that field is
    /// `Some`, this `bitrate` field is ignored.
    pub bitrate: Bitrate,
    /// When `Some(k)`, the encoder emits a **free-format** frame
    /// (§2.4.2.3 `bitrate_index == 0b0000`): the four-bit
    /// `bitrate_index` header field is written as `0b0000` and the
    /// frame is sized to a fixed, possibly off-ladder rate of `k`
    /// kbit/s via the same §2.4.2.1 slot formula
    /// `N = floor(12 · k / Fs)` the fixed ladder uses. The §2.4.3.1
    /// prose permits the bitrate to be any constant value the syntax
    /// can carry (a frame holds at most `2^9 = 512` four-byte slots
    /// for Layer I); a free-format stream must hold this rate
    /// **constant** across every frame so the decoder's §2.4.3.1
    /// next-syncword distance probe ([`detect_free_format_frame_length`](crate::header::detect_free_format_frame_length))
    /// recovers the same `N`. When set, [`EncodeParams::bitrate`] is
    /// ignored. Defaults to `None` (fixed-ladder output).
    pub free_format_kbps: Option<u16>,
    /// Sampling frequency in Hz (44100, 48000 or 32000).
    pub sampling_frequency: u32,
    /// Channel mode. The encoder writes one allocation/scalefactor/
    /// sample set per channel (no joint-stereo bound — every subband is
    /// per-channel for stereo / dual_channel).
    pub mode: Mode,
    /// When `true`, the encoder emits the optional §2.4.1.4
    /// `error_check()` field: it writes the header with `protection_bit
    /// == 0`, then writes a 16-bit CRC word computed via §2.4.3.1
    /// (`G(X) = X^16 + X^15 + X^2 + 1`, initial state `0xFFFF`) over
    /// the Annex B Table 3-B.5 Layer I protected fields (header bits
    /// 16…31 plus the bit-allocation field).
    ///
    /// Defaults to `false` (no CRC, `protection_bit == 1`), matching
    /// the encoder's behaviour before optional CRC emission was added.
    /// When `true`, the 16-bit CRC is deducted from the per-frame audio
    /// data budget (`adb` of §C.1.5.1.6) so the slot count remains the
    /// §2.4.2.1 `N = floor(12 · bitrate / Fs) + padding` value.
    pub emit_crc: bool,
    /// Which MPEG audio layer the top-level
    /// [`Mp1Encoder`](crate::Mp1Encoder) should emit. Defaults to
    /// [`LayerSelect::LayerI`], preserving byte-for-byte compatibility
    /// with the encoder's pre-switch behaviour.
    ///
    /// The lower-level [`Mp1FrameEncoder`] always emits Layer I and the
    /// lower-level [`Mp1Layer2FrameEncoder`] always emits Layer II; this
    /// field is only consulted by the top-level
    /// [`Mp1Encoder`](crate::Mp1Encoder) wrapper that adapts an
    /// [`oxideav_core::Encoder`] to either inner encoder.
    pub layer: LayerSelect,
}

impl EncodeParams {
    /// A new [`EncodeParams`] with `emit_crc = false` and `layer =
    /// LayerI`, the encoder's historical defaults (the optional §2.4.1.4
    /// CRC `error_check()` is **not** written, `protection_bit == 1`,
    /// and the top-level encoder produces Layer I output).
    ///
    /// `bitrate` is a fixed Layer I ladder value (free format is not
    /// produced); `sampling_frequency` must be one of the six Layer I
    /// sampling rates (MPEG-1 32 / 44.1 / 48 kHz from 11172-3 §2.4.2.3,
    /// or MPEG-2 LSF 16 / 22.05 / 24 kHz from 13818-3 §2.4.2.3); `mode`
    /// is the per-channel mode written into the header `mode` field.
    pub fn new(bitrate: Bitrate, sampling_frequency: u32, mode: Mode) -> EncodeParams {
        EncodeParams {
            bitrate,
            free_format_kbps: None,
            sampling_frequency,
            mode,
            emit_crc: false,
            layer: LayerSelect::LayerI,
        }
    }

    /// Builder: select **free-format** Layer I output (§2.4.2.3
    /// `bitrate_index == 0b0000`) at the fixed, possibly off-ladder
    /// rate `kbps`. See [`EncodeParams::free_format_kbps`]. When set,
    /// the [`EncodeParams::bitrate`] field is ignored and the encoder
    /// writes `0b0000` into the header's four-bit `bitrate_index`
    /// field, sizing the frame to `N = floor(12 · kbps / Fs)` slots.
    pub fn with_free_format(mut self, kbps: u16) -> EncodeParams {
        self.free_format_kbps = Some(kbps);
        self
    }

    /// Builder: select whether the encoder emits the optional §2.4.1.4
    /// `error_check()` CRC field (see [`EncodeParams::emit_crc`]).
    pub fn with_emit_crc(mut self, emit: bool) -> EncodeParams {
        self.emit_crc = emit;
        self
    }

    /// Builder: select which MPEG audio layer the top-level
    /// [`Mp1Encoder`](crate::Mp1Encoder) should emit. Layer I (the
    /// default) drives [`Mp1FrameEncoder`] and consumes 384 PCM samples
    /// per channel per frame; Layer II drives [`Mp1Layer2FrameEncoder`]
    /// and consumes 1152.
    pub fn with_layer(mut self, layer: LayerSelect) -> EncodeParams {
        self.layer = layer;
        self
    }
}

/// Errors raised while encoding a Layer I frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// The PCM input did not carry exactly 384 samples per channel.
    WrongSampleCount {
        /// The number of samples per channel that were supplied.
        got: usize,
    },
    /// The requested bitrate is not a fixed Layer I ladder value (free
    /// format / forbidden), so the frame length is undetermined.
    UnsupportedBitrate,
    /// The sampling frequency is not one of 44100 / 48000 / 32000 Hz.
    UnsupportedSamplingFrequency(u32),
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EncodeError::WrongSampleCount { got } => {
                write!(f, "expected 384 samples per channel, got {got}")
            }
            EncodeError::UnsupportedBitrate => {
                write!(f, "bitrate must be a fixed Layer I ladder value")
            }
            EncodeError::UnsupportedSamplingFrequency(hz) => {
                write!(f, "unsupported sampling frequency {hz} Hz")
            }
        }
    }
}

impl std::error::Error for EncodeError {}

/// The §2.4.2.3 `sampling_frequency` field code for a frequency, plus
/// the `ID` bit (`1` for MPEG-1, `0` for MPEG-2 LSF), or `None` if the
/// frequency is on neither Layer I sampling-frequency table.
///
/// MPEG-1 (11172-3 §2.4.2.3, `ID == 1`): 44.1 / 48 / 32 kHz maps to
/// `0b00 / 0b01 / 0b10`. MPEG-2 LSF (13818-3 §2.4.2.3, `ID == 0`):
/// 22.05 / 24 / 16 kHz maps to `0b00 / 0b01 / 0b10`.
fn sampling_code(hz: u32) -> Option<(u8, u8)> {
    match hz {
        // MPEG-1 (11172-3 §2.4.2.3).
        44_100 => Some((1, 0b00)),
        48_000 => Some((1, 0b01)),
        32_000 => Some((1, 0b10)),
        // MPEG-2 LSF (13818-3 §2.4.2.3).
        22_050 => Some((0, 0b00)),
        24_000 => Some((0, 0b01)),
        16_000 => Some((0, 0b10)),
        _ => None,
    }
}

/// The §2.4.2.3 `bitrate_index` for a fixed Layer I ladder value in
/// kbit/s, given the `ID` bit (`1` → MPEG-1, `0` → MPEG-2 LSF), or
/// `None` if the value is not on that ladder.
///
/// The MPEG-1 ladder (11172-3 §2.4.2.3) is `32 / 64 / 96 / 128 / 160 /
/// 192 / 224 / 256 / 288 / 320 / 352 / 384 / 416 / 448`. The LSF
/// ladder (13818-3 §2.4.2.3) is `32 / 48 / 56 / 64 / 80 / 96 / 112 /
/// 128 / 144 / 160 / 176 / 192 / 224 / 256`.
fn bitrate_index(kbps: u16, id_bit: u8) -> Option<u8> {
    const LADDER_MPEG1: [u16; 14] = [
        32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448,
    ];
    const LADDER_LSF: [u16; 14] = [
        32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256,
    ];
    let ladder: &[u16] = if id_bit == 1 {
        &LADDER_MPEG1
    } else {
        &LADDER_LSF
    };
    ladder
        .iter()
        .position(|&k| k == kbps)
        .map(|p| (p + 1) as u8)
}

/// A stateful Layer I encoder: holds one [`AnalysisFilter`] per channel
/// so the windowing history carries across frames, exactly as the
/// decoder's synthesis history does.
#[derive(Debug)]
pub struct Mp1FrameEncoder {
    params: EncodeParams,
    filters: Vec<AnalysisFilter>,
}

impl Mp1FrameEncoder {
    /// Build an encoder for the given parameters with fresh (zeroed)
    /// analysis history.
    pub fn new(params: EncodeParams) -> Mp1FrameEncoder {
        let nch = params.mode.channels() as usize;
        Mp1FrameEncoder {
            params,
            filters: (0..nch).map(|_| AnalysisFilter::new()).collect(),
        }
    }

    /// Zero the analysis history (for a seek / stream restart).
    pub fn reset(&mut self) {
        for f in &mut self.filters {
            f.reset();
        }
    }

    /// The channel count implied by the encode mode.
    pub fn channels(&self) -> usize {
        self.params.mode.channels() as usize
    }

    /// Encode one frame of interleaved `f64` PCM in `[-1, 1)` to Layer I
    /// frame bytes.
    ///
    /// `pcm` is interleaved: `pcm[sample*nch + ch]`, with exactly 384
    /// samples per channel (`pcm.len() == 384 * nch`). When the encoder's
    /// [`EncodeParams::emit_crc`] is `false` (the default), no CRC is
    /// written (`protection_bit == 1`); when `true`, a 16-bit CRC word
    /// (§2.4.1.4 / §2.4.3.1) is computed over the Annex B Table 3-B.5
    /// protected fields (header bits 16…31 + bit-allocation field) and
    /// inserted between the header and the audio data, with
    /// `protection_bit == 0`.
    // The figure-C.4 / §C.1.5.1 encode body is a nested set of
    // index-driven loops over (slot, channel, subband) that walk the
    // §2.4.1.5 packed-bitstream order exactly as the spec lists it.
    // Rewriting them as iterator zips of two simultaneously-borrowed
    // arrays would obscure the §C.1.5.1 structure without changing
    // behaviour.
    #[allow(clippy::needless_range_loop)]
    pub fn encode_frame(&mut self, pcm: &[f64]) -> Result<Vec<u8>, EncodeError> {
        let nch = self.channels();
        let per_ch = SAMPLES_PER_SUBBAND * SUBBANDS; // 384
        if pcm.len() != per_ch * nch {
            return Err(EncodeError::WrongSampleCount {
                got: pcm.len() / nch.max(1),
            });
        }
        let (id_bit, samp_code) = sampling_code(self.params.sampling_frequency).ok_or(
            EncodeError::UnsupportedSamplingFrequency(self.params.sampling_frequency),
        )?;
        // Free format (§2.4.2.3 `bitrate_index == 0b0000`) is selected
        // by `free_format_kbps`: the header carries `0b0000` while the
        // frame is sized to the fixed, possibly off-ladder target rate.
        // Otherwise the fixed ladder rate must resolve to a valid
        // `bitrate_index`.
        let (bitrate_kbps, brate_idx) = match self.params.free_format_kbps {
            Some(k) => {
                // §2.4.3.1: free format carries a constant bitrate, but
                // a frame holds at most 2^9 = 512 four-byte slots
                // (`N` is read back from a 9-bit-significant distance
                // by the decoder probe). Reject a target whose slot
                // count would not fit, and reject a zero target (no
                // audio-data budget at all).
                let slots = (12 * k as u32 * 1000) / self.params.sampling_frequency;
                if k == 0 || slots == 0 || slots > 512 {
                    return Err(EncodeError::UnsupportedBitrate);
                }
                (k, 0u8)
            }
            None => {
                let bitrate_kbps = match self.params.bitrate {
                    Bitrate::Fixed(k) => k,
                    _ => return Err(EncodeError::UnsupportedBitrate),
                };
                let brate_idx =
                    bitrate_index(bitrate_kbps, id_bit).ok_or(EncodeError::UnsupportedBitrate)?;
                (bitrate_kbps, brate_idx)
            }
        };

        // --- 1. Analysis: 12 slots * 32 subband samples per channel ---
        // subbands[ch][sb][slot]
        let mut subbands = [[[0.0f64; SAMPLES_PER_SUBBAND]; SUBBANDS]; 2];
        for slot in 0..SAMPLES_PER_SUBBAND {
            for ch in 0..nch {
                let mut input = [0.0f64; SUBBANDS];
                for (j, v) in input.iter_mut().enumerate() {
                    let sample_idx = slot * SUBBANDS + j;
                    *v = pcm[sample_idx * nch + ch];
                }
                let s = self.filters[ch].analyze(&input);
                for sb in 0..SUBBANDS {
                    subbands[ch][sb][slot] = s[sb];
                }
            }
        }

        // --- 2. Scalefactor selection (§C.1.5.1.4) ---
        // peak[ch][sb] is the max abs subband sample over the 12 slots.
        let mut peak = [[0.0f64; SUBBANDS]; 2];
        let mut scf_index = [[0u8; SUBBANDS]; 2];
        for ch in 0..nch {
            for sb in 0..SUBBANDS {
                let mut m = 0.0f64;
                for slot in 0..SAMPLES_PER_SUBBAND {
                    let a = subbands[ch][sb][slot].abs();
                    if a > m {
                        m = a;
                    }
                }
                peak[ch][sb] = m;
                scf_index[ch][sb] = select_scalefactor(m);
            }
        }

        // --- 3. Bit allocation (§C.1.5.1.6) ---
        let has_crc = self.params.emit_crc;
        let budget = frame_payload_bits(
            bitrate_kbps as u32,
            self.params.sampling_frequency,
            has_crc,
            nch,
        );
        let alloc = allocate_bits(&peak, nch, budget);

        // --- 4. Frame assembly (§C.1.5.1.10 / figure C.2) ---
        let mut bw = BitWriter::new();
        // Header (§2.4.1.3), MSB-first.
        bw.put(0xFFF, 12); // syncword
        bw.put(id_bit as u32, 1); // ID: 1 = MPEG-1, 0 = MPEG-2 LSF (13818-3 §2.4.2.3)
        bw.put(0b11, 2); // layer I
                         // §2.4.2.3: protection_bit '0' indicates redundancy (CRC)
                         // *has* been added; '1' indicates no redundancy. The encoder
                         // writes '0' iff `emit_crc` was requested.
        bw.put(if has_crc { 0 } else { 1 }, 1);
        bw.put(brate_idx as u32, 4); // bitrate_index
        bw.put(samp_code as u32, 2); // sampling_frequency
        bw.put(0, 1); // padding_bit = 0
        bw.put(0, 1); // private_bit
        bw.put(self.mode_bits(), 2); // mode
        bw.put(0, 2); // mode_extension
        bw.put(0, 1); // copyright
        bw.put(1, 1); // original/home
        bw.put(0, 2); // emphasis = none

        // When emitting a CRC, leave a 16-bit placeholder immediately
        // after the header. The header is exactly 32 bits / 4 bytes, so
        // the placeholder occupies bytes 4..6 of the eventual frame.
        // §2.4.1.4: the `error_check()` field directly follows the
        // header when `protection_bit == 0`.
        if has_crc {
            bw.put(0, 16);
        }

        // ALLOC: 4 bits per (sb, ch) — §2.4.1.5 reads sb-major, ch-minor.
        for sb in 0..SUBBANDS {
            for ch in 0..nch {
                bw.put(allocation_code(alloc[ch][sb]) as u32, 4);
            }
        }
        // SCALEFACTORS: 6 bits for each allocated (sb, ch), sb-major.
        for sb in 0..SUBBANDS {
            for ch in 0..nch {
                if alloc[ch][sb] != 0 {
                    bw.put(scf_index[ch][sb] as u32, 6);
                }
            }
        }
        // SAMPLES: 12 passes; each pass writes sb-major, ch-minor.
        for slot in 0..SAMPLES_PER_SUBBAND {
            for sb in 0..SUBBANDS {
                for ch in 0..nch {
                    let nb = alloc[ch][sb];
                    if nb != 0 {
                        let scf = SCALEFACTORS[scf_index[ch][sb] as usize];
                        let code = quantize(subbands[ch][sb][slot], scf, nb);
                        bw.put(code, nb);
                    }
                }
            }
        }

        // Pad the partial trailing byte, then extend to the full slot
        // count (§2.4.2.1: a frame is an integer number of 4-byte slots;
        // the remaining bytes are the ANC region, written as zero).
        let mut bytes = bw.finish();
        let slots = (12 * bitrate_kbps as u32 * 1000) / self.params.sampling_frequency;
        let frame_len = slots as usize * 4;
        if bytes.len() < frame_len {
            bytes.resize(frame_len, 0);
        }

        // Patch the §2.4.1.4 CRC placeholder. The CRC covers the
        // Table 3-B.5 Layer I protected fields: header bits 16…31 plus
        // the bit-allocation field (§2.4.3.1). `FrameHeader::compute_crc`
        // walks the parsed header to size the allocation field
        // correctly, and is the exact inverse of the decoder's
        // `verify_crc` path. The allocation field lives at the start of
        // the audio-data region, which begins immediately after the
        // 16-bit CRC word at byte 6.
        if has_crc {
            // Parse our own header back to drive `compute_crc`. This is
            // self-consistent (we just wrote it) and avoids duplicating
            // the protected-field sizing logic across the codebase.
            let header_bytes = &bytes[0..4];
            let header = FrameHeader::parse(header_bytes)
                .expect("encoder produced a valid Layer I header; FrameHeader::parse must succeed");
            let audio = &bytes[6..];
            let crc = header
                .compute_crc(header_bytes, audio)
                .expect("encoder allocation field fits the frame; compute_crc must succeed");
            let crc_bytes = crc.to_be_bytes();
            bytes[4] = crc_bytes[0];
            bytes[5] = crc_bytes[1];
        }

        Ok(bytes)
    }

    /// The §2.4.2.3 two-bit `mode` code for the encode mode.
    fn mode_bits(&self) -> u32 {
        match self.params.mode {
            Mode::Stereo => 0b00,
            Mode::JointStereo => 0b01,
            Mode::DualChannel => 0b10,
            Mode::SingleChannel => 0b11,
        }
    }
}

/// A stateful Layer II encoder: holds one [`AnalysisFilter`] per
/// channel so the §C.1.3 input FIFO carries across frames, mirroring
/// the decoder's per-channel synthesis-filter history.
///
/// One call to [`encode_frame`](Self::encode_frame) consumes exactly
/// [`LAYER2_SAMPLES_PER_FRAME`] interleaved `f64` PCM samples per
/// channel (`1152 * channels` values total), runs 36 slots of analysis
/// to populate the §2.4.1.6 sub-band matrix, and dispatches to the
/// top-level [`encode_layer2_frame`] for header + alloc + scfsi +
/// samples assembly. The optional §2.4.1.4 CRC is emitted iff the
/// caller's [`Layer2HeaderParams::has_crc`] is `true`.
///
/// This is the Layer II analogue of [`Mp1FrameEncoder`]: the Layer I
/// path packs 12 slots × 32 sub-bands = 384 PCM samples per channel
/// per frame; the Layer II path packs 36 slots × 32 sub-bands = 1152
/// PCM samples per channel per frame.
///
/// [`LAYER2_SAMPLES_PER_FRAME`]: crate::decode_layer2::LAYER2_SAMPLES_PER_FRAME
#[derive(Debug)]
pub struct Mp1Layer2FrameEncoder {
    params: Layer2HeaderParams,
    filters: Vec<AnalysisFilter>,
    /// Caller-supplied §2.4.1.8 `ancillary_data()` payload consumed by
    /// the next [`encode_frame`](Self::encode_frame) call and cleared
    /// afterwards. `None` (or empty) means "zero-pad the tail".
    pending_ancillary: Option<Vec<u8>>,
}

impl Mp1Layer2FrameEncoder {
    /// Build a Layer II frame encoder for the given header parameters
    /// with fresh (zeroed) analysis history.
    ///
    /// The header's `mode` field selects the channel count
    /// (`SingleChannel` → 1, every other variant → 2). Construction
    /// does not validate the §2.4.2.3 bitrate ladder; that check fires
    /// inside [`encode_frame`](Self::encode_frame) on first use, the
    /// same point [`encode_layer2_frame`] would raise it. This matches
    /// [`Mp1FrameEncoder`]'s lazy-validation contract.
    pub fn new(params: Layer2HeaderParams) -> Mp1Layer2FrameEncoder {
        let nch = params.mode.channels() as usize;
        Mp1Layer2FrameEncoder {
            params,
            filters: (0..nch).map(|_| AnalysisFilter::new()).collect(),
            pending_ancillary: None,
        }
    }

    /// Zero the analysis history (for a seek / stream restart). Also
    /// drops any caller-supplied §2.4.1.8 ancillary payload buffered by
    /// [`set_pending_ancillary`](Self::set_pending_ancillary) since the
    /// last [`encode_frame`](Self::encode_frame).
    pub fn reset(&mut self) {
        for f in &mut self.filters {
            f.reset();
        }
        self.pending_ancillary = None;
    }

    /// Stage a §2.4.1.8 `ancillary_data()` payload to be written into
    /// the tail of the next [`encode_frame`](Self::encode_frame) call.
    ///
    /// The bytes are copied into the §2.4.1.6 audio-data tail of the
    /// next frame; the staged payload is cleared after that one encode
    /// call (success or [`Layer2EncodeError::AncillaryTooLarge`]) so
    /// subsequent frames return to the default zero-padded tail.
    /// Passing an empty slice or calling
    /// [`clear_pending_ancillary`](Self::clear_pending_ancillary)
    /// restores the default behaviour without consuming a frame.
    pub fn set_pending_ancillary(&mut self, bytes: &[u8]) {
        if bytes.is_empty() {
            self.pending_ancillary = None;
        } else {
            self.pending_ancillary = Some(bytes.to_vec());
        }
    }

    /// Drop any §2.4.1.8 ancillary payload buffered by
    /// [`set_pending_ancillary`](Self::set_pending_ancillary).
    ///
    /// Subsequent [`encode_frame`](Self::encode_frame) calls will
    /// zero-pad the §2.4.1.8 tail until the caller stages another
    /// payload.
    pub fn clear_pending_ancillary(&mut self) {
        self.pending_ancillary = None;
    }

    /// Returns the currently-staged §2.4.1.8 ancillary payload, or `&[]`
    /// when no payload has been staged since the last
    /// [`encode_frame`](Self::encode_frame) (or
    /// [`reset`](Self::reset) / [`clear_pending_ancillary`](Self::clear_pending_ancillary)).
    pub fn pending_ancillary(&self) -> &[u8] {
        self.pending_ancillary.as_deref().unwrap_or(&[])
    }

    /// The channel count implied by the header mode (1 or 2).
    pub fn channels(&self) -> usize {
        self.params.mode.channels() as usize
    }

    /// A read-only view of the configured Layer II header parameters.
    pub fn params(&self) -> &Layer2HeaderParams {
        &self.params
    }

    /// Encode one Layer II frame of interleaved `f64` PCM in `[-1, 1)`
    /// to a complete §2.4.2.1 frame byte buffer.
    ///
    /// `pcm` is interleaved: `pcm[sample*nch + ch]`, with exactly
    /// `LAYER2_SAMPLES_PER_FRAME` (= 1152) samples per channel
    /// (§2.4.2.1). Internally runs 36 slots × 32 PCM samples through
    /// [`AnalysisFilter::analyze`] per channel, builds the
    /// `subbands[ch][sb][slot]` matrix the §2.4.1.6 writers expect,
    /// and dispatches to [`encode_layer2_frame`] (or
    /// [`encode_layer2_frame_with_ancillary`] when
    /// [`set_pending_ancillary`](Self::set_pending_ancillary) staged a
    /// §2.4.1.8 `ancillary_data()` payload). The returned frame is
    /// exactly `floor(144·bitrate/Fs) + padding_bit` bytes long. The
    /// staged ancillary buffer is cleared after this call regardless
    /// of outcome.
    ///
    /// [`LAYER2_SAMPLES_PER_FRAME`]: crate::decode_layer2::LAYER2_SAMPLES_PER_FRAME
    // The §C.1.3 analysis body is a strict (slot, ch, sb) nested walk
    // mirroring the §C.1.5.2 spec presentation; an iterator rewrite
    // would obscure the spec mapping for no behavioural gain.
    #[allow(clippy::needless_range_loop)]
    pub fn encode_frame(&mut self, pcm: &[f64]) -> Result<Vec<u8>, Layer2EncodeError> {
        use crate::decode_layer2::{LAYER2_SAMPLES_PER_FRAME, LAYER2_SAMPLES_PER_SUBBAND};

        let nch = self.channels();
        if !(nch == 1 || nch == 2) {
            return Err(Layer2EncodeError::UnsupportedChannelCount(nch));
        }
        let per_ch = LAYER2_SAMPLES_PER_FRAME; // 1152
        if pcm.len() != per_ch * nch {
            return Err(Layer2EncodeError::WrongSampleCount {
                got: pcm.len() / nch.max(1),
            });
        }

        // §C.1.3 analysis: run the per-channel AnalysisFilter 36 times,
        // 32 PCM samples per slot, capturing 32 sub-band outputs per
        // slot. The resulting `subbands[ch][sb][slot]` matrix has the
        // exact shape `encode_layer2_frame` consumes.
        let mut subbands = [[[0.0f64; LAYER2_SAMPLES_PER_SUBBAND]; SUBBANDS]; 2];
        for slot in 0..LAYER2_SAMPLES_PER_SUBBAND {
            for ch in 0..nch {
                let mut input = [0.0f64; SUBBANDS];
                for (j, v) in input.iter_mut().enumerate() {
                    let sample_idx = slot * SUBBANDS + j;
                    *v = pcm[sample_idx * nch + ch];
                }
                let s = self.filters[ch].analyze(&input);
                for sb in 0..SUBBANDS {
                    subbands[ch][sb][slot] = s[sb];
                }
            }
        }

        let pending = self.pending_ancillary.take().unwrap_or_default();
        if pending.is_empty() {
            encode_layer2_frame(&self.params, &subbands)
        } else {
            encode_layer2_frame_with_ancillary(&self.params, &subbands, &pending)
        }
    }
}

#[cfg(test)]
// Tests walk per-channel (ch, sb) grids in the same row-major order as
// the §C.1.5.x encode body, so flat index loops mirror the spec
// presentation; allow `needless_range_loop` here for that reason.
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;
    use crate::decode::requantize;
    use crate::synthesis::SynthesisFilter;

    // ---- analysis matrix (§C.1.3) -----------------------------

    #[test]
    fn analysis_matrix_spot_values() {
        let m = analysis_matrix();
        // M_ik = cos[(2i+1)(k-16) pi / 64].
        // i=0,k=16: (1)(0) -> cos(0) = 1.
        assert!((m[16] - 1.0).abs() < 1e-12);
        // i=0,k=0: (1)(-16) pi/64 = -pi/4 -> cos = sqrt(2)/2.
        assert!((m[0] - (std::f64::consts::FRAC_PI_4).cos()).abs() < 1e-12);
        // i=0,k=48: (1)(32) pi/64 = pi/2 -> cos = 0.
        assert!(m[48].abs() < 1e-12);
    }

    // ---- quantize is the inverse of requantize (§C.1.5.1.7) ---

    #[test]
    fn quantize_inverts_requantize() {
        // For every nb and every code, requantize -> (renormalize) ->
        // quantize must recover the original code exactly. This proves
        // the encoder's Table C.3 quantizer is the exact inverse of the
        // decoder's §2.4.3.2 requantizer.
        for nb in 2u8..=12 {
            for code in 0..(1u32 << nb) {
                // s'' = requantize(code) is the value the decoder yields
                // for a unit scalefactor; quantizing it back must return
                // the same code.
                let value = requantize(code, nb);
                let got = quantize(value, 1.0, nb);
                assert_eq!(got, code, "nb={nb} code={code} -> {got}");
            }
        }
    }

    #[test]
    fn quantize_respects_scalefactor() {
        // A loud sample with a scalefactor that just covers it should
        // quantize near full scale and decode back close to the input.
        let nb = 6u8;
        let scf = SCALEFACTORS[select_scalefactor(0.5) as usize];
        let code = quantize(0.5, scf, nb);
        let back = requantize(code, nb) * scf;
        assert!((back - 0.5).abs() < scf * 0.05, "back={back}");
    }

    // ---- scalefactor selection (§C.1.5.1.4) -------------------

    #[test]
    fn select_scalefactor_picks_lowest_value_above_peak() {
        // SCALEFACTORS[3] = 1.0, [4] = 0.79370. A peak of 0.9 needs a
        // multiplier strictly larger than 0.9: the lowest such value is
        // 1.0 at index 3.
        assert_eq!(select_scalefactor(0.9), 3);
        // A peak just below 1.0 still picks index 3.
        assert_eq!(select_scalefactor(0.999), 3);
        // A peak of 0.7 (< 0.79370) picks index 4.
        assert_eq!(select_scalefactor(0.7), 4);
        // A tiny peak picks a deep index (large attenuation) but never
        // exceeds 62.
        assert!(select_scalefactor(1e-9) <= 62);
        // A peak >= the largest multiplier (2.0) clamps to index 0.
        assert_eq!(select_scalefactor(3.0), 0);
    }

    #[test]
    fn select_scalefactor_multiplier_exceeds_peak() {
        // The chosen multiplier must be > peak for any in-range peak, so
        // the normalized sample X = peak/scf stays in [-1, 1).
        for &peak in &[0.001f64, 0.01, 0.1, 0.33, 0.5, 0.8, 0.95] {
            let idx = select_scalefactor(peak);
            assert!(
                SCALEFACTORS[idx as usize] > peak,
                "peak={peak} idx={idx} scf={}",
                SCALEFACTORS[idx as usize]
            );
        }
    }

    // ---- bit allocation (§C.1.5.1.6) --------------------------

    #[test]
    fn allocate_respects_budget() {
        // One loud subband, rest silent: the loud band should get bits,
        // the silent ones none, and total cost must fit the budget.
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        energy[0][0] = 0.5;
        let budget = 200usize;
        let alloc = allocate_bits(&energy, 1, budget);
        assert!(alloc[0][0] >= 2, "loud band got {} bits", alloc[0][0]);
        for (sb, &nb) in alloc[0].iter().enumerate().skip(1) {
            assert_eq!(nb, 0, "silent sb{sb} got bits");
        }
        // Re-derive the spent bits and confirm <= budget.
        let mut spent = 0usize;
        let nb = alloc[0][0];
        if nb > 0 {
            spent += 6 + 12 * nb as usize;
        }
        assert!(spent <= budget, "spent {spent} > budget {budget}");
    }

    #[test]
    fn allocate_prefers_louder_subbands() {
        // Two subbands, one much louder: with a tight budget the louder
        // one should not receive fewer bits than the quieter one.
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        energy[0][0] = 0.5; // loud
        energy[0][1] = 0.001; // quiet
        let alloc = allocate_bits(&energy, 1, 120);
        assert!(
            alloc[0][0] >= alloc[0][1],
            "loud sb0={} quiet sb1={}",
            alloc[0][0],
            alloc[0][1]
        );
    }

    #[test]
    fn allocate_zero_budget_allocates_nothing() {
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        energy[0][0] = 0.5;
        let alloc = allocate_bits(&energy, 1, 0);
        for &nb in alloc[0].iter() {
            assert_eq!(nb, 0);
        }
    }

    // ---- analysis/synthesis reconstruction --------------------

    /// Feed a sine through analyze() then synthesize() (no quantization)
    /// and confirm the polyphase pair reconstructs the input after the
    /// filterbank delay. This is the strongest check that the Table C.1
    /// analysis window signs and the matrixing formula are correct: a
    /// sign error anywhere in the window destroys reconstruction.
    #[test]
    fn analysis_synthesis_reconstructs() {
        let mut af = AnalysisFilter::new();
        let mut sf = SynthesisFilter::new();
        // 256-sample analysis delay (annex C.1.5.1.10 note: "The delay
        // of the analysis subband filter is 256 samples"); synthesis
        // adds the same, so the round-trip group delay is 481 samples
        // for this prototype. Drive enough slots to clear it.
        let n_slots = 64;
        let freq = 0.05f64; // cycles/sample
        let mut input_hist = Vec::new();
        let mut output_hist = Vec::new();
        let mut t = 0usize;
        for _ in 0..n_slots {
            let mut block = [0.0f64; SUBBANDS];
            for v in block.iter_mut() {
                *v = 0.4 * (2.0 * std::f64::consts::PI * freq * t as f64).sin();
                input_hist.push(*v);
                t += 1;
            }
            let s = af.analyze(&block);
            let out = sf.synthesize(&s);
            for &o in out.iter() {
                output_hist.push(o);
            }
        }
        // Find the best alignment delay that minimizes error in the
        // steady-state region. The MPEG analysis+synthesis pair is
        // normalised so the steady-state output reconstructs the input
        // unit-for-unit (delay 481 samples, output ≈ input). A sign
        // error anywhere in the Table C.1 transcription, the matrixing
        // formula, or the cross-window phasing would blow this up.
        let mut best_delay = 0usize;
        let mut best_err = f64::INFINITY;
        for d in 400..520 {
            let mut err = 0.0f64;
            let mut cnt = 0usize;
            for i in (d + 50)..(input_hist.len().saturating_sub(50)) {
                if i < output_hist.len() {
                    let diff = output_hist[i] - input_hist[i - d];
                    err += diff * diff;
                    cnt += 1;
                }
            }
            if cnt > 32 {
                let rms = (err / cnt as f64).sqrt();
                if rms < best_err {
                    best_err = rms;
                    best_delay = d;
                }
            }
        }
        assert!(
            best_err < 1e-3,
            "reconstruction RMS {best_err} too large (best delay {best_delay})"
        );
    }

    // ---- §2.4.1.4 optional CRC emission -----------------------

    use crate::header::{CrcStatus, FrameHeader as TestFrameHeader};

    /// 384 samples per channel = one Layer I frame (§2.4.2.1).
    fn unit_mono_pcm() -> Vec<f64> {
        let mut pcm = vec![0.0f64; SAMPLES_PER_SUBBAND * SUBBANDS];
        for (n, v) in pcm.iter_mut().enumerate() {
            // 1 kHz tone at Fs = 48 kHz; modest amplitude to keep
            // every subband below clip.
            let t = n as f64 / 48_000.0;
            *v = 0.4 * (2.0 * std::f64::consts::PI * 1_000.0 * t).sin();
        }
        pcm
    }

    #[test]
    fn encode_params_emit_crc_default_off() {
        // The historical encoder default is `protection_bit == 1` (no
        // CRC); the new field must preserve that out of the box.
        let p = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel);
        assert!(!p.emit_crc);
    }

    #[test]
    fn encode_params_with_emit_crc_builder() {
        let p =
            EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel).with_emit_crc(true);
        assert!(p.emit_crc);
        // The builder must not flip any other field.
        assert_eq!(p.bitrate, Bitrate::Fixed(256));
        assert_eq!(p.sampling_frequency, 48_000);
        assert_eq!(p.mode, Mode::SingleChannel);
    }

    #[test]
    fn encoded_frame_without_crc_has_protection_bit_1() {
        // Baseline: the default (emit_crc=false) writes
        // protection_bit==1 and the header reports has_crc()==false.
        let params = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel);
        let mut enc = Mp1FrameEncoder::new(params);
        let bytes = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        assert!(!h.has_crc(), "default encoder must not emit a CRC");
        assert!(h.protection, "protection_bit must be 1 when no CRC");
    }

    #[test]
    fn encoded_frame_with_crc_has_protection_bit_0() {
        let params =
            EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel).with_emit_crc(true);
        let mut enc = Mp1FrameEncoder::new(params);
        let bytes = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        assert!(h.has_crc(), "emit_crc=true must clear protection_bit");
        assert!(
            !h.protection,
            "protection_bit must be 0 when CRC is emitted"
        );
    }

    #[test]
    fn encoded_frame_with_crc_verifies_against_spec_helper() {
        // The CRC the encoder writes must verify clean through the
        // decoder-side `FrameHeader::verify_crc` (§2.4.3.1). This is the
        // round-trip property that guarantees an external decoder will
        // accept the frame's `error_check()` field.
        let params =
            EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel).with_emit_crc(true);
        let mut enc = Mp1FrameEncoder::new(params);
        let bytes = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        let status = h
            .verify_crc(&bytes[..4], &bytes[4..])
            .expect("CRC region present");
        match status {
            CrcStatus::Ok(_) => {}
            other => panic!("encoder-emitted CRC failed verification: {other:?}"),
        }
        assert!(status.is_good());
    }

    #[test]
    fn encoded_frame_with_crc_keeps_slot_count() {
        // §2.4.2.1: the slot count `N = floor(12 * bitrate / Fs) +
        // padding` is a function of the header alone, so enabling the
        // CRC must not change the per-frame byte length.
        let mut enc_no = Mp1FrameEncoder::new(EncodeParams::new(
            Bitrate::Fixed(256),
            48_000,
            Mode::SingleChannel,
        ));
        let mut enc_crc = Mp1FrameEncoder::new(
            EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel).with_emit_crc(true),
        );
        let pcm = unit_mono_pcm();
        let n = enc_no.encode_frame(&pcm).unwrap().len();
        let c = enc_crc.encode_frame(&pcm).unwrap().len();
        assert_eq!(n, c, "CRC emission must not change the slot/byte count");
        // 12 * 256000 / 48000 = 64 slots * 4 bytes/slot = 256.
        assert_eq!(n, 256);
    }

    #[test]
    fn encoded_frame_with_crc_corruption_is_detected() {
        // Flip a single bit inside the protected bit-allocation field
        // and confirm `verify_crc` reports `Mismatch`. The first
        // allocation nibble lives at bytes[6] for a CRC-protected
        // frame (header 0..4, CRC 4..6, allocation 6..).
        let params =
            EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::Stereo).with_emit_crc(true);
        let mut enc = Mp1FrameEncoder::new(params);
        let mut pcm = Vec::with_capacity(SAMPLES_PER_SUBBAND * SUBBANDS * 2);
        for n in 0..SAMPLES_PER_SUBBAND * SUBBANDS {
            let v = 0.3 * (n as f64 * 0.07).sin();
            pcm.push(v); // ch0
            pcm.push(v); // ch1
        }
        let mut bytes = enc.encode_frame(&pcm).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        assert!(h.has_crc());
        // Sanity: the un-touched frame verifies.
        assert!(matches!(
            h.verify_crc(&bytes[..4], &bytes[4..]),
            Some(CrcStatus::Ok(_))
        ));
        // Flip a bit inside the allocation field.
        bytes[6] ^= 0x80;
        match h.verify_crc(&bytes[..4], &bytes[4..]) {
            Some(CrcStatus::Mismatch { stored, computed }) => {
                assert_ne!(stored, computed);
            }
            other => panic!("expected Mismatch after corruption, got {other:?}"),
        }
    }

    #[test]
    fn encode_frame_with_crc_stereo_round_trips() {
        // Stereo (`nch = 2`) doubles the allocation field to 256 bits;
        // confirm `compute_crc` sizes it correctly via the parsed
        // header and the round-trip still verifies.
        let params =
            EncodeParams::new(Bitrate::Fixed(384), 48_000, Mode::Stereo).with_emit_crc(true);
        let mut enc = Mp1FrameEncoder::new(params);
        let mut pcm = vec![0.0f64; SAMPLES_PER_SUBBAND * SUBBANDS * 2];
        for n in 0..SAMPLES_PER_SUBBAND * SUBBANDS {
            let t = n as f64 / 48_000.0;
            let v = 0.4 * (2.0 * std::f64::consts::PI * 880.0 * t).sin();
            pcm[n * 2] = v;
            pcm[n * 2 + 1] = -v;
        }
        let bytes = enc.encode_frame(&pcm).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        assert!(h.has_crc());
        assert_eq!(h.mode, Mode::Stereo);
        let status = h
            .verify_crc(&bytes[..4], &bytes[4..])
            .expect("CRC region present");
        assert!(matches!(status, CrcStatus::Ok(_)));
    }

    // ---- Free-format Layer I encode (§2.4.2.3 bitrate_index 0b0000) ----

    #[test]
    fn free_format_builder_sets_target_and_keeps_other_fields() {
        let p = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel)
            .with_free_format(200);
        assert_eq!(p.free_format_kbps, Some(200));
        // The builder must not disturb the unrelated fields.
        assert_eq!(p.sampling_frequency, 48_000);
        assert_eq!(p.mode, Mode::SingleChannel);
        assert!(!p.emit_crc);
        // The default constructor leaves free format off.
        let d = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel);
        assert_eq!(d.free_format_kbps, None);
    }

    #[test]
    fn free_format_header_carries_bitrate_index_zero_and_off_ladder_length() {
        // §2.4.2.3: a free-format frame writes `bitrate_index == 0b0000`
        // into the header. The frame is still sized by §2.4.2.1
        // `N = floor(12 * kbps / Fs)`, here at an OFF-ladder 200 kbit/s
        // (the MPEG-1 Layer I ladder has 192 then 224, never 200).
        let params = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel)
            .with_free_format(200);
        let mut enc = Mp1FrameEncoder::new(params);
        let bytes = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        // The parsed header must report free format, not a ladder rate.
        assert_eq!(h.bitrate, Bitrate::Free);
        // 12 * 200000 / 48000 = 50 slots * 4 bytes = 200 bytes.
        assert_eq!(bytes.len(), 200);
        // protection_bit defaults to 1 (no CRC) in the free path too.
        assert!(h.protection);
    }

    #[test]
    fn free_format_frame_length_recovered_by_decoder_probe() {
        // The decoder cannot size a free-format frame from the header
        // alone (§2.4.3.1): it measures the byte distance to the next
        // syncword. Encode two consecutive constant-rate free-format
        // frames and confirm the §2.4.3.1 probe recovers the same `N`,
        // byte length and back-derived bitrate the encoder used.
        use crate::header::detect_free_format_frame_length;
        let params = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel)
            .with_free_format(200);
        let mut enc = Mp1FrameEncoder::new(params);
        let f0 = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let f1 = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let mut stream = f0.clone();
        stream.extend_from_slice(&f1);

        let h = TestFrameHeader::parse(&stream[..4]).unwrap();
        assert_eq!(h.bitrate, Bitrate::Free);
        let probe = detect_free_format_frame_length(&h, &stream[4..])
            .expect("next syncword present after first free-format frame");
        // 50 slots, 200 bytes, 200 kbit/s — exactly what we encoded.
        assert_eq!(probe.base_slot_count, 50);
        assert_eq!(probe.frame_length_bytes as usize, f0.len());
        assert_eq!(probe.bitrate_kbps, 200);
    }

    #[test]
    fn free_format_round_trips_to_pcm() {
        // A free-format frame must still decode to PCM through the
        // standard §2.4.3.2 audio-data path — the free-format marker
        // only affects frame sizing, not the audio-data syntax.
        use crate::decode::decode_audio_data;
        let params = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel)
            .with_free_format(200);
        let mut enc = Mp1FrameEncoder::new(params);
        let bytes = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        // Audio data begins immediately after the 4-byte header (no CRC).
        let sb = decode_audio_data(&h, &bytes[4..]).expect("free-format audio data decodes");
        // At least one subband must carry a non-zero requantized sample
        // for the 1 kHz test tone.
        let any_nonzero = sb.subbands[0]
            .iter()
            .any(|band| band.allocated && band.samples.iter().any(|&v| v != 0.0));
        assert!(any_nonzero, "decoded free-format frame is silent");
    }

    #[test]
    fn free_format_rejects_zero_and_oversized_targets() {
        // §2.4.3.1: a free-format frame holds at most 512 four-byte
        // slots, and a zero-rate target leaves no audio-data budget.
        let mut zero = Mp1FrameEncoder::new(
            EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel).with_free_format(0),
        );
        assert!(matches!(
            zero.encode_frame(&unit_mono_pcm()),
            Err(EncodeError::UnsupportedBitrate)
        ));
        // 12 * 2100 kbit/s / 48 kHz = 525 slots > 512.
        let mut big = Mp1FrameEncoder::new(
            EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel)
                .with_free_format(2100),
        );
        assert!(matches!(
            big.encode_frame(&unit_mono_pcm()),
            Err(EncodeError::UnsupportedBitrate)
        ));
    }

    #[test]
    fn free_format_with_crc_verifies() {
        // Free format and the optional §2.4.1.4 CRC are independent
        // switches; combining them must still write `bitrate_index == 0`
        // AND a verifying CRC word.
        let params = EncodeParams::new(Bitrate::Fixed(256), 48_000, Mode::SingleChannel)
            .with_free_format(160)
            .with_emit_crc(true);
        let mut enc = Mp1FrameEncoder::new(params);
        let bytes = enc.encode_frame(&unit_mono_pcm()).unwrap();
        let h = TestFrameHeader::parse(&bytes[..4]).unwrap();
        assert_eq!(h.bitrate, Bitrate::Free);
        assert!(h.has_crc());
        let status = h
            .verify_crc(&bytes[..4], &bytes[4..])
            .expect("CRC region present");
        assert!(status.is_good(), "free-format CRC failed: {status:?}");
    }

    // ---- Layer II bit allocation (§C.1.5.2.7) -----------------

    use crate::header::{
        Bitrate as TestBitrate, Emphasis as TestEmphasis, FrameHeader as TestFrameHeader2,
        Id as TestId, Layer as TestLayer, Mode as TestMode, ModeExtension as TestModeExt,
    };
    use crate::tables_layer2::layer2_bit_allocation_table;

    fn header_layer2(fs: u32, kbps: u16, mode: TestMode) -> TestFrameHeader2 {
        TestFrameHeader2 {
            id: TestId::Mpeg,
            layer: TestLayer::II,
            protection: true,
            bitrate: TestBitrate::Fixed(kbps),
            sampling_frequency: fs,
            padding: false,
            private: false,
            mode,
            mode_extension: TestModeExt(0),
            copyright: false,
            original: true,
            emphasis: TestEmphasis::None,
        }
    }

    #[test]
    fn layer2_payload_bits_matches_spec_formula() {
        // §2.4.2.1: a Layer II frame is `floor(144 · bitrate / Fs)`
        // bytes; the §C.1.5.2.7 audio-data budget `adb` is
        // (frame_bytes·8) − (32 + bcrc + bbal). For 128 kbit/s mono at
        // 44.1 kHz with no CRC and Table B.2b (94 bits of `bbal` per
        // channel), the budget is `floor(144·128000/44100)·8 − 32 − 94`.
        let h = header_layer2(44_100, 128, TestMode::SingleChannel);
        let table = layer2_bit_allocation_table(&h);
        let bytes = 144 * 128_000u32 / 44_100;
        let expected = (bytes as usize * 8) - 32 - 94;
        assert_eq!(
            layer2_frame_payload_bits(128, 44_100, false, 1, table),
            expected
        );
        // CRC eats another 16 bits.
        assert_eq!(
            layer2_frame_payload_bits(128, 44_100, true, 1, table),
            expected - 16
        );
        // Stereo doubles `bbal` (two channels of nbal=94).
        assert_eq!(
            layer2_frame_payload_bits(128, 44_100, false, 2, table),
            (bytes as usize * 8) - 32 - 2 * 94
        );
    }

    #[test]
    fn layer2_class_cost_known_values() {
        use crate::tables_layer2::QUANT_CLASSES;
        // nlevels = 3, grouped, bits_per_codeword = 5, 12 codewords/frame.
        let cls = QUANT_CLASSES.iter().find(|c| c.nlevels == 3).unwrap();
        assert_eq!(layer2_class_cost_bits(cls), 12 * 5);
        // nlevels = 7, non-grouped, bits_per_codeword = 3, 36 codewords.
        let cls = QUANT_CLASSES.iter().find(|c| c.nlevels == 7).unwrap();
        assert_eq!(layer2_class_cost_bits(cls), 36 * 3);
        // nlevels = 65535, non-grouped, bits_per_codeword = 16.
        let cls = QUANT_CLASSES.iter().find(|c| c.nlevels == 65535).unwrap();
        assert_eq!(layer2_class_cost_bits(cls), 36 * 16);
    }

    #[test]
    fn layer2_allocate_respects_budget() {
        // Mono frame at 128 kbit/s, 44.1 kHz (Table B.2b): one loud
        // subband, rest silent. The loud band must get some allocation;
        // every other (and every subband at or above sblimit) must stay
        // at zero. The total spent bits must fit the budget.
        let h = header_layer2(44_100, 128, TestMode::SingleChannel);
        let table = layer2_bit_allocation_table(&h);
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        energy[0][2] = 0.5;
        let budget = layer2_frame_payload_bits(128, 44_100, false, 1, table);
        let alloc = allocate_bits_layer2(&energy, 1, table, budget);

        assert!(alloc[0][2] >= 1, "loud sb got nothing: {}", alloc[0][2]);
        for (sb, &a) in alloc[0].iter().enumerate() {
            if sb == 2 {
                continue;
            }
            assert_eq!(a, 0, "silent sb{sb} got alloc {a}");
        }
        // Re-derive spent bits and confirm it fits the budget.
        let cls = table.quant_class(2, alloc[0][2]).expect("class");
        let cost = layer2_class_cost_bits(cls) + LAYER2_PER_SUBBAND_OVERHEAD_BITS;
        assert!(cost <= budget, "cost {cost} > budget {budget}");
    }

    #[test]
    fn layer2_allocate_prefers_louder_subbands() {
        // Two subbands, one much louder: with a tight budget the louder
        // band must not receive a coarser class than the quieter one.
        let h = header_layer2(44_100, 128, TestMode::SingleChannel);
        let table = layer2_bit_allocation_table(&h);
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        energy[0][0] = 0.5;
        energy[0][1] = 0.001;
        let alloc = allocate_bits_layer2(&energy, 1, table, 400);
        let nl_a = table
            .quant_class(0, alloc[0][0])
            .map(|c| c.nlevels)
            .unwrap_or(0);
        let nl_b = table
            .quant_class(1, alloc[0][1])
            .map(|c| c.nlevels)
            .unwrap_or(0);
        assert!(
            nl_a >= nl_b,
            "loud sb0 nlevels={} < quiet sb1 nlevels={}",
            nl_a,
            nl_b,
        );
    }

    #[test]
    fn layer2_allocate_zero_budget_allocates_nothing() {
        let h = header_layer2(44_100, 128, TestMode::SingleChannel);
        let table = layer2_bit_allocation_table(&h);
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        energy[0][0] = 0.5;
        let alloc = allocate_bits_layer2(&energy, 1, table, 0);
        for &a in alloc[0].iter() {
            assert_eq!(a, 0);
        }
    }

    #[test]
    fn layer2_allocate_skips_subbands_above_sblimit() {
        // Table B.2c has sblimit = 8 (32 kbit/s mono at 44.1 kHz).
        // Energy in a high subband (sb 20) must NOT receive any
        // allocation — the bitstream has no row for it.
        let h = header_layer2(44_100, 32, TestMode::SingleChannel);
        let table = layer2_bit_allocation_table(&h);
        assert_eq!(table.sblimit(), 8);
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        energy[0][20] = 0.9; // above sblimit
        let budget = layer2_frame_payload_bits(32, 44_100, false, 1, table);
        let alloc = allocate_bits_layer2(&energy, 1, table, budget);
        // All subbands `>= sblimit` must remain zero.
        for sb in table.sblimit()..SUBBANDS {
            assert_eq!(alloc[0][sb], 0, "sb{sb} (>= sblimit) got alloc");
        }
    }

    #[test]
    fn layer2_allocate_step_picks_legal_alloc_index() {
        // For every (ch, sb) with non-zero allocation, the chosen index
        // must resolve to a real `QuantClass` (i.e. it must not land on
        // a `None` cell in the row). This is the structural correctness
        // check against Table B.2x's `-` gaps.
        let h = header_layer2(48_000, 96, TestMode::Stereo);
        let table = layer2_bit_allocation_table(&h);
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        // Spread some energy across subbands in both channels.
        for sb in 0..table.sblimit() {
            energy[0][sb] = 0.3 / (sb as f64 + 1.0);
            energy[1][sb] = 0.2 / (sb as f64 + 1.0);
        }
        let budget = layer2_frame_payload_bits(96, 48_000, false, 2, table);
        let alloc = allocate_bits_layer2(&energy, 2, table, budget);
        for ch in 0..2 {
            for sb in 0..table.sblimit() {
                if alloc[ch][sb] != 0 {
                    assert!(
                        table.quant_class(sb, alloc[ch][sb]).is_some(),
                        "ch{ch} sb{sb} chose illegal alloc {}",
                        alloc[ch][sb],
                    );
                }
            }
        }
    }

    #[test]
    fn layer2_allocate_total_cost_fits_budget() {
        // Sum the per-(ch, sb) costs of the produced allocation and
        // assert the total stays within budget. This is the §C.1.5.2.7
        // safety property — the loop must not "spend bits it doesn't
        // have".
        let h = header_layer2(44_100, 192, TestMode::Stereo);
        let table = layer2_bit_allocation_table(&h);
        let mut energy = [[0.0f64; SUBBANDS]; 2];
        for sb in 0..table.sblimit() {
            energy[0][sb] = 0.4 / (sb as f64 + 1.0);
            energy[1][sb] = 0.4 / (sb as f64 + 1.0);
        }
        let budget = layer2_frame_payload_bits(192, 44_100, false, 2, table);
        let alloc = allocate_bits_layer2(&energy, 2, table, budget);

        let mut spent = 0usize;
        for ch in 0..2 {
            for sb in 0..table.sblimit() {
                if alloc[ch][sb] == 0 {
                    continue;
                }
                let cls = table.quant_class(sb, alloc[ch][sb]).unwrap();
                spent += layer2_class_cost_bits(cls) + LAYER2_PER_SUBBAND_OVERHEAD_BITS;
            }
        }
        assert!(spent <= budget, "spent {spent} > budget {budget}");
    }

    // ---- Layer II frame-header writer (§2.4.2.3) ------------------

    #[test]
    fn bitrate_index_layer2_mpeg1_ladder_endpoints() {
        // Spec §2.4.2.3 ladder (MPEG-1 Layer II): smallest is 32 → 1,
        // largest is 384 → 14. A value outside the ladder is rejected.
        assert_eq!(bitrate_index_layer2(32, 1), Some(1));
        assert_eq!(bitrate_index_layer2(48, 1), Some(2));
        assert_eq!(bitrate_index_layer2(384, 1), Some(14));
        assert_eq!(bitrate_index_layer2(33, 1), None);
        // 448 is on the Layer I ladder but NOT Layer II — must reject.
        assert_eq!(bitrate_index_layer2(448, 1), None);
    }

    #[test]
    fn bitrate_index_layer2_lsf_ladder_endpoints() {
        // LSF Layer II/III ladder (13818-3 §2.4.2.3): 8 → 1, 160 → 14.
        assert_eq!(bitrate_index_layer2(8, 0), Some(1));
        assert_eq!(bitrate_index_layer2(16, 0), Some(2));
        assert_eq!(bitrate_index_layer2(160, 0), Some(14));
        // 256 is on the LSF Layer I ladder but NOT the LSF L2/L3
        // ladder — must reject.
        assert_eq!(bitrate_index_layer2(256, 0), None);
    }

    #[test]
    fn pack_layer2_header_known_bits() {
        // 128 kbit/s mono at 44.1 kHz, no CRC, no padding, original,
        // no emphasis. The expected 32-bit big-endian word is built
        // field-by-field from §2.4.1.3:
        //   syncword (0xFFF)        = 0xFFF00000
        //   ID = 1                  -> bit 19 set
        //   layer 0b10              -> bits 18..17
        //   protection_bit = 1      -> bit 16 (NO CRC)
        //   bitrate_index = 0b1000  -> 128 kbit/s on L2 MPEG-1 ladder
        //   sampling = 0b00         -> 44.1 kHz
        //   padding = 0
        //   private = 0
        //   mode = 0b11             -> single_channel
        //   mode_ext = 0
        //   copyright = 0
        //   original = 1
        //   emphasis = 0
        let params = Layer2HeaderParams::new(44_100, 128, Mode::SingleChannel);
        let bytes = pack_layer2_header(&params).expect("pack");
        let word = u32::from_be_bytes(bytes);

        // syncword
        assert_eq!(word >> 20, 0xFFF);
        // ID
        assert_eq!((word >> 19) & 0x1, 1);
        // layer = II = 0b10
        assert_eq!((word >> 17) & 0x3, 0b10);
        // protection_bit = 1 (no CRC, the default)
        assert_eq!((word >> 16) & 0x1, 1);
        // bitrate_index for 128 kbit/s on the L2 MPEG-1 ladder. Ladder
        // is {32,48,56,64,80,96,112,128,...} so index 8 → bits 0b1000.
        assert_eq!((word >> 12) & 0xF, 0b1000);
        // sampling = 0b00 (44.1 kHz on the MPEG-1 table)
        assert_eq!((word >> 10) & 0x3, 0b00);
        // mode = 0b11 (single_channel)
        assert_eq!((word >> 6) & 0x3, 0b11);
        // original = 1
        assert_eq!((word >> 2) & 0x1, 1);
    }

    #[test]
    fn pack_layer2_header_with_crc_clears_protection_bit() {
        // §2.4.2.3: protection_bit `'0'` indicates redundancy has been
        // added. The writer flips the bit accordingly when has_crc is
        // set on the params.
        let mut params = Layer2HeaderParams::new(44_100, 128, Mode::SingleChannel);
        params.has_crc = true;
        let bytes = pack_layer2_header(&params).expect("pack");
        let word = u32::from_be_bytes(bytes);
        assert_eq!((word >> 16) & 0x1, 0, "protection_bit must clear on CRC");
        // Parse-back must report the bit (FrameHeader::protection is
        // `true` when NO redundancy was added).
        let h = crate::header::FrameHeader::parse(&bytes).unwrap();
        assert!(!h.protection, "FrameHeader::protection must be false");
        assert!(h.has_crc(), "FrameHeader::has_crc must be true");
    }

    #[test]
    fn pack_layer2_header_lsf_id_bit() {
        // Any LSF sampling frequency (16 / 22.05 / 24 kHz) must set
        // ID == 0 and pick a bitrate from the LSF ladder.
        for fs in [16_000u32, 22_050, 24_000] {
            let params = Layer2HeaderParams::new(fs, 64, Mode::Stereo);
            let bytes = pack_layer2_header(&params).expect("pack");
            let word = u32::from_be_bytes(bytes);
            assert_eq!((word >> 19) & 0x1, 0, "ID must be 0 for LSF Fs={fs}");
            let h = crate::header::FrameHeader::parse(&bytes).unwrap();
            assert!(h.is_lsf(), "FrameHeader::is_lsf must hold for Fs={fs}");
            assert_eq!(h.sampling_frequency, fs);
        }
    }

    #[test]
    fn pack_layer2_header_rejects_off_ladder_bitrate() {
        // 448 kbit/s is on Layer I (MPEG-1) but NOT Layer II.
        let params = Layer2HeaderParams::new(44_100, 448, Mode::Stereo);
        match pack_layer2_header(&params) {
            Err(Layer2HeaderError::UnsupportedBitrate(448)) => {}
            other => panic!("expected UnsupportedBitrate(448), got {other:?}"),
        }
    }

    #[test]
    fn pack_layer2_header_rejects_unknown_sampling() {
        let params = Layer2HeaderParams::new(11_025, 128, Mode::Stereo);
        match pack_layer2_header(&params) {
            Err(Layer2HeaderError::UnsupportedSamplingFrequency(11_025)) => {}
            other => panic!("expected UnsupportedSamplingFrequency(11_025), got {other:?}"),
        }
    }

    #[test]
    fn pack_layer2_header_mpeg1_matrix_roundtrips_through_parse() {
        // The §2.4.2.3 MPEG-1 Layer II header carries 14 bitrates
        // (the full ladder), 3 sampling frequencies, and 4 channel
        // modes — 168 (bitrate, fs, mode) combinations. Each must
        // pack and re-parse to the same FrameHeader fields.
        use crate::header::{Bitrate, FrameHeader, Layer};
        const L2_MPEG1: [u16; 14] = [
            32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384,
        ];
        const SAMPS: [u32; 3] = [44_100, 48_000, 32_000];
        const MODES: [Mode; 4] = [
            Mode::Stereo,
            Mode::JointStereo,
            Mode::DualChannel,
            Mode::SingleChannel,
        ];
        for &kbps in L2_MPEG1.iter() {
            for &fs in SAMPS.iter() {
                for &mode in MODES.iter() {
                    let params = Layer2HeaderParams::new(fs, kbps, mode);
                    let bytes = pack_layer2_header(&params)
                        .unwrap_or_else(|_| panic!("pack fs={fs} kbps={kbps}"));
                    let h = FrameHeader::parse(&bytes)
                        .unwrap_or_else(|_| panic!("parse fs={fs} kbps={kbps}"));
                    assert_eq!(h.layer, Layer::II);
                    assert_eq!(h.bitrate, Bitrate::Fixed(kbps));
                    assert_eq!(h.sampling_frequency, fs);
                    assert_eq!(h.mode, mode);
                    assert!(h.protection, "default params: no CRC");
                    assert!(!h.padding);
                    assert!(!h.copyright);
                    assert!(h.original);
                }
            }
        }
    }

    #[test]
    fn pack_layer2_header_lsf_matrix_roundtrips_through_parse() {
        // Same matrix shape, but the LSF ladder and LSF sampling
        // table from 13818-3 §2.4.2.3.
        use crate::header::{Bitrate, FrameHeader, Layer};
        const L2_LSF: [u16; 14] = [8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160];
        const SAMPS: [u32; 3] = [16_000, 22_050, 24_000];
        const MODES: [Mode; 4] = [
            Mode::Stereo,
            Mode::JointStereo,
            Mode::DualChannel,
            Mode::SingleChannel,
        ];
        for &kbps in L2_LSF.iter() {
            for &fs in SAMPS.iter() {
                for &mode in MODES.iter() {
                    let params = Layer2HeaderParams::new(fs, kbps, mode);
                    let bytes = pack_layer2_header(&params)
                        .unwrap_or_else(|_| panic!("pack fs={fs} kbps={kbps}"));
                    let h = FrameHeader::parse(&bytes)
                        .unwrap_or_else(|_| panic!("parse fs={fs} kbps={kbps}"));
                    assert_eq!(h.layer, Layer::II);
                    assert!(h.is_lsf());
                    assert_eq!(h.bitrate, Bitrate::Fixed(kbps));
                    assert_eq!(h.sampling_frequency, fs);
                    assert_eq!(h.mode, mode);
                }
            }
        }
    }

    #[test]
    fn pack_layer2_header_carries_padding_private_copyright_emphasis() {
        // Each of the four "free-form" boolean fields toggles
        // independently and round-trips through FrameHeader::parse.
        use crate::header::{Emphasis as H, FrameHeader};
        let mut params = Layer2HeaderParams::new(48_000, 96, Mode::JointStereo);
        params.padding = true;
        params.private = true;
        params.copyright = true;
        params.original = false;
        params.emphasis = H::CcittJ17;
        params.mode_extension = ModeExtension(0b10); // bound = 12
        let bytes = pack_layer2_header(&params).expect("pack");
        let h = FrameHeader::parse(&bytes).expect("parse");
        assert!(h.padding);
        assert!(h.private);
        assert!(h.copyright);
        assert!(!h.original);
        assert_eq!(h.emphasis, H::CcittJ17);
        assert_eq!(h.mode_extension.0 & 0b11, 0b10);
        assert_eq!(h.mode, Mode::JointStereo);
    }

    #[test]
    fn write_layer2_header_matches_pack_layer2_header() {
        // The streaming BitWriter convenience and the byte-returning
        // pack function must emit identical bytes.
        let params = Layer2HeaderParams::new(44_100, 192, Mode::Stereo);
        let mut bw = BitWriter::new();
        write_layer2_header(&mut bw, &params).expect("write");
        // After 32 bits of header the byte buffer is exactly 4 bytes.
        let streamed = bw.finish();
        let packed = pack_layer2_header(&params).expect("pack");
        assert_eq!(streamed, packed.to_vec());
        assert_eq!(streamed.len(), 4);
    }

    #[test]
    fn write_layer2_header_then_more_bits_stays_msb_first() {
        // Append an extra 10 bits after the 32-bit header and confirm
        // they pack MSB-first into bytes 4..6 — exercises the
        // BitWriter interaction (the header must not leave the writer
        // with a partial trailing byte).
        let params = Layer2HeaderParams::new(44_100, 128, Mode::SingleChannel);
        let mut bw = BitWriter::new();
        write_layer2_header(&mut bw, &params).expect("write");
        assert_eq!(bw.byte_len(), 4, "32-bit header → 4 full bytes");
        // After the header, the next bits start at the MSB of byte 4.
        // Write 0b1010101010 (10 bits) — should become 0b10101010
        // (=0xAA) at byte 4 with the trailing 0b10 (zero-padded to
        // 0b1000_0000 = 0x80) at byte 5.
        bw.put(0b1010101010, 10);
        let bytes = bw.finish();
        assert_eq!(bytes.len(), 6);
        assert_eq!(bytes[4], 0xAA);
        assert_eq!(bytes[5], 0x80);
    }

    // ---- Layer II allocation-field writer (§2.4.1.6) -----------

    // Local FrameHeader builder for tests in this module — mirrors the
    // `header_mp1` helper used by the tables_layer2 tests, repeated here
    // so the encode tests don't depend on a private item from a sibling
    // module's tests submodule.
    fn header_l2(fs: u32, kbps: u16, mode: Mode) -> FrameHeader {
        FrameHeader {
            id: crate::header::Id::Mpeg,
            layer: crate::header::Layer::II,
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

    fn header_l2_joint(fs: u32, kbps: u16, bound_code: u8) -> FrameHeader {
        let mut h = header_l2(fs, kbps, Mode::JointStereo);
        h.mode_extension = crate::header::ModeExtension(bound_code & 0b11);
        h
    }

    #[test]
    fn layer2_stereo_bound_matches_decode_path() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        // Stereo / dual_channel / mono never share an upper band.
        let h_st = header_l2(44_100, 192, Mode::Stereo);
        let t_st = layer2_bit_allocation_table(&h_st);
        assert_eq!(layer2_stereo_bound(&h_st, t_st.sblimit()), t_st.sblimit());
        let h_du = header_l2(44_100, 192, Mode::DualChannel);
        assert_eq!(layer2_stereo_bound(&h_du, t_st.sblimit()), t_st.sblimit());
        let h_mo = header_l2(44_100, 64, Mode::SingleChannel);
        let t_mo = layer2_bit_allocation_table(&h_mo);
        assert_eq!(layer2_stereo_bound(&h_mo, t_mo.sblimit()), t_mo.sblimit());
        // Joint stereo: ModeExtension code → bound, clamped to sblimit.
        // 64 kbit/s stereo @ 44.1 kHz → B.2c (sblimit = 8).
        let h_js = header_l2_joint(44_100, 64, 0b00); // raw bound = 4
        let t_js = layer2_bit_allocation_table(&h_js);
        assert_eq!(layer2_stereo_bound(&h_js, t_js.sblimit()), 4);
        let h_js2 = header_l2_joint(44_100, 64, 0b11); // raw bound = 16 → clamp to sblimit = 8
        assert_eq!(layer2_stereo_bound(&h_js2, t_js.sblimit()), t_js.sblimit());
    }

    #[test]
    fn write_layer2_allocation_field_stereo_zero_bits_match_sum_nbal() {
        // An all-zero allocation under any (Fs, bitrate, mode) writes
        // exactly `nch · Σ nbal[sb]` zero bits — the `bbal` budget the
        // §C.1.5.2.7 formula deducts from the frame payload.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let nch = header.channels() as usize;
        let bound = layer2_stereo_bound(&header, table.sblimit());
        let mut bw = BitWriter::new();
        write_layer2_allocation_field(&mut bw, table, &alloc, nch, bound).expect("write");
        // bbal in bits, since alloc field bits sum exactly to nch·Σ nbal
        // when bound == sblimit (stereo, no shared upper band).
        let expected_bits = nch * sum_nbal_per_channel(table);
        let bytes = bw.finish();
        // All zero contents.
        assert!(bytes.iter().all(|&b| b == 0));
        // The packed size must be ceil(expected_bits / 8).
        assert_eq!(bytes.len(), expected_bits.div_ceil(8));
    }

    #[test]
    fn write_layer2_allocation_field_known_low_band_pattern() {
        // Take Table B.2a (sblimit = 27): sb 0..11 have nbal = 4, sb
        // 11..23 have nbal = 3, sb 23..27 have nbal = 2. Pick a few
        // non-zero codes and confirm the bitstream layout reads them
        // back in the same order with sb-major, ch-minor packing
        // (§2.4.1.6 low-band loop). 48 kHz @ 192 kbit/s stereo (96
        // kbit/s/channel) lands on B.2a per the §2.4.2.3 footnote.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(48_000, 192, Mode::Stereo); // → B.2a, sblimit = 27
        let table = layer2_bit_allocation_table(&header);
        assert_eq!(table.sblimit(), 27);
        assert_eq!(table.nbal(0), 4);
        assert_eq!(table.nbal(11), 3);
        assert_eq!(table.nbal(23), 2);
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        alloc[0][0] = 0b0011; // valid: small allocation
        alloc[1][0] = 0b0101;
        alloc[0][11] = 0b010; // 3-bit code
        alloc[1][11] = 0b001;
        alloc[0][23] = 0b10; // 2-bit code
        alloc[1][23] = 0b01;
        let mut bw = BitWriter::new();
        write_layer2_allocation_field(&mut bw, table, &alloc, 2, table.sblimit()).expect("write");
        let bytes = bw.finish();
        // sb=0, ch=0: high 4 bits of byte 0 = 0b0011 = 0x3.
        assert_eq!(bytes[0] >> 4, 0b0011);
        // sb=0, ch=1: low 4 bits of byte 0 = 0b0101 = 0x5.
        assert_eq!(bytes[0] & 0xF, 0b0101);
    }

    #[test]
    fn write_layer2_allocation_field_joint_stereo_shared_upper_band() {
        // joint_stereo with bound = 4 (mode_extension '00'): subbands
        // [0, 4) write nch × nbal bits, subbands [4, sblimit) write
        // exactly one nbal bits each. Verify the total payload bits
        // matches that breakdown.
        use crate::tables_layer2::layer2_bit_allocation_table;
        // 32 kbit/s mono @ 44.1 kHz puts us on B.2c (sblimit = 8). To
        // exercise joint_stereo we need a stereo header; 64 kbit/s
        // stereo @ 44.1 kHz also resolves to B.2c (32 kbit/s/channel).
        let header = header_l2_joint(44_100, 64, 0b00); // bound = 4
        let table = layer2_bit_allocation_table(&header);
        let sblimit = table.sblimit();
        let bound = layer2_stereo_bound(&header, sblimit);
        assert!(
            bound > 0 && bound < sblimit,
            "bound={bound} sblimit={sblimit}"
        );
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let mut bw = BitWriter::new();
        write_layer2_allocation_field(&mut bw, table, &alloc, 2, bound).expect("write");
        let total_bits: usize = (0..bound)
            .map(|sb| 2 * table.nbal(sb) as usize)
            .sum::<usize>()
            + (bound..sblimit)
                .map(|sb| table.nbal(sb) as usize)
                .sum::<usize>();
        let bytes = bw.finish();
        assert_eq!(bytes.len(), total_bits.div_ceil(8));
        // Smaller than the unshared payload by exactly Σ_{bound..sblimit} nbal[sb].
        let unshared: usize = (0..sblimit).map(|sb| 2 * table.nbal(sb) as usize).sum();
        assert_eq!(
            unshared - total_bits,
            (bound..sblimit)
                .map(|sb| table.nbal(sb) as usize)
                .sum::<usize>()
        );
    }

    #[test]
    fn write_layer2_allocation_field_rejects_bad_channel_count() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let mut bw = BitWriter::new();
        let err = write_layer2_allocation_field(&mut bw, table, &alloc, 0, table.sblimit())
            .expect_err("nch=0");
        assert_eq!(err, Layer2AllocationFieldError::UnsupportedChannelCount(0));
        let err = write_layer2_allocation_field(&mut bw, table, &alloc, 3, table.sblimit())
            .expect_err("nch=3");
        assert_eq!(err, Layer2AllocationFieldError::UnsupportedChannelCount(3));
    }

    #[test]
    fn write_layer2_allocation_field_rejects_bound_above_sblimit() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo); // B.2b, sblimit = 30
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let mut bw = BitWriter::new();
        let err = write_layer2_allocation_field(&mut bw, table, &alloc, 2, table.sblimit() + 1)
            .expect_err("bound > sblimit");
        assert!(matches!(
            err,
            Layer2AllocationFieldError::BoundExceedsSblimit { .. }
        ));
    }

    #[test]
    fn write_layer2_allocation_field_rejects_mono_with_shared_upper_band() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 64, Mode::SingleChannel);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let mut bw = BitWriter::new();
        // Mono must use bound == sblimit (no upper-band sharing exists
        // because there are no channels to share between).
        let err = write_layer2_allocation_field(&mut bw, table, &alloc, 1, 4)
            .expect_err("mono bound < sblimit");
        assert!(matches!(
            err,
            Layer2AllocationFieldError::MonoBoundBelowSblimit { .. }
        ));
    }

    #[test]
    fn write_layer2_allocation_field_rejects_oversize_allocation_code() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        // sb=11 has nbal=3 under B.2b; values >= 8 don't fit.
        assert_eq!(table.nbal(11), 3);
        alloc[0][11] = 8;
        let mut bw = BitWriter::new();
        let err = write_layer2_allocation_field(&mut bw, table, &alloc, 2, table.sblimit())
            .expect_err("alloc too wide");
        assert_eq!(
            err,
            Layer2AllocationFieldError::InvalidAllocationCode {
                channel: 0,
                subband: 11,
                allocation: 8,
            }
        );
    }

    #[test]
    fn write_layer2_allocation_field_rejects_non_zero_above_sblimit() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        // B.2c stereo @ 44.1 kHz (64 kbit/s) → sblimit = 8.
        let header = header_l2(44_100, 64, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        assert_eq!(table.sblimit(), 8);
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        alloc[1][20] = 1; // above sblimit
        let mut bw = BitWriter::new();
        let err = write_layer2_allocation_field(&mut bw, table, &alloc, 2, table.sblimit())
            .expect_err("above sblimit");
        assert!(matches!(
            err,
            Layer2AllocationFieldError::NonZeroAllocationAboveSblimit { .. }
        ));
    }

    #[test]
    fn write_layer2_allocation_field_rejects_upper_band_disagreement() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2_joint(44_100, 64, 0b00); // bound = 4
        let table = layer2_bit_allocation_table(&header);
        let sblimit = table.sblimit();
        let bound = layer2_stereo_bound(&header, sblimit);
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        // Pick an upper-band subband and give the two channels different
        // codes — the writer refuses to silently collapse them.
        alloc[0][bound] = 1;
        alloc[1][bound] = 2;
        let mut bw = BitWriter::new();
        let err =
            write_layer2_allocation_field(&mut bw, table, &alloc, 2, bound).expect_err("disagree");
        assert!(matches!(
            err,
            Layer2AllocationFieldError::UpperBandChannelsDisagree {
                subband: _,
                left: 1,
                right: 2
            }
        ));
    }

    #[test]
    fn write_layer2_allocation_field_round_trips_through_decoder_read() {
        // Encode every (ch, sb) allocation, then decode the bits back
        // using the same MSB-first BitReader semantics and recover the
        // exact codes. Exercises both the low-band and upper-band paths.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2_joint(44_100, 64, 0b01); // bound = 8 → clamped to sblimit=8
        let table = layer2_bit_allocation_table(&header);
        let sblimit = table.sblimit();
        let bound = layer2_stereo_bound(&header, sblimit);
        // For each subband, pick the smallest legal non-zero allocation
        // (smallest valid Table 3-B.2x level) and the symmetric shared
        // value in [bound, sblimit). Smallest legal index is the first
        // `Some` entry in the row's levels array; alloc = idx + 1.
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        for sb in 0..sblimit {
            let nbal = table.nbal(sb);
            if nbal == 0 {
                continue;
            }
            // Find the smallest alloc > 0 with a valid quant_class.
            let mut chosen = 0u8;
            for a in 1u8..(1u8 << nbal) {
                if table.quant_class(sb, a).is_some() {
                    chosen = a;
                    break;
                }
            }
            if sb < bound {
                alloc[0][sb] = chosen;
                alloc[1][sb] = chosen;
            } else {
                // Shared: both channels must hold the same code, but the
                // writer only emits one slot.
                alloc[0][sb] = chosen;
                alloc[1][sb] = chosen;
            }
        }
        let mut bw = BitWriter::new();
        write_layer2_allocation_field(&mut bw, table, &alloc, 2, bound).expect("write");
        let bytes = bw.finish();
        // Replay the §2.4.1.6 low-band per-channel read followed by the
        // shared upper-band read; every recovered code must match the
        // value we wrote (low band per channel, upper band single read
        // mirrored into both channels).
        let mut reader = crate::decode::BitReader::new(&bytes);
        for sb in 0..bound {
            let nbal = table.nbal(sb);
            for ch in 0..2 {
                let got = reader.read_bits(nbal).expect("read low") as u8;
                assert_eq!(got, alloc[ch][sb], "sb={sb} ch={ch}");
            }
        }
        for sb in bound..sblimit {
            let nbal = table.nbal(sb);
            let got = reader.read_bits(nbal).expect("read upper") as u8;
            assert_eq!(got, alloc[0][sb], "shared sb={sb}");
            assert_eq!(got, alloc[1][sb], "shared sb={sb}");
        }
    }

    // ---- §2.4.1.6 scfsi + scalefactor field writer ----------------

    /// Helper: build a stereo allocation with the smallest legal
    /// non-zero code in every subband below `sblimit`, mirrored across
    /// channels in `[bound, sblimit)`. Used by the scfsi-and-scalefactor
    /// tests below so every (ch, sb) carries an allocation and therefore
    /// emits scfsi + scalefactor bits.
    fn dense_layer2_alloc(table: &AllocationTable, bound: usize, nch: usize) -> Layer2Allocation {
        let sblimit = table.sblimit();
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        for sb in 0..sblimit {
            let nbal = table.nbal(sb);
            if nbal == 0 {
                continue;
            }
            let mut chosen = 0u8;
            for a in 1u8..(1u8 << nbal) {
                if table.quant_class(sb, a).is_some() {
                    chosen = a;
                    break;
                }
            }
            for ch in 0..nch {
                if sb < bound {
                    alloc[ch][sb] = chosen;
                } else {
                    // Shared upper band: write the same value to every
                    // channel so the alloc-field writer accepts it; the
                    // scalefactor writer still reads per-channel scfsi.
                    alloc[ch][sb] = chosen;
                }
            }
        }
        alloc
    }

    #[test]
    fn write_layer2_scalefactor_field_zero_alloc_writes_no_bits() {
        // When every subband is unallocated the §2.4.1.6 scfsi and
        // scalefactor fields are empty: the writer emits zero bits.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Layer2ScalefactorFieldInput::default();
        let mut bw = BitWriter::new();
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 2, table.sblimit())
            .expect("write");
        // No data byte should be buffered when no bits were appended.
        assert_eq!(bw.byte_len(), 0);
        let bytes = bw.finish();
        assert!(bytes.is_empty());
    }

    #[test]
    fn write_layer2_scalefactor_field_bit_count_matches_schedule() {
        // For a dense stereo allocation the §2.4.1.6 scfsi field is
        // 2 bits per (ch, sb) with non-zero allocation, and the scfs
        // field is { 0b00 → 18, 0b01 → 12, 0b10 → 6, 0b11 → 12 } bits
        // per (ch, sb). Verify the writer's total byte count tracks
        // that exact sum.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo); // B.2b, sblimit=30
        let table = layer2_bit_allocation_table(&header);
        let nch = 2usize;
        let bound = layer2_stereo_bound(&header, table.sblimit());
        let alloc = dense_layer2_alloc(table, bound, nch);
        // Mix scfsi codes across (ch, sb): cycle through 00, 01, 10, 11.
        let mut input = Layer2ScalefactorFieldInput::default();
        let mut expected_bits = 0usize;
        let mut k = 0u8;
        for sb in 0..table.sblimit() {
            for ch in 0..nch {
                if alloc[ch][sb] == 0 {
                    continue;
                }
                let code = k & 0b11;
                input.scfsi[ch][sb] = code;
                // Keep all three parts equal so any schedule is legal,
                // including 0b10 (single-value broadcast).
                let v = 7u8;
                input.scalefactor_indices[ch][sb] = [v, v, v];
                expected_bits += 2; // scfsi
                expected_bits += match code {
                    0b00 => 18,
                    0b01 => 12,
                    0b10 => 6,
                    _ => 12, // 0b11
                };
                k = k.wrapping_add(1);
            }
        }
        let mut bw = BitWriter::new();
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, nch, bound).expect("write");
        let bytes = bw.finish();
        assert_eq!(bytes.len(), expected_bits.div_ceil(8));
    }

    #[test]
    fn write_layer2_scalefactor_field_round_trips_against_bitreader() {
        // The §2.4.1.6 decoder reads scfsi (sb-major / ch-minor over all
        // sb < sblimit, two bits per non-zero allocation) then the
        // scalefactors per the §2.4.2.6 schedule. Replay both phases
        // through the BitReader and confirm every value we wrote
        // round-trips bit-exact under every scfsi code.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(48_000, 192, Mode::Stereo); // B.2a, sblimit=27
        let table = layer2_bit_allocation_table(&header);
        let sblimit = table.sblimit();
        let nch = 2usize;
        let bound = layer2_stereo_bound(&header, sblimit);
        let alloc = dense_layer2_alloc(table, bound, nch);
        // Synthetic but legal input: alternate scfsi values, with the
        // per-part array set so every schedule's "collapse" rule is
        // satisfied.
        let mut input = Layer2ScalefactorFieldInput::default();
        let scfsi_cycle = [0b00u8, 0b01, 0b10, 0b11];
        let mut idx = 0usize;
        for sb in 0..sblimit {
            for ch in 0..nch {
                if alloc[ch][sb] == 0 {
                    continue;
                }
                let s = scfsi_cycle[idx % 4];
                input.scfsi[ch][sb] = s;
                // Pick parts that satisfy the §2.4.2.6 collapse rules.
                let p0 = ((idx * 3 + 7) % 60) as u8;
                let p1 = ((idx * 5 + 11) % 60) as u8;
                let p2 = ((idx * 7 + 13) % 60) as u8;
                input.scalefactor_indices[ch][sb] = match s {
                    0b00 => [p0, p1, p2],
                    0b01 => [p0, p0, p2], // parts 0 and 1 equal
                    0b10 => [p0, p0, p0], // all three equal
                    _ => [p0, p1, p1],    // parts 1 and 2 equal
                };
                idx += 1;
            }
        }
        let mut bw = BitWriter::new();
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, nch, bound).expect("write");
        let bytes = bw.finish();
        let mut reader = crate::decode::BitReader::new(&bytes);
        // Phase 1: scfsi.
        let mut got_scfsi = [[0u8; SUBBANDS]; 2];
        for sb in 0..sblimit {
            for ch in 0..nch {
                if alloc[ch][sb] != 0 {
                    got_scfsi[ch][sb] = reader.read_bits(2).expect("read scfsi") as u8;
                    assert_eq!(got_scfsi[ch][sb], input.scfsi[ch][sb], "sb={sb} ch={ch}");
                }
            }
        }
        // Phase 2: scalefactors per the §2.4.2.6 schedule. Reconstruct
        // the full per-part triplet the decoder would expose and check
        // it equals what we fed the writer.
        for sb in 0..sblimit {
            for ch in 0..nch {
                if alloc[ch][sb] == 0 {
                    continue;
                }
                let s = got_scfsi[ch][sb];
                let got_parts = match s {
                    0b00 => {
                        let a = reader.read_bits(6).expect("p0") as u8;
                        let b = reader.read_bits(6).expect("p1") as u8;
                        let c = reader.read_bits(6).expect("p2") as u8;
                        [a, b, c]
                    }
                    0b01 => {
                        let a = reader.read_bits(6).expect("p01") as u8;
                        let c = reader.read_bits(6).expect("p2") as u8;
                        [a, a, c]
                    }
                    0b10 => {
                        let a = reader.read_bits(6).expect("p012") as u8;
                        [a, a, a]
                    }
                    _ => {
                        let a = reader.read_bits(6).expect("p0") as u8;
                        let b = reader.read_bits(6).expect("p12") as u8;
                        [a, b, b]
                    }
                };
                assert_eq!(
                    got_parts, input.scalefactor_indices[ch][sb],
                    "sb={sb} ch={ch} scfsi={s}"
                );
            }
        }
    }

    #[test]
    fn write_layer2_scalefactor_field_known_two_subband_pattern() {
        // Hand-trace the smallest possible scfsi+scalefactor region:
        // a mono frame whose only allocated subband is sb=0 with
        // scfsi=0b00 and parts {1, 2, 3}. Expected bitstream is
        // 2 + 18 = 20 bits: `00 000001 000010 000011`, packed MSB-first
        // into ceil(20/8) = 3 bytes.
        use crate::tables_layer2::layer2_bit_allocation_table;
        // Pick a mono header that lands on B.2c so the smallest legal
        // allocation in sb=0 is straightforward.
        let header = header_l2(44_100, 64, Mode::SingleChannel); // → B.2c
        let table = layer2_bit_allocation_table(&header);
        let nch = 1usize;
        let bound = table.sblimit();
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        // Smallest legal code in sb=0 (first Some in the row).
        let nbal0 = table.nbal(0);
        let mut chosen = 0u8;
        for a in 1u8..(1u8 << nbal0) {
            if table.quant_class(0, a).is_some() {
                chosen = a;
                break;
            }
        }
        alloc[0][0] = chosen;
        let mut input = Layer2ScalefactorFieldInput::default();
        input.scfsi[0][0] = 0b00;
        input.scalefactor_indices[0][0] = [1, 2, 3];
        let mut bw = BitWriter::new();
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, nch, bound).expect("write");
        let bytes = bw.finish();
        // 20 bits → 3 bytes.
        assert_eq!(bytes.len(), 3);
        // Reconstruct the expected stream MSB-first:
        //   scfsi=00       (2 bits)  → '00'
        //   part0=1=000001 (6 bits)
        //   part1=2=000010 (6 bits)
        //   part2=3=000011 (6 bits)
        // Concatenated: '00 000001 000010 000011' = 20 bits.
        // Pack MSB-first into 24 bits, low 4 bits zero-padded:
        //   byte0 = 0b0000_0001 = 0x01
        //   byte1 = 0b0000_0010 = 0x02 ... wait, need to lay out carefully.
        // Bit stream (left-to-right):
        // 00 000001 000010 000011 0000
        // Group into bytes of 8:
        // 00000001 00001000 00110000
        // = 0x01 0x08 0x30
        assert_eq!(bytes[0], 0x01);
        assert_eq!(bytes[1], 0x08);
        assert_eq!(bytes[2], 0x30);
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_bad_channel_count() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Layer2ScalefactorFieldInput::default();
        let mut bw = BitWriter::new();
        let err =
            write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 0, table.sblimit())
                .expect_err("nch=0");
        assert_eq!(err, Layer2ScalefactorFieldError::UnsupportedChannelCount(0));
        let err =
            write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 3, table.sblimit())
                .expect_err("nch=3");
        assert_eq!(err, Layer2ScalefactorFieldError::UnsupportedChannelCount(3));
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_bound_above_sblimit() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Layer2ScalefactorFieldInput::default();
        let mut bw = BitWriter::new();
        let err =
            write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 2, table.sblimit() + 1)
                .expect_err("bound > sblimit");
        assert!(matches!(
            err,
            Layer2ScalefactorFieldError::BoundExceedsSblimit { .. }
        ));
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_mono_with_shared_upper_band() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 64, Mode::SingleChannel);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Layer2ScalefactorFieldInput::default();
        let mut bw = BitWriter::new();
        let err = write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 1, 4)
            .expect_err("mono bound < sblimit");
        assert!(matches!(
            err,
            Layer2ScalefactorFieldError::MonoBoundBelowSblimit { .. }
        ));
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_oversize_scfsi_code() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = layer2_stereo_bound(&header, table.sblimit());
        let alloc = dense_layer2_alloc(table, bound, 2);
        let mut input = Layer2ScalefactorFieldInput::default();
        // 4 doesn't fit in 2 bits.
        input.scfsi[1][0] = 4;
        let mut bw = BitWriter::new();
        let err = write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 2, bound)
            .expect_err("scfsi=4");
        assert_eq!(
            err,
            Layer2ScalefactorFieldError::InvalidScfsiCode {
                channel: 1,
                subband: 0,
                scfsi: 4,
            }
        );
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_invalid_scalefactor_index() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = layer2_stereo_bound(&header, table.sblimit());
        let alloc = dense_layer2_alloc(table, bound, 2);
        let mut input = Layer2ScalefactorFieldInput::default();
        // 63 is reserved per §2.4.3.2 prose.
        input.scalefactor_indices[0][0][2] = 63;
        let mut bw = BitWriter::new();
        let err = write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 2, bound)
            .expect_err("scf=63");
        assert_eq!(
            err,
            Layer2ScalefactorFieldError::InvalidScalefactorIndex {
                channel: 0,
                subband: 0,
                part: 2,
                index: 63,
            }
        );
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_inconsistent_scfsi_01() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = layer2_stereo_bound(&header, table.sblimit());
        let alloc = dense_layer2_alloc(table, bound, 2);
        let mut input = Layer2ScalefactorFieldInput::default();
        input.scfsi[0][0] = 0b01;
        // scfsi=01 broadcasts part0 over parts 0+1: differing part0/part1
        // means the writer would lose part1.
        input.scalefactor_indices[0][0] = [3, 5, 7];
        let mut bw = BitWriter::new();
        let err = write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 2, bound)
            .expect_err("inconsistent 01");
        assert_eq!(
            err,
            Layer2ScalefactorFieldError::ScfsiPartsInconsistent01 {
                channel: 0,
                subband: 0,
                part0: 3,
                part1: 5,
            }
        );
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_inconsistent_scfsi_10() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = layer2_stereo_bound(&header, table.sblimit());
        let alloc = dense_layer2_alloc(table, bound, 2);
        let mut input = Layer2ScalefactorFieldInput::default();
        input.scfsi[0][0] = 0b10;
        // scfsi=10 broadcasts part0 over all three parts: any divergence
        // means information would be silently lost.
        input.scalefactor_indices[0][0] = [3, 3, 4];
        let mut bw = BitWriter::new();
        let err = write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 2, bound)
            .expect_err("inconsistent 10");
        assert_eq!(
            err,
            Layer2ScalefactorFieldError::ScfsiPartsInconsistent10 {
                channel: 0,
                subband: 0,
                parts: [3, 3, 4],
            }
        );
    }

    #[test]
    fn write_layer2_scalefactor_field_rejects_inconsistent_scfsi_11() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = layer2_stereo_bound(&header, table.sblimit());
        let alloc = dense_layer2_alloc(table, bound, 2);
        let mut input = Layer2ScalefactorFieldInput::default();
        input.scfsi[0][0] = 0b11;
        // scfsi=11 broadcasts part1 over parts 1+2: differing values lose
        // information.
        input.scalefactor_indices[0][0] = [3, 5, 7];
        let mut bw = BitWriter::new();
        let err = write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 2, bound)
            .expect_err("inconsistent 11");
        assert_eq!(
            err,
            Layer2ScalefactorFieldError::ScfsiPartsInconsistent11 {
                channel: 0,
                subband: 0,
                part1: 5,
                part2: 7,
            }
        );
    }

    #[test]
    fn write_layer2_scalefactor_field_skips_unallocated_subbands() {
        // Only sb=0 is allocated in a mono frame; only that subband
        // contributes scfsi + scalefactor bits. The writer must emit
        // no bits for the (many) unallocated subbands.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 64, Mode::SingleChannel);
        let table = layer2_bit_allocation_table(&header);
        let bound = table.sblimit();
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let nbal0 = table.nbal(0);
        for a in 1u8..(1u8 << nbal0) {
            if table.quant_class(0, a).is_some() {
                alloc[0][0] = a;
                break;
            }
        }
        let mut input = Layer2ScalefactorFieldInput::default();
        input.scfsi[0][0] = 0b10; // single scalefactor read
        input.scalefactor_indices[0][0] = [9, 9, 9];
        let mut bw = BitWriter::new();
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 1, bound).expect("write");
        let bytes = bw.finish();
        // 2 scfsi bits + 6 scf bits = 8 bits = exactly one byte.
        assert_eq!(bytes.len(), 1);
        // Stream MSB-first: '10 001001' = 0b10001001 = 0x89.
        assert_eq!(bytes[0], 0x89);
    }

    // ---- §C.1.5.1.4 Layer II scalefactor extraction -------------------

    fn zero_layer2_subbands() -> Box<[[[f64; 36]; SUBBANDS]; 2]> {
        Box::new([[[0.0f64; 36]; SUBBANDS]; 2])
    }

    #[test]
    fn layer2_subband_peak_per_part_splits_36_slots_into_three_12_blocks() {
        // Per-part peaks must come from disjoint 12-slot windows: part 0
        // sees only slots 0..12, part 1 only 12..24, part 2 only 24..36.
        // Stamping a distinct peak into each window and zero elsewhere
        // proves the windowing is correct.
        let mut subbands = zero_layer2_subbands();
        // sb=0, ch=0: peak 0.25 in part 0 (slot 5), 0.50 in part 1
        // (slot 17), 0.10 in part 2 (slot 30). Other slots stay zero.
        subbands[0][0][5] = 0.25;
        subbands[0][0][17] = -0.50; // sign should not matter (absolute)
        subbands[0][0][30] = 0.10;
        // sb=7, ch=1: only part 2 carries energy at slot 24.
        subbands[1][7][24] = 0.75;
        let peaks = layer2_subband_peak_per_part(&subbands, 2, SUBBANDS);
        assert_eq!(peaks[0][0][0], 0.25);
        assert_eq!(peaks[0][0][1], 0.50);
        assert_eq!(peaks[0][0][2], 0.10);
        assert_eq!(peaks[1][7][0], 0.0);
        assert_eq!(peaks[1][7][1], 0.0);
        assert_eq!(peaks[1][7][2], 0.75);
        // Untouched (ch, sb) cells remain zero.
        assert_eq!(peaks[0][5][0], 0.0);
        assert_eq!(peaks[1][0][2], 0.0);
    }

    #[test]
    fn layer2_subband_peak_per_part_respects_nch_and_sblimit() {
        // Cells outside `nch` rows or `0..sblimit` columns must not be
        // sampled — even when data is sitting there in the input array.
        let mut subbands = zero_layer2_subbands();
        // ch=1 should be ignored when nch == 1.
        subbands[1][0][0] = 0.9;
        // sb=20 should be ignored when sblimit == 8.
        subbands[0][20][0] = 0.9;
        let peaks = layer2_subband_peak_per_part(&subbands, 1, 8);
        assert_eq!(peaks[1][0][0], 0.0, "ch=1 must be skipped when nch=1");
        assert_eq!(peaks[0][20][0], 0.0, "sb=20 must be skipped when sblimit=8");
    }

    #[test]
    fn select_layer2_scalefactors_matches_layer_i_helper_per_part() {
        // For each (ch, sb, part) the chosen Table 3-B.1 index must
        // equal what `select_scalefactor` returns when applied to that
        // part's peak in isolation — the Layer II extractor is exactly
        // §C.1.5.1.4 applied three times (once per scalefactor part).
        let mut subbands = zero_layer2_subbands();
        subbands[0][3][2] = 0.9; // part 0, sb=3, ch=0
        subbands[0][3][15] = 0.001; // part 1
        subbands[0][3][27] = 1.5; // part 2 (over the SCF[1]=1.587… edge)
        let idx = select_layer2_scalefactors(&subbands, 1, SUBBANDS);
        // Cross-check each part's index against the per-part peak fed
        // through the Layer I helper.
        assert_eq!(idx[0][3][0], select_scalefactor(0.9));
        assert_eq!(idx[0][3][1], select_scalefactor(0.001));
        assert_eq!(idx[0][3][2], select_scalefactor(1.5));
    }

    #[test]
    fn select_layer2_scalefactors_zero_signal_picks_tiniest_index() {
        // An all-zero analysed sub-band trace must collapse to the
        // tiniest-multiplier index (62, value ≈ 1.2e-6 — the smallest
        // Table 3-B.1 multiplier still > 0). Index 63 is reserved and
        // never produced.
        let subbands = zero_layer2_subbands();
        let idx = select_layer2_scalefactors(&subbands, 2, SUBBANDS);
        for ch in 0..2 {
            for sb in 0..SUBBANDS {
                for part in 0..3 {
                    assert_eq!(
                        idx[ch][sb][part], 62,
                        "zero-signal (ch={ch}, sb={sb}, part={part}) must pick index 62"
                    );
                }
            }
        }
    }

    #[test]
    fn select_layer2_scalefactors_feeds_write_layer2_scalefactor_field() {
        // The extractor output is shaped exactly to drop into
        // [`Layer2ScalefactorFieldInput::scalefactor_indices`]. Build a
        // small canonical fixture (mono, sb=0, three distinct per-part
        // peaks that round to three distinct Table 3-B.1 indices), feed
        // the extractor → field-writer chain, and assert the bytes
        // round-trip back to the same indices through the decoder
        // [`crate::decode::BitReader`] when written with `scfsi == 0b00`.
        use crate::decode::BitReader;
        use crate::tables_layer2::layer2_bit_allocation_table;

        // Three distinct part peaks that span the SCF table:
        //   • part 0 peak 0.9 → SCF[3] = 1.0   → index 3
        //   • part 1 peak 0.4 → SCF[5] = 0.5   → index 5 (next > 0.4)
        //   • part 2 peak 0.1 → SCF[9] = 0.25 / SCF[10] = 0.198 / etc.
        // We don't hard-code the exact indices — we re-derive them via
        // `select_scalefactor` to keep the test honest against any
        // future Table 3-B.1 audit. The cardinal property under test is
        // that the SAME indices flow through the §2.4.1.6 field writer.
        let mut subbands = zero_layer2_subbands();
        subbands[0][0][0] = 0.9; // part 0
        subbands[0][0][13] = 0.4; // part 1
        subbands[0][0][29] = 0.1; // part 2
        let idx = select_layer2_scalefactors(&subbands, 1, SUBBANDS);
        let expect = [
            select_scalefactor(0.9),
            select_scalefactor(0.4),
            select_scalefactor(0.1),
        ];
        assert_eq!(idx[0][0], expect);

        // Allocate only sb=0 so the writer emits exactly one
        // (scfsi, scf0, scf1, scf2) tuple — 2 + 3·6 = 20 bits.
        let header = header_l2(44_100, 64, Mode::SingleChannel);
        let table = layer2_bit_allocation_table(&header);
        let bound = table.sblimit();
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let nbal0 = table.nbal(0);
        for a in 1u8..(1u8 << nbal0) {
            if table.quant_class(0, a).is_some() {
                alloc[0][0] = a;
                break;
            }
        }
        let mut input = Layer2ScalefactorFieldInput::default();
        input.scfsi[0][0] = 0b00; // emit all three parts
        input.scalefactor_indices[0][0] = idx[0][0];
        let mut bw = BitWriter::new();
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &input, 1, bound).expect("write");
        let bytes = bw.finish();
        // Exactly 20 bits → 3 bytes with 4 trailing pad bits.
        assert_eq!(bytes.len(), 3);
        // Read it back as the decoder would.
        let mut rd = BitReader::new(&bytes);
        let scfsi = rd.read_bits(2).expect("scfsi");
        assert_eq!(scfsi, 0b00);
        for part in 0..3 {
            let i = rd.read_bits(6).expect("scf") as u8;
            assert_eq!(
                i, idx[0][0][part],
                "round-trip part={part} expected={} got={i}",
                idx[0][0][part]
            );
        }
    }

    // ---- §2.4.1.6 / §2.4.3.3.4 SAMPLES region writer ------------------

    #[test]
    fn write_layer2_samples_field_zero_alloc_writes_no_bits() {
        // When every subband is unallocated the §2.4.1.6 SAMPLES region
        // is empty: 12 granules × 0 emissions = 0 bits.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Box::new(Layer2SamplesFieldInput::default());
        let mut bw = BitWriter::new();
        write_layer2_samples_field(&mut bw, table, &alloc, &input, 2, table.sblimit())
            .expect("write");
        assert_eq!(bw.byte_len(), 0);
        let bytes = bw.finish();
        assert!(bytes.is_empty());
    }

    #[test]
    fn write_layer2_samples_field_bit_count_matches_class_sums() {
        // Dense stereo allocation, every subband allocated at the smallest
        // legal level. Total bits = 12 (granules) × Σ_(ch, sb)
        // class.bits_per_codeword (per the §2.4.1.6 syntax — one codeword
        // per triplet, whether grouped or split into three samples).
        // (For non-grouped classes the per-triplet emission is 3 *
        // bits_per_codeword; pre-multiply by samples_per_codeword for the
        // grouped flavour where one codeword covers all three samples.)
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = table.sblimit();
        let alloc = dense_layer2_alloc(table, bound, 2);

        // Build the expected bit count by walking every (ch, sb) and
        // looking up the class's bits-per-triplet contribution.
        let mut expected_per_granule_bits = 0usize;
        for ch in 0..2 {
            for sb in 0..table.sblimit() {
                let a = alloc[ch][sb];
                if a == 0 {
                    continue;
                }
                let cls = table.quant_class(sb, a).expect("class");
                let per_triplet_bits = if cls.grouping {
                    cls.bits_per_codeword as usize
                } else {
                    3 * cls.bits_per_codeword as usize
                };
                expected_per_granule_bits += per_triplet_bits;
            }
        }
        let expected_total_bits = 12 * expected_per_granule_bits;

        let input = Box::new(Layer2SamplesFieldInput::default()); // all-zero codes
        let mut bw = BitWriter::new();
        write_layer2_samples_field(&mut bw, table, &alloc, &input, 2, bound).expect("write");
        // Use byte_len + flush remainder for an exact bit count.
        let written_full_bytes = bw.byte_len();
        let bytes = bw.finish();
        let total_bits = bytes.len() * 8;
        let pad_bits = total_bits - expected_total_bits;
        assert!(
            pad_bits < 8,
            "padding must be < 8 bits; got total_bits={total_bits} expected={expected_total_bits}"
        );
        assert_eq!(
            written_full_bytes + if pad_bits == 0 { 0 } else { 1 },
            bytes.len(),
            "finish() padded a partial trailing byte if any"
        );
    }

    #[test]
    fn write_layer2_samples_field_grouped_known_pattern() {
        // 64 kbit/s mono at 44.1 kHz selects B.2c (sblimit=8). B.2c sb0
        // row's smallest legal non-zero allocation maps to a grouped
        // class (nlevels=3, bits_per_codeword=5). With every (ch, sb)
        // other than sb=0 unallocated, the 12 emitted samplecodes are
        // 5 bits each = 60 bits = 7.5 bytes — `finish` rounds up to 8
        // bytes. With codes [s0=2, s1=1, s2=0] the packed value is
        // `2 + 1*3 + 0*9 = 5 = 0b00101`, repeated 12 times.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 64, Mode::SingleChannel);
        let table = layer2_bit_allocation_table(&header);
        let bound = table.sblimit();
        // Pick smallest non-zero alloc at sb=0.
        let nbal0 = table.nbal(0);
        let mut chosen = 0u8;
        for a in 1u8..(1u8 << nbal0) {
            if table.quant_class(0, a).is_some() {
                chosen = a;
                break;
            }
        }
        let cls = table.quant_class(0, chosen).expect("class");
        // The smallest non-zero level in B.2c sb0 must be a grouped
        // class (nlevels = 3, bits_per_codeword = 5).
        assert!(cls.grouping, "expected sb=0 smallest alloc to be grouped");
        assert_eq!(cls.nlevels, 3);
        assert_eq!(cls.bits_per_codeword, 5);

        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        alloc[0][0] = chosen;
        let mut input = Box::new(Layer2SamplesFieldInput::default());
        for gr in 0..12 {
            input.codes[0][gr][0] = [2, 1, 0];
        }
        let mut bw = BitWriter::new();
        write_layer2_samples_field(&mut bw, table, &alloc, &input, 1, bound).expect("write");
        let bytes = bw.finish();
        // 12 codewords × 5 bits = 60 bits → 8 bytes after the 4-bit
        // zero pad on the trailing byte.
        assert_eq!(bytes.len(), 8);
        // Each codeword is 0b00101. The MSB-first stream is
        //   00101 00101 00101 00101 00101 00101 00101 00101 00101 00101 00101 00101 0000
        // = 0x29 0x4A 0x52 0x94 0xA5 0x29 0x4A 0x50
        // (the trailing nibble of byte 7 is zero pad).
        // Verify byte-by-byte.
        let expected: [u8; 8] = [0x29, 0x4A, 0x52, 0x94, 0xA5, 0x29, 0x4A, 0x50];
        assert_eq!(bytes, expected);
    }

    #[test]
    fn write_layer2_samples_field_round_trips_through_decoder() {
        // End-to-end: write a stereo SAMPLES region with a deterministic
        // set of codes per (ch, sb, gr), then route the bytes through
        // `decode_layer2_audio_data` (after a fresh allocation+scfsi+scf
        // prefix). Recovered samples must match the expected
        // requantization of the codes we wrote.
        use crate::decode_layer2::{decode_layer2_audio_data, SYNTAX_GRANULES};
        use crate::tables_layer2::layer2_bit_allocation_table;

        // 192 kbit/s stereo at 48 kHz selects B.2a (sblimit=27). We use
        // a sparse allocation (sb=0 only) to keep the codeword stream
        // short and easy to verify against the decoder.
        let header = header_l2(48_000, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let sblimit = table.sblimit();
        let bound = layer2_stereo_bound(&header, sblimit);
        // sb=0 in B.2a — smallest legal non-zero alloc.
        let nbal0 = table.nbal(0);
        let mut chosen = 0u8;
        for a in 1u8..(1u8 << nbal0) {
            if table.quant_class(0, a).is_some() {
                chosen = a;
                break;
            }
        }
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        alloc[0][0] = chosen;
        alloc[1][0] = chosen;

        // SCFSI = 0b00, three scalefactor indices per (ch, sb).
        let mut scf_input = Layer2ScalefactorFieldInput::default();
        scf_input.scfsi[0][0] = 0b00;
        scf_input.scfsi[1][0] = 0b00;
        scf_input.scalefactor_indices[0][0] = [10, 20, 30];
        scf_input.scalefactor_indices[1][0] = [11, 21, 31];

        // Layer2 SAMPLES: a deterministic triplet pattern per granule.
        let cls = table.quant_class(0, chosen).expect("class");
        let n = cls.nlevels as u32;
        assert!(n >= 2, "need at least 2 levels for a non-trivial test");
        let mut samples_input = Box::new(Layer2SamplesFieldInput::default());
        for gr in 0..SYNTAX_GRANULES {
            for ch in 0..2 {
                let s0 = (gr as u32 + ch as u32) % n;
                let s1 = (gr as u32 + 1 + ch as u32) % n;
                let s2 = (gr as u32 + 2 + ch as u32) % n;
                samples_input.codes[ch][gr][0] = [s0, s1, s2];
            }
        }

        let mut bw = BitWriter::new();
        write_layer2_allocation_field(&mut bw, table, &alloc, 2, bound).expect("alloc");
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &scf_input, 2, bound).expect("scf");
        write_layer2_samples_field(&mut bw, table, &alloc, &samples_input, 2, bound)
            .expect("samples");
        let bytes = bw.finish();

        // Decode the bytes back.
        let decoded = decode_layer2_audio_data(&header, &bytes).expect("decode");
        assert_eq!(decoded.channels, 2);
        assert_eq!(decoded.sblimit, sblimit);

        // The decoder rescales by Table 3-B.1 then applies the §2.4.3.3.4
        // C * (s''' + D) requantization. To verify the round-trip, we
        // replicate the same math here from the codes we wrote and check
        // the decoded samples match.
        use crate::tables::SCALEFACTORS;
        let nb = cls.bits_per_sample();
        let msb = 1u32 << (nb - 1);
        for gr in 0..SYNTAX_GRANULES {
            let part = gr / 4; // 12 granules / 3 parts = 4 granules per part
            for ch in 0..2 {
                let codes = samples_input.codes[ch][gr][0];
                let scf_idx = scf_input.scalefactor_indices[ch][0][part] as usize & 0x3F;
                let factor = SCALEFACTORS[scf_idx];
                for s in 0..3 {
                    let c = codes[s];
                    // Re-derive the requantization the decoder applies.
                    let inverted = c ^ msb;
                    let signed = if inverted & msb != 0 {
                        i64::from(inverted) - (1i64 << nb)
                    } else {
                        i64::from(inverted)
                    };
                    let s_frac = signed as f64 / (1u64 << (nb - 1)) as f64;
                    let expected_dp = cls.c * (s_frac + cls.d);
                    let expected = factor * expected_dp;
                    let got = decoded.subbands[ch][0].samples[gr * 3 + s];
                    let diff = (got - expected).abs();
                    assert!(
                        diff < 1e-12,
                        "mismatch gr={gr} ch={ch} s={s}: expected {expected} got {got}"
                    );
                }
            }
        }
    }

    #[test]
    fn write_layer2_samples_field_shared_upper_band_mirrors_into_both_channels() {
        // Joint-stereo 64 kbit/s @ 44.1 kHz (B.2c, sblimit=8) with
        // mode_extension=0b01 → bound=8 clamped to sblimit=8. To exercise
        // the shared upper band, force bound below sblimit by raw header
        // value. Use mode_extension=0b00 → bound=4.
        use crate::decode_layer2::{decode_layer2_audio_data, SYNTAX_GRANULES};
        use crate::tables_layer2::layer2_bit_allocation_table;

        let header = header_l2_joint(44_100, 64, 0b00); // raw bound = 4
        let table = layer2_bit_allocation_table(&header);
        let sblimit = table.sblimit();
        let bound = layer2_stereo_bound(&header, sblimit);
        assert!(bound < sblimit, "test needs a real shared upper band");

        // sb >= bound carries a shared allocation. We allocate at the
        // smallest legal non-zero level at sb=bound only, for clarity.
        let nbal_b = table.nbal(bound);
        let mut chosen = 0u8;
        for a in 1u8..(1u8 << nbal_b) {
            if table.quant_class(bound, a).is_some() {
                chosen = a;
                break;
            }
        }
        let cls = table.quant_class(bound, chosen).expect("class");
        // Both channels must record the shared allocation — the
        // allocation-field writer requires it.
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        alloc[0][bound] = chosen;
        alloc[1][bound] = chosen;

        // SCFSI + scalefactor: distinct per channel even though the
        // *samples* are shared (per §2.4.1.6 / §2.4.3.3 / §2.4.2.6).
        let mut scf_input = Layer2ScalefactorFieldInput::default();
        scf_input.scfsi[0][bound] = 0b00;
        scf_input.scfsi[1][bound] = 0b00;
        scf_input.scalefactor_indices[0][bound] = [10, 20, 30];
        scf_input.scalefactor_indices[1][bound] = [11, 21, 31];

        // SAMPLES: the writer only reads channel 0 in the shared band.
        let n = cls.nlevels as u32;
        let mut samples_input = Box::new(Layer2SamplesFieldInput::default());
        for gr in 0..SYNTAX_GRANULES {
            let s0 = (gr as u32) % n;
            let s1 = (gr as u32 + 1) % n;
            let s2 = (gr as u32 + 2) % n;
            // Channel 0 carries the actual codes; channel 1 is ignored
            // by the writer but is left at zero to confirm that fact.
            samples_input.codes[0][gr][bound] = [s0, s1, s2];
            samples_input.codes[1][gr][bound] = [0, 0, 0];
        }

        let mut bw = BitWriter::new();
        write_layer2_allocation_field(&mut bw, table, &alloc, 2, bound).expect("alloc");
        write_layer2_scalefactor_field(&mut bw, table, &alloc, &scf_input, 2, bound).expect("scf");
        write_layer2_samples_field(&mut bw, table, &alloc, &samples_input, 2, bound)
            .expect("samples");
        let bytes = bw.finish();

        let decoded = decode_layer2_audio_data(&header, &bytes).expect("decode");
        assert_eq!(decoded.channels, 2);

        // Both decoded channels should carry the same s'' values
        // (re-rescaled by each channel's distinct scalefactor).
        use crate::tables::SCALEFACTORS;
        let nb = cls.bits_per_sample();
        let msb = 1u32 << (nb - 1);
        for gr in 0..SYNTAX_GRANULES {
            let part = gr / 4;
            let codes = samples_input.codes[0][gr][bound];
            for s in 0..3 {
                let c = codes[s];
                let inverted = c ^ msb;
                let signed = if inverted & msb != 0 {
                    i64::from(inverted) - (1i64 << nb)
                } else {
                    i64::from(inverted)
                };
                let s_frac = signed as f64 / (1u64 << (nb - 1)) as f64;
                let expected_dp = cls.c * (s_frac + cls.d);
                for ch in 0..2 {
                    let scf_idx = scf_input.scalefactor_indices[ch][bound][part] as usize & 0x3F;
                    let factor = SCALEFACTORS[scf_idx];
                    let expected = factor * expected_dp;
                    let got = decoded.subbands[ch][bound].samples[gr * 3 + s];
                    assert!(
                        (got - expected).abs() < 1e-12,
                        "ch={ch} gr={gr} s={s}: expected {expected} got {got}"
                    );
                }
            }
        }
    }

    #[test]
    fn write_layer2_samples_field_rejects_bad_channel_count() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Box::new(Layer2SamplesFieldInput::default());
        let mut bw = BitWriter::new();
        match write_layer2_samples_field(&mut bw, table, &alloc, &input, 0, table.sblimit()) {
            Err(Layer2SamplesFieldError::UnsupportedChannelCount(0)) => {}
            other => panic!("expected UnsupportedChannelCount(0), got {other:?}"),
        }
        match write_layer2_samples_field(&mut bw, table, &alloc, &input, 3, table.sblimit()) {
            Err(Layer2SamplesFieldError::UnsupportedChannelCount(3)) => {}
            other => panic!("expected UnsupportedChannelCount(3), got {other:?}"),
        }
    }

    #[test]
    fn write_layer2_samples_field_rejects_bound_above_sblimit() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Box::new(Layer2SamplesFieldInput::default());
        let mut bw = BitWriter::new();
        let bad_bound = table.sblimit() + 1;
        match write_layer2_samples_field(&mut bw, table, &alloc, &input, 2, bad_bound) {
            Err(Layer2SamplesFieldError::BoundExceedsSblimit { bound, sblimit }) => {
                assert_eq!(bound, bad_bound);
                assert_eq!(sblimit, table.sblimit());
            }
            other => panic!("expected BoundExceedsSblimit, got {other:?}"),
        }
    }

    #[test]
    fn write_layer2_samples_field_rejects_mono_with_shared_upper_band() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 64, Mode::SingleChannel);
        let table = layer2_bit_allocation_table(&header);
        let alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        let input = Box::new(Layer2SamplesFieldInput::default());
        let mut bw = BitWriter::new();
        // Mono with bound < sblimit is invalid (no shared upper band
        // exists for mono).
        let bad_bound = 4usize;
        match write_layer2_samples_field(&mut bw, table, &alloc, &input, 1, bad_bound) {
            Err(Layer2SamplesFieldError::MonoBoundBelowSblimit { bound, sblimit }) => {
                assert_eq!(bound, bad_bound);
                assert_eq!(sblimit, table.sblimit());
            }
            other => panic!("expected MonoBoundBelowSblimit, got {other:?}"),
        }
    }

    #[test]
    fn write_layer2_samples_field_rejects_invalid_allocation_code() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = table.sblimit();
        // Find an allocation index that is NOT a legal slot in sb=0's
        // row (a `-` cell). Walk down looking for the first invalid.
        let nbal0 = table.nbal(0);
        let mut bad = 0u8;
        for a in 0u8..(1u8 << nbal0) {
            if a != 0 && table.quant_class(0, a).is_none() {
                bad = a;
                break;
            }
        }
        if bad == 0 {
            // If sb=0 happens to have every slot defined, mark sb=0
            // with the max value of nbal+1 (off the field).
            bad = (1u8 << nbal0).wrapping_sub(1);
            if table.quant_class(0, bad).is_some() {
                // Skip the test if no invalid slot exists in this row.
                return;
            }
        }
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        alloc[0][0] = bad;
        let input = Box::new(Layer2SamplesFieldInput::default());
        let mut bw = BitWriter::new();
        match write_layer2_samples_field(&mut bw, table, &alloc, &input, 2, bound) {
            Err(Layer2SamplesFieldError::InvalidAllocationCode {
                channel,
                subband,
                allocation,
            }) => {
                assert_eq!(channel, 0);
                assert_eq!(subband, 0);
                assert_eq!(allocation, bad);
            }
            other => panic!("expected InvalidAllocationCode, got {other:?}"),
        }
    }

    #[test]
    fn write_layer2_samples_field_rejects_sample_code_out_of_range() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(44_100, 64, Mode::SingleChannel);
        let table = layer2_bit_allocation_table(&header);
        let bound = table.sblimit();
        let nbal0 = table.nbal(0);
        let mut chosen = 0u8;
        for a in 1u8..(1u8 << nbal0) {
            if table.quant_class(0, a).is_some() {
                chosen = a;
                break;
            }
        }
        let cls = table.quant_class(0, chosen).expect("class");
        let mut alloc: Layer2Allocation = [[0u8; SUBBANDS]; 2];
        alloc[0][0] = chosen;
        let mut input = Box::new(Layer2SamplesFieldInput::default());
        // Put a code at exactly nlevels (out of range).
        input.codes[0][0][0] = [cls.nlevels as u32, 0, 0];
        let mut bw = BitWriter::new();
        match write_layer2_samples_field(&mut bw, table, &alloc, &input, 1, bound) {
            Err(Layer2SamplesFieldError::SampleCodeOutOfRange {
                channel,
                subband,
                granule,
                sample,
                code,
                nlevels,
            }) => {
                assert_eq!(channel, 0);
                assert_eq!(subband, 0);
                assert_eq!(granule, 0);
                assert_eq!(sample, 0);
                assert_eq!(code, cls.nlevels as u32);
                assert_eq!(nlevels, cls.nlevels);
            }
            other => panic!("expected SampleCodeOutOfRange, got {other:?}"),
        }
        // And no bits written on error.
        let bytes = bw.finish();
        assert!(bytes.is_empty());
    }

    #[test]
    fn write_layer2_samples_field_error_paths_write_no_bytes() {
        // Pre-flight failure must not emit a single bit, even if it's
        // a per-(ch, sb, gr) sample-code rejection deep into the input.
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(48_000, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        let bound = table.sblimit();
        let alloc = dense_layer2_alloc(table, bound, 2);
        let mut input = Box::new(Layer2SamplesFieldInput::default());
        // Plant a bad code in the LAST granule of the LAST subband on
        // channel 1. Confirm the writer still rejects without emitting.
        let last_sb = table.sblimit() - 1;
        let a = alloc[1][last_sb];
        let cls = table.quant_class(last_sb, a).expect("class");
        input.codes[1][11][last_sb] = [cls.nlevels as u32 + 5, 0, 0];
        let mut bw = BitWriter::new();
        let err = write_layer2_samples_field(&mut bw, table, &alloc, &input, 2, bound);
        assert!(err.is_err());
        let bytes = bw.finish();
        assert!(bytes.is_empty(), "pre-flight rejection must not emit bytes");
    }

    // ---- §C.1.5.2 / §2.4.3.3.4 per-sample quantizer ----------------

    /// `quantize_layer2_sample` must be the exact inverse of the decoder
    /// `requantize_triplet` for every legal `(class, code)`. Feeding the
    /// per-class decoder reconstruction back through the encoder must
    /// recover the original code on every quantizer step.
    #[test]
    fn quantize_layer2_sample_inverts_requantize_triplet() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(48_000, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        // Walk a representative cross section of Tables 3-B.4 classes —
        // the small grouped ones (nlevels 3/5/9) plus a handful of the
        // non-grouped classes that drive most Layer II bitrates.
        let mut classes_seen = std::collections::HashSet::new();
        for sb in 0..table.sblimit() {
            let nbal = table.nbal(sb);
            for a in 1..(1u8 << nbal) {
                if let Some(class) = table.quant_class(sb, a) {
                    classes_seen.insert((class.nlevels, class.grouping));
                    // For every legal code [0, nlevels), reconstruct the
                    // decoder s'' value and round-trip through the
                    // encoder. The reconstructed value sits exactly on a
                    // quantizer grid point, so the encoder must recover
                    // the original code exactly (no half-step ambiguity).
                    for code in 0..(class.nlevels as u32) {
                        // Decoder reconstruction at unit scalefactor.
                        let s_dp = reconstruct_layer2_sample(class, code);
                        let got = quantize_layer2_sample(s_dp, 1.0, class);
                        assert_eq!(
                            got, code,
                            "class nlevels={} sb={sb} code={code} → {got}",
                            class.nlevels
                        );
                    }
                }
            }
        }
        // Sanity: we exercised more than a single class.
        assert!(
            classes_seen.len() >= 5,
            "expected the B.2a sweep to cover >= 5 quantization classes, saw {}",
            classes_seen.len()
        );
    }

    /// Helper: reproduce the decoder's §2.4.3.3.4 reconstruction for one
    /// raw code under a quantization class (no scalefactor rescale).
    fn reconstruct_layer2_sample(class: &QuantClass, code: u32) -> f64 {
        let nb = class.bits_per_sample();
        let msb = 1u32 << (nb - 1);
        let inverted = code ^ msb;
        let signed = if inverted & msb != 0 {
            i64::from(inverted) - (1i64 << nb)
        } else {
            i64::from(inverted)
        };
        let s_frac = signed as f64 / (1u64 << (nb - 1)) as f64;
        class.c * (s_frac + class.d)
    }

    /// Out-of-range PCM inputs must clamp into `[0, nlevels)` so the
    /// §2.4.1.6 writer's `SampleCodeOutOfRange` pre-flight never trips
    /// when the caller feeds the encoder its own analysed sub-bands —
    /// even on a scalefactor-saturating signal.
    #[test]
    fn quantize_layer2_sample_clamps_to_nlevels() {
        use crate::tables_layer2::layer2_bit_allocation_table;
        let header = header_l2(48_000, 192, Mode::Stereo);
        let table = layer2_bit_allocation_table(&header);
        for sb in 0..table.sblimit() {
            for a in 1..(1u8 << table.nbal(sb)) {
                if let Some(class) = table.quant_class(sb, a) {
                    // Push the encoder past the class's natural domain
                    // in both directions; clamp keeps the code on-grid.
                    for value in [-10.0f64, -2.0, 2.0, 10.0, 1e6] {
                        let code = quantize_layer2_sample(value, 1.0, class);
                        assert!(
                            code < class.nlevels as u32,
                            "class nlevels={} value={value} → code={code}",
                            class.nlevels
                        );
                    }
                }
            }
        }
    }

    // ---- top-level §C.1.5.2 frame encoder --------------------------

    /// A clean §2.4.1.6 round-trip: encode a known sub-band matrix,
    /// re-parse the header, walk the same `decode_layer2_audio_data`
    /// path a real decoder uses, and confirm the recovered sub-band
    /// samples sit on the chosen scalefactor grid (within one quantizer
    /// step of the analysed input).
    #[test]
    fn encode_layer2_frame_round_trip_mono() {
        use crate::decode_layer2::{decode_layer2_audio_data, LAYER2_SAMPLES_PER_SUBBAND};
        use crate::tables_layer2::layer2_bit_allocation_table;
        // 48 kHz / 128 kbit/s mono — selects Table 3-B.2c (sblimit = 8,
        // matching Annex B for that bitrate/Fs cell).
        let params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        let mut subbands = Box::new([[[0.0f64; 36]; SUBBANDS]; 2]);
        // Stamp a slowly-varying pattern into sb 0 and sb 4 (both inside
        // every Layer II `sblimit`) so the allocator must place at least
        // those two subbands.
        for slot in 0..LAYER2_SAMPLES_PER_SUBBAND {
            let t = slot as f64 / LAYER2_SAMPLES_PER_SUBBAND as f64;
            subbands[0][0][slot] = 0.5 * (2.0 * std::f64::consts::PI * t).sin();
            subbands[0][4][slot] = 0.3 * (2.0 * std::f64::consts::PI * 2.0 * t).cos();
        }

        let frame = encode_layer2_frame(&params, &subbands).expect("encode_layer2_frame");
        // §2.4.2.1 byte-count: floor(144·128·1000/48000) = 384 bytes.
        let expected_len = (144u32 * 128 * 1000) / 48_000;
        assert_eq!(frame.len(), expected_len as usize);
        // The header byte 0 must be 0xFF and byte 1's high nibble 0xF
        // (the §2.4.1.3 sync). The §2.4.1.3 layer field at bits 17:18
        // selects Layer II (`0b10` in the high nibble of byte 1) — for
        // mono / no-CRC the protection bit is `1`, so byte 1 should be
        // `0b1111_1101 = 0xFD`.
        assert_eq!(frame[0], 0xFF);
        assert_eq!(frame[1] & 0xF0, 0xF0);
        let header = FrameHeader::parse(&frame).expect("parse encoded header");
        assert_eq!(header.layer, crate::header::Layer::II);
        assert_eq!(header.channels(), 1);

        // Decode through the production decoder path.
        let recovered = decode_layer2_audio_data(&header, &frame[4..]).expect("layer II decode");
        let table = layer2_bit_allocation_table(&header);
        // Per-slot tolerance per the chosen scalefactor's grid. Pick the
        // coarsest scalefactor / class active in the frame so any
        // subband's reconstruction sits within one quantizer step of
        // that grid.
        let mut peak_err_within_grid = true;
        let mut any_alloc = false;
        for sb in 0..table.sblimit() {
            let alloc = recovered.subbands[0][sb].allocation;
            if alloc == 0 {
                continue;
            }
            any_alloc = true;
            let class = table.quant_class(sb, alloc).expect("class");
            // Reconstruction step at unit scalefactor is at most
            // 2·class.c / nlevels; the encoder picked a scalefactor
            // strictly larger than the per-part peak so the grid step
            // for that part is at most `2·class.c·scf / nlevels`. Take
            // the worst-case scalefactor (== 2.0 = SCALEFACTORS[0])
            // as a safe upper bound.
            let step = 2.0 * class.c * 2.0 / class.nlevels as f64;
            for slot in 0..LAYER2_SAMPLES_PER_SUBBAND {
                let want = subbands[0][sb][slot];
                let got = recovered.subbands[0][sb].samples[slot];
                if (got - want).abs() > step {
                    peak_err_within_grid = false;
                }
            }
        }
        assert!(
            any_alloc,
            "encoder must place at least one Layer II allocation"
        );
        assert!(
            peak_err_within_grid,
            "Layer II round-trip exceeded one-quantizer-step error per allocated subband"
        );
    }

    /// The §2.4.1.4 CRC opt-in on the top-level Layer II encoder writes
    /// a CRC the decoder's `verify_layer2_crc` accepts as `Ok`, and the
    /// frame byte length is unchanged from the no-CRC variant (the CRC
    /// occupies bytes the allocator would otherwise have spent on audio
    /// data, but the §2.4.2.1 frame envelope stays the same).
    #[test]
    fn encode_layer2_frame_emits_verifying_crc() {
        use crate::decode_layer2::verify_layer2_crc;
        let mut params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        params.has_crc = true;
        let subbands = Box::new([[[0.05f64; 36]; SUBBANDS]; 2]);
        let frame = encode_layer2_frame(&params, &subbands).expect("encode_layer2_frame");
        // §2.4.2.1 byte count is the same as the no-CRC frame at the
        // same bitrate.
        let expected_len = (144u32 * 128 * 1000) / 48_000;
        assert_eq!(frame.len(), expected_len as usize);
        // Re-parse and verify the CRC.
        let header = FrameHeader::parse(&frame[..4]).expect("parse header");
        assert!(
            header.has_crc(),
            "has_crc must be true on the encoded frame"
        );
        let status =
            verify_layer2_crc(&header, &frame[..4], &frame[4..]).expect("verify_layer2_crc");
        assert!(
            status.is_good(),
            "verify_layer2_crc must accept the encoder's CRC, got {:?}",
            status
        );
    }

    /// Joint-stereo Layer II frames encode + decode through the
    /// intensity-stereo upper band: the bitstream's shared cells store
    /// a single sample triplet but the §2.4.1.6 syntax still reads one
    /// scalefactor per channel, so channel 0 / channel 1 recovered
    /// samples are the same `s_dp` triplet rescaled by each channel's
    /// own scalefactor. The ratio between the two per-slot
    /// reconstructions is therefore the ratio of their scalefactors,
    /// independent of the chosen quantizer.
    #[test]
    fn encode_layer2_frame_round_trip_joint_stereo() {
        use crate::decode_layer2::{decode_layer2_audio_data, LAYER2_SAMPLES_PER_SUBBAND};
        use crate::tables_layer2::layer2_bit_allocation_table;
        let mut params = Layer2HeaderParams::new(44_100, 192, Mode::JointStereo);
        // mode_extension == 0b01 → bound subband index = 8 (per
        // §2.4.2.3). With Fs=44.1kHz/192kbit/s Table 3-B.2a (sblimit=27),
        // the upper band [8, 27) is the shared region.
        params.mode_extension = crate::header::ModeExtension(0b01);
        let mut subbands = Box::new([[[0.0f64; 36]; SUBBANDS]; 2]);
        for slot in 0..LAYER2_SAMPLES_PER_SUBBAND {
            let t = slot as f64 / LAYER2_SAMPLES_PER_SUBBAND as f64;
            // Channel 0 carries content in both the low band (sb 2) and
            // the shared upper band (sb 10). Channel 1 also has content
            // in the shared upper band so its per-part scalefactor is
            // not the silent-fallback `62`; the encoder pre-mirrors the
            // peaks anyway (shared cells take the per-channel max so the
            // allocator's quantizer-grid step covers both).
            subbands[0][2][slot] = 0.4 * (2.0 * std::f64::consts::PI * t).sin();
            subbands[0][10][slot] = 0.3 * (4.0 * std::f64::consts::PI * t).cos();
            subbands[1][2][slot] = 0.2 * (2.0 * std::f64::consts::PI * t).cos();
            subbands[1][10][slot] = 0.25 * (4.0 * std::f64::consts::PI * t).sin();
        }
        let frame = encode_layer2_frame(&params, &subbands).expect("encode_layer2_frame");
        let header = FrameHeader::parse(&frame).expect("parse header");
        assert_eq!(header.mode, Mode::JointStereo);
        let recovered = decode_layer2_audio_data(&header, &frame[4..]).expect("decode");
        let table = layer2_bit_allocation_table(&header);
        let bound = layer2_stereo_bound(&header, table.sblimit());
        // §2.4.1.6 shared upper band: both channels see the same
        // allocation (writer pre-flight enforces this), and the decoder
        // applies the same `s_dp` triplet to each per-channel scalefactor.
        // Confirm: (a) allocations match in the shared band, and (b) the
        // ch-0/ch-1 sample ratio matches the per-part scalefactor ratio
        // within a single quantizer step.
        let mut any_shared = false;
        for sb in bound..table.sblimit() {
            let a0 = recovered.subbands[0][sb].allocation;
            let a1 = recovered.subbands[1][sb].allocation;
            assert_eq!(
                a0, a1,
                "joint_stereo shared sb={sb} allocation must match: ch0={a0} ch1={a1}"
            );
            if a0 == 0 {
                continue;
            }
            any_shared = true;
            for slot in 0..LAYER2_SAMPLES_PER_SUBBAND {
                let s0 = recovered.subbands[0][sb].samples[slot];
                let s1 = recovered.subbands[1][sb].samples[slot];
                let part = slot / 12;
                let scf0 = SCALEFACTORS
                    [recovered.subbands[0][sb].scalefactor_indices[part] as usize & 0x3F];
                let scf1 = SCALEFACTORS
                    [recovered.subbands[1][sb].scalefactor_indices[part] as usize & 0x3F];
                // s0/scf0 must equal s1/scf1 (the shared §2.4.3.3.4
                // `s_dp` triplet), within a one-ULP floating-point
                // tolerance: the rescale is the only step that differs
                // between the two channels.
                let r0 = s0 / scf0;
                let r1 = s1 / scf1;
                assert!(
                    (r0 - r1).abs() < 1e-9,
                    "joint_stereo shared sb={sb} slot={slot}: \
                     ch0/scf0={r0} vs ch1/scf1={r1}"
                );
            }
        }
        assert!(
            any_shared,
            "joint_stereo expected at least one allocated shared upper-band subband"
        );
    }

    /// The top-level encoder rejects header parameters whose bitrate is
    /// not on the §2.4.2.3 Layer II ladder, mirroring `pack_layer2_header`.
    #[test]
    fn encode_layer2_frame_rejects_off_ladder_bitrate() {
        // 100 kbit/s is not on the MPEG-1 Layer II ladder (32 / 48 / 56
        // / 64 / 80 / 96 / 112 / 128 / 160 / 192 / 224 / 256 / 320 / 384).
        let params = Layer2HeaderParams::new(48_000, 100, Mode::SingleChannel);
        let subbands = Box::new([[[0.0f64; 36]; SUBBANDS]; 2]);
        let err = encode_layer2_frame(&params, &subbands)
            .expect_err("100 kbit/s must be rejected on the MPEG-1 L2 ladder");
        match err {
            Layer2EncodeError::Header(Layer2HeaderError::UnsupportedBitrate(100)) => {}
            other => panic!("expected Header(UnsupportedBitrate(100)), got {other:?}"),
        }
    }

    /// LSF Layer II (16 / 22.05 / 24 kHz, `ID == 0`) frames encode via
    /// the §2.4.3.1 Table B.1 LSF allocation table and round-trip
    /// through the decoder's matching LSF Layer II path.
    #[test]
    fn encode_layer2_frame_round_trip_lsf_mono() {
        use crate::decode_layer2::decode_layer2_audio_data;
        let params = Layer2HeaderParams::new(24_000, 64, Mode::SingleChannel);
        let mut subbands = Box::new([[[0.0f64; 36]; SUBBANDS]; 2]);
        for slot in 0..36 {
            let t = slot as f64 / 36.0;
            subbands[0][1][slot] = 0.3 * (2.0 * std::f64::consts::PI * t).sin();
            subbands[0][5][slot] = 0.2 * (2.0 * std::f64::consts::PI * 2.0 * t).cos();
        }
        let frame = encode_layer2_frame(&params, &subbands).expect("encode_layer2_frame LSF");
        let header = FrameHeader::parse(&frame).expect("parse LSF header");
        assert!(header.is_lsf(), "LSF Fs must produce an LSF (ID==0) header");
        let _recovered =
            decode_layer2_audio_data(&header, &frame[4..]).expect("LSF Layer II decode");
        // Byte count: floor(144·64·1000/24000) = 384.
        let expected = (144u32 * 64 * 1000) / 24_000;
        assert_eq!(frame.len(), expected as usize);
    }

    // ---- Mp1Layer2FrameEncoder (top-level §C.1.3 + §C.1.5.2) ---

    /// Build an interleaved `f64` PCM block carrying `nch` channels of
    /// a sum of two sinusoids per channel at `fs`, with
    /// `LAYER2_SAMPLES_PER_FRAME` (1152) samples per channel — the
    /// Layer II frame granularity. Channel 1 (when present) is
    /// π/4-phase-shifted relative to channel 0 so the two channels
    /// carry distinct sub-band content.
    fn layer2_test_pcm(fs: u32, nch: usize) -> Vec<f64> {
        use crate::decode_layer2::LAYER2_SAMPLES_PER_FRAME;
        let n = LAYER2_SAMPLES_PER_FRAME;
        let mut pcm = vec![0.0f64; n * nch];
        for s in 0..n {
            let t = s as f64 / fs as f64;
            for ch in 0..nch {
                let phase = if ch == 0 {
                    0.0
                } else {
                    std::f64::consts::FRAC_PI_4
                };
                pcm[s * nch + ch] = 0.4 * (2.0 * std::f64::consts::PI * 440.0 * t + phase).sin()
                    + 0.2 * (2.0 * std::f64::consts::PI * 1_320.0 * t + phase).cos();
            }
        }
        pcm
    }

    /// Top-level Layer II encoder: a mono 48 kHz / 128 kbit/s PCM
    /// frame round-trips through the decoder into a non-silent PCM
    /// frame, the produced bytes match the §2.4.2.1 frame length, and
    /// the §2.4.1.3 header re-parses to Layer II / `ID == 1`.
    #[test]
    fn mp1_layer2_frame_encoder_round_trip_mono() {
        use crate::decode_layer2::{decode_layer2_audio_data, LAYER2_SAMPLES_PER_FRAME};

        let params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        assert_eq!(enc.channels(), 1);
        let pcm = layer2_test_pcm(48_000, 1);
        assert_eq!(pcm.len(), LAYER2_SAMPLES_PER_FRAME);
        let frame = enc.encode_frame(&pcm).expect("encode_frame mono");

        // §2.4.2.1: floor(144·128·1000/48000) = 384 bytes.
        let expected_len = (144u32 * 128 * 1000) / 48_000;
        assert_eq!(frame.len(), expected_len as usize);
        let header = FrameHeader::parse(&frame).expect("parse header");
        assert_eq!(header.layer, crate::header::Layer::II);
        assert_eq!(header.channels(), 1);
        assert!(!header.is_lsf());

        // The decoder reconstructs the sub-band matrix; the encoder
        // must have placed at least one non-zero allocation (silence
        // would be a degenerate sanity-fail).
        let recovered =
            decode_layer2_audio_data(&header, &frame[4..]).expect("decode_layer2_audio_data");
        let any_alloc = (0..recovered.sblimit).any(|sb| recovered.subbands[0][sb].allocation != 0);
        assert!(any_alloc, "encoder must place at least one allocation");
    }

    /// Top-level Layer II encoder, joint-stereo 44.1 kHz / 192 kbit/s:
    /// the §2.4.1.6 shared upper band's per-channel allocations agree
    /// in the bound..sblimit range, and the decoder reconstructs both
    /// channels.
    #[test]
    fn mp1_layer2_frame_encoder_round_trip_joint_stereo() {
        use crate::decode_layer2::{decode_layer2_audio_data, LAYER2_SAMPLES_PER_FRAME};

        let params = Layer2HeaderParams::new(44_100, 192, Mode::JointStereo);
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        assert_eq!(enc.channels(), 2);
        let pcm = layer2_test_pcm(44_100, 2);
        assert_eq!(pcm.len(), LAYER2_SAMPLES_PER_FRAME * 2);
        let frame = enc.encode_frame(&pcm).expect("encode_frame stereo");

        // §2.4.2.1: floor(144·192·1000/44100) = 626 bytes.
        let expected_len = (144u32 * 192 * 1000) / 44_100;
        assert_eq!(frame.len(), expected_len as usize);
        let header = FrameHeader::parse(&frame).expect("parse header");
        assert_eq!(header.layer, crate::header::Layer::II);
        assert_eq!(header.channels(), 2);

        let recovered =
            decode_layer2_audio_data(&header, &frame[4..]).expect("decode_layer2_audio_data");
        // Joint-stereo upper-band invariant: ch0 and ch1 allocations
        // identical for sb ∈ [bound, sblimit).
        let table = crate::tables_layer2::layer2_bit_allocation_table(&header);
        let bound = layer2_stereo_bound(&header, table.sblimit());
        for sb in bound..table.sblimit() {
            assert_eq!(
                recovered.subbands[0][sb].allocation, recovered.subbands[1][sb].allocation,
                "shared upper band must have identical allocation per channel \
                 (sb={sb}, bound={bound})"
            );
        }
        // Each channel placed at least one allocation overall.
        for ch in 0..2 {
            let any = (0..recovered.sblimit).any(|sb| recovered.subbands[ch][sb].allocation != 0);
            assert!(any, "channel {ch} must place at least one allocation");
        }
    }

    /// `encode_frame` rejects the wrong PCM length with the new
    /// [`Layer2EncodeError::WrongSampleCount`] variant.
    #[test]
    fn mp1_layer2_frame_encoder_rejects_wrong_sample_count() {
        let params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        // 1024 samples is not 1152 — short frame.
        let pcm = vec![0.0f64; 1024];
        let err = enc.encode_frame(&pcm).unwrap_err();
        assert_eq!(err, Layer2EncodeError::WrongSampleCount { got: 1024 });
    }

    /// `encode_frame` surfaces the off-ladder bitrate rejection from
    /// the underlying [`encode_layer2_frame`] (lazy validation in the
    /// header packer fires on first use, matching `Mp1FrameEncoder`).
    #[test]
    fn mp1_layer2_frame_encoder_rejects_off_ladder_bitrate() {
        // 100 kbit/s is not on either Layer II ladder.
        let params = Layer2HeaderParams::new(48_000, 100, Mode::SingleChannel);
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        let pcm = layer2_test_pcm(48_000, 1);
        let err = enc.encode_frame(&pcm).unwrap_err();
        match err {
            Layer2EncodeError::Header(Layer2HeaderError::UnsupportedBitrate(100)) => {}
            other => panic!("expected UnsupportedBitrate(100), got {other:?}"),
        }
    }

    /// LSF round-trip through the top-level encoder: a 24 kHz / 64
    /// kbit/s mono PCM frame encodes via Table B.1 and decodes back
    /// through the matching LSF Layer II path.
    #[test]
    fn mp1_layer2_frame_encoder_round_trip_lsf_mono() {
        use crate::decode_layer2::decode_layer2_audio_data;

        let params = Layer2HeaderParams::new(24_000, 64, Mode::SingleChannel);
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        let pcm = layer2_test_pcm(24_000, 1);
        let frame = enc.encode_frame(&pcm).expect("encode_frame LSF");
        // §2.4.2.1: floor(144·64·1000/24000) = 384 bytes.
        assert_eq!(frame.len(), ((144u32 * 64 * 1000) / 24_000) as usize);
        let header = FrameHeader::parse(&frame).expect("parse LSF header");
        assert!(header.is_lsf(), "LSF Fs must produce an LSF (ID==0) header");
        let _ = decode_layer2_audio_data(&header, &frame[4..]).expect("LSF Layer II decode");
    }

    /// `Mp1Layer2FrameEncoder::reset` zeroes the per-channel
    /// AnalysisFilter input FIFO — the same condition as a freshly
    /// constructed encoder. So encoding a Layer II frame on a fresh
    /// encoder produces byte-identical output to encoding it on a
    /// previously-used-then-reset encoder, given identical input.
    #[test]
    fn mp1_layer2_frame_encoder_reset_zeroes_history() {
        let params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        let mut a = Mp1Layer2FrameEncoder::new(params);
        let mut b = Mp1Layer2FrameEncoder::new(params);
        let pcm = layer2_test_pcm(48_000, 1);
        // Prime `a` with a different signal so its AnalysisFilter
        // history diverges from `b`.
        let prime: Vec<f64> = (0..pcm.len())
            .map(|i| ((i % 64) as f64) / 64.0 - 0.5)
            .collect();
        let _ = a.encode_frame(&prime).expect("prime");
        // Now reset `a` and encode the test signal on both.
        a.reset();
        let fa = a.encode_frame(&pcm).expect("encode after reset");
        let fb = b.encode_frame(&pcm).expect("encode fresh");
        assert_eq!(fa, fb, "reset must restore fresh-encoder history");
    }

    /// The §2.4.1.4 CRC opt-in on the top-level Layer II frame
    /// encoder writes a §2.4.3.1 CRC the decoder's `verify_layer2_crc`
    /// accepts as `Ok`, and the frame byte length is unchanged from
    /// the no-CRC variant (§2.4.2.1 slot count is bitrate-derived).
    #[test]
    fn mp1_layer2_frame_encoder_emits_verifying_crc() {
        use crate::decode_layer2::verify_layer2_crc;

        let mut params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        params.has_crc = true;
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        let pcm = layer2_test_pcm(48_000, 1);
        let frame = enc.encode_frame(&pcm).expect("encode_frame with CRC");
        // §2.4.2.1: floor(144·128·1000/48000) = 384 bytes regardless of
        // CRC (CRC bits come out of the audio-data budget).
        let expected_len = (144u32 * 128 * 1000) / 48_000;
        assert_eq!(frame.len(), expected_len as usize);

        let header = FrameHeader::parse(&frame).expect("parse header");
        assert!(header.has_crc(), "has_crc must clear protection_bit");
        // `verify_layer2_crc` takes (header_bytes, after_header) where
        // after_header carries the CRC word at the start followed by
        // the §2.4.1.6 allocation + scfsi region.
        let status = verify_layer2_crc(&header, &frame[..4], &frame[4..])
            .expect("verify_layer2_crc: CRC region present");
        assert!(
            status.is_good(),
            "encoded CRC must verify clean: {status:?}"
        );
    }

    // ---- §2.4.1.8 ancillary_data() tail (Layer II encoder) ---------

    /// Top-level `encode_layer2_frame_with_ancillary` writes the
    /// supplied bytes into the §2.4.1.8 tail. The frame still has the
    /// §2.4.2.1 byte count and the decoder still recovers the
    /// audio-data region; the ancillary payload appears at the end of
    /// the frame just before any zero-padded slack.
    #[test]
    fn encode_layer2_frame_with_ancillary_appends_payload() {
        use crate::decode_layer2::decode_layer2_audio_data;

        // Mono 48 kHz / 192 kbps → 576-byte frames; a silence input
        // yields silent sub-bands and the §C.1.5.2.7 allocator emits
        // the empty allocation, leaving the §2.4.1.8 tail at its
        // maximum size for the configuration.
        let params = Layer2HeaderParams::new(48_000, 192, Mode::SingleChannel);
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        let pcm = vec![0.0f64; crate::decode_layer2::LAYER2_SAMPLES_PER_FRAME];
        // The reference frame: no ancillary, encoder runs to completion
        // on the same PCM with a fresh history.
        let ref_frame = enc.encode_frame(&pcm).expect("encode_frame reference");

        // Same PCM through a second encoder with the same fresh
        // history, plus a §2.4.1.8 ancillary payload.
        let mut enc2 = Mp1Layer2FrameEncoder::new(params);
        let payload: [u8; 5] = [0xDE, 0xAD, 0xBE, 0xEF, 0x42];
        enc2.set_pending_ancillary(&payload);
        assert_eq!(enc2.pending_ancillary(), &payload);
        let frame = enc2.encode_frame(&pcm).expect("encode_frame ancillary");
        assert_eq!(
            frame.len(),
            ref_frame.len(),
            "§2.4.2.1 frame length unchanged by ancillary"
        );

        // The §2.4.1.6 audio-data prefix is identical in both frames;
        // they differ only in the §2.4.1.8 tail. The first byte where
        // the two frames differ marks the §2.4.1.8 tail boundary, and
        // the ancillary frame's bytes from that offset onward must
        // start with the caller's payload.
        let mut tail_start = ref_frame.len();
        for (i, (r, f)) in ref_frame.iter().zip(frame.iter()).enumerate() {
            if r != f {
                tail_start = i;
                break;
            }
        }
        assert!(
            tail_start < ref_frame.len(),
            "ancillary must produce at least one differing byte"
        );
        assert!(
            tail_start + payload.len() <= frame.len(),
            "tail must hold the §2.4.1.8 payload"
        );
        assert_eq!(
            &frame[tail_start..tail_start + payload.len()],
            &payload,
            "ancillary payload must appear at the §2.4.1.8 tail"
        );
        // Anything after the payload is zero-padded, and the
        // corresponding region of the reference is also zero.
        assert!(
            frame[tail_start + payload.len()..].iter().all(|b| *b == 0),
            "post-payload §2.4.2.1 padding must be zero"
        );
        assert!(
            ref_frame[tail_start..].iter().all(|b| *b == 0),
            "reference frame's §2.4.1.8 tail must be zero",
        );

        // Audio-data region still decodes identically.
        let header = FrameHeader::parse(&frame).expect("parse header");
        let recovered =
            decode_layer2_audio_data(&header, &frame[4..]).expect("decode with ancillary");
        let ref_header = FrameHeader::parse(&ref_frame).expect("parse ref header");
        let ref_recovered =
            decode_layer2_audio_data(&ref_header, &ref_frame[4..]).expect("decode ref");
        assert_eq!(recovered.sblimit, ref_recovered.sblimit);
        for sb in 0..recovered.sblimit {
            assert_eq!(
                recovered.subbands[0][sb].allocation, ref_recovered.subbands[0][sb].allocation,
                "§2.4.1.6 allocation must be unchanged by §2.4.1.8 ancillary",
            );
        }
    }

    /// An ancillary payload larger than the §2.4.1.8 tail capacity
    /// surfaces a typed [`Layer2EncodeError::AncillaryTooLarge`] error,
    /// and the stateful encoder clears the staged payload after the
    /// failed call (next encode reverts to zero-padded tail).
    #[test]
    fn encode_layer2_frame_with_oversized_ancillary_errors() {
        let params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        // The §2.4.2.1 frame is 384 bytes; padding a payload bigger
        // than the whole frame guarantees overflow.
        let oversized = vec![0u8; 4096];
        let mut subbands = [[[0.0f64; 36]; SUBBANDS]; 2];
        // Populate trivial sub-bands so the encode pipeline runs.
        for sb in 0..2 {
            for slot in 0..36 {
                subbands[0][sb][slot] = 0.1;
            }
        }
        let err = encode_layer2_frame_with_ancillary(&params, &subbands, &oversized)
            .expect_err("oversized ancillary must error");
        match err {
            Layer2EncodeError::AncillaryTooLarge { space, got } => {
                assert!(space < got);
                assert_eq!(got, oversized.len());
            }
            other => panic!("expected AncillaryTooLarge, got {other:?}"),
        }
    }

    /// The stateful encoder consumes the staged ancillary once: a
    /// second `encode_frame` call after the first one with ancillary
    /// returns to the no-ancillary frame layout. We compare against
    /// pairs of fresh encoders (same params, same PCM, same history)
    /// to confirm the two-frame sequence "ancillary then nothing" on
    /// one encoder matches "ancillary then nothing" on two encoders
    /// where the second has no staged payload.
    #[test]
    fn mp1_layer2_frame_encoder_pending_ancillary_consumed_once() {
        // Mono 48 kHz / 192 kbps + silence input: the §C.1.5.2.7
        // allocator emits the empty allocation, leaving the §2.4.1.8
        // tail at its maximum size.
        let params = Layer2HeaderParams::new(48_000, 192, Mode::SingleChannel);
        let pcm = vec![0.0f64; crate::decode_layer2::LAYER2_SAMPLES_PER_FRAME];
        let payload: [u8; 4] = [0xA5, 0x5A, 0x12, 0x34];

        // One stateful encoder: stage payload, encode, then encode
        // again without staging anything.
        let mut enc = Mp1Layer2FrameEncoder::new(params);
        enc.set_pending_ancillary(&payload);
        let frame_with = enc.encode_frame(&pcm).expect("with ancillary");
        // Buffer cleared after consumption.
        assert_eq!(enc.pending_ancillary(), &[] as &[u8]);
        let frame_after = enc.encode_frame(&pcm).expect("after ancillary");

        // Reference encoders that mirror the same analysis-history
        // trajectory: encoder A stages the payload for frame 0;
        // encoder B encodes two plain frames. Frame 1 of A must equal
        // frame 1 of B (both ancillary-free after a frame of history).
        let mut a = Mp1Layer2FrameEncoder::new(params);
        a.set_pending_ancillary(&payload);
        let a0 = a.encode_frame(&pcm).expect("a0");
        assert_eq!(a0, frame_with);
        let a1 = a.encode_frame(&pcm).expect("a1");
        assert_eq!(a1, frame_after);

        let mut b = Mp1Layer2FrameEncoder::new(params);
        // b also runs ancillary on frame 0 so its analysis history
        // tracks a's; the test target is "frame 1 ancillary-free".
        b.set_pending_ancillary(&payload);
        let _ = b.encode_frame(&pcm).expect("b0");
        let b1 = b.encode_frame(&pcm).expect("b1");
        // Sanity: b1 is exactly a1 (deterministic).
        assert_eq!(a1, b1);

        // And the two frames `frame_with` and `frame_after` differ:
        // the ancillary tail is gone.
        assert_ne!(frame_with, frame_after);
    }

    /// [`Mp1Layer2FrameEncoder::clear_pending_ancillary`] drops a
    /// staged payload without consuming a frame; the next encode is
    /// indistinguishable from one made on a fresh encoder.
    #[test]
    fn mp1_layer2_frame_encoder_clear_pending_ancillary() {
        let params = Layer2HeaderParams::new(48_000, 128, Mode::SingleChannel);
        let mut enc_a = Mp1Layer2FrameEncoder::new(params);
        let mut enc_b = Mp1Layer2FrameEncoder::new(params);

        enc_a.set_pending_ancillary(&[1, 2, 3]);
        assert_eq!(enc_a.pending_ancillary(), &[1, 2, 3]);
        enc_a.clear_pending_ancillary();
        assert_eq!(enc_a.pending_ancillary(), &[] as &[u8]);

        let pcm = layer2_test_pcm(48_000, 1);
        let frame_a = enc_a.encode_frame(&pcm).expect("encode a");
        let frame_b = enc_b.encode_frame(&pcm).expect("encode b");
        assert_eq!(frame_a, frame_b, "cleared ancillary == no ancillary");
    }

    /// §2.4.1.8 ancillary on a §2.4.1.4-CRC frame: the CRC region is
    /// header bits 16…31 + allocation + scfsi (per Annex B Table 3-B.5)
    /// — it does NOT cover the §2.4.1.8 tail — so the stored CRC
    /// continues to verify with `verify_layer2_crc` and the tail still
    /// carries the caller's payload.
    #[test]
    fn encode_layer2_frame_with_ancillary_and_crc_verifies() {
        use crate::decode_layer2::verify_layer2_crc;

        // Mono 48 kHz / 192 kbps + silence: §2.4.1.8 tail is maximal
        // even after the 16-bit §2.4.1.4 CRC eats two bytes.
        let mut params = Layer2HeaderParams::new(48_000, 192, Mode::SingleChannel);
        params.has_crc = true;
        let pcm = vec![0.0f64; crate::decode_layer2::LAYER2_SAMPLES_PER_FRAME];
        let payload: [u8; 6] = [0xCA, 0xFE, 0xBA, 0xBE, 0x55, 0xAA];

        let mut enc_ref = Mp1Layer2FrameEncoder::new(params);
        let ref_frame = enc_ref.encode_frame(&pcm).expect("ref crc frame");

        let mut enc = Mp1Layer2FrameEncoder::new(params);
        enc.set_pending_ancillary(&payload);
        let frame = enc.encode_frame(&pcm).expect("encode with crc + ancillary");

        let header = FrameHeader::parse(&frame).expect("parse header");
        assert!(header.has_crc(), "protection_bit must indicate CRC present");
        let status = verify_layer2_crc(&header, &frame[..4], &frame[4..])
            .expect("verify_layer2_crc: CRC region present");
        assert!(
            status.is_good(),
            "§2.4.3.1 CRC must verify clean even with §2.4.1.8 ancillary tail: {status:?}"
        );
        // Both frames carry the same CRC word (bytes 4..6) since the
        // §2.4.3.1 protected region excludes the §2.4.1.8 tail.
        assert_eq!(&frame[4..6], &ref_frame[4..6]);

        // Locate the §2.4.1.8 tail by diffing against the no-ancillary
        // reference; the payload must appear at the first differing
        // byte.
        let tail_start = frame
            .iter()
            .zip(ref_frame.iter())
            .position(|(a, b)| a != b)
            .expect("ancillary frame differs from reference somewhere");
        assert_eq!(
            &frame[tail_start..tail_start + payload.len()],
            &payload,
            "§2.4.1.8 payload must appear at the differing-byte boundary"
        );
    }
}
