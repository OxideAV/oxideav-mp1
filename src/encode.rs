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

/// Parameters for a single Layer I encode: the target bitrate and the
/// stream's sampling frequency and channel mode.
#[derive(Debug, Clone, Copy)]
pub struct EncodeParams {
    /// Target bitrate. Must be a fixed Layer I ladder value
    /// ([`Bitrate::Fixed`]); free format is not produced by this
    /// encoder.
    pub bitrate: Bitrate,
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
}

impl EncodeParams {
    /// A new [`EncodeParams`] with `emit_crc = false`, the encoder's
    /// historical default (the optional §2.4.1.4 CRC `error_check()` is
    /// **not** written; `protection_bit == 1`).
    ///
    /// `bitrate` is a fixed Layer I ladder value (free format is not
    /// produced); `sampling_frequency` must be one of the six Layer I
    /// sampling rates (MPEG-1 32 / 44.1 / 48 kHz from 11172-3 §2.4.2.3,
    /// or MPEG-2 LSF 16 / 22.05 / 24 kHz from 13818-3 §2.4.2.3); `mode`
    /// is the per-channel mode written into the header `mode` field.
    pub fn new(bitrate: Bitrate, sampling_frequency: u32, mode: Mode) -> EncodeParams {
        EncodeParams {
            bitrate,
            sampling_frequency,
            mode,
            emit_crc: false,
        }
    }

    /// Builder: select whether the encoder emits the optional §2.4.1.4
    /// `error_check()` CRC field (see [`EncodeParams::emit_crc`]).
    pub fn with_emit_crc(mut self, emit: bool) -> EncodeParams {
        self.emit_crc = emit;
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
        let bitrate_kbps = match self.params.bitrate {
            Bitrate::Fixed(k) => k,
            _ => return Err(EncodeError::UnsupportedBitrate),
        };
        let (id_bit, samp_code) = sampling_code(self.params.sampling_frequency).ok_or(
            EncodeError::UnsupportedSamplingFrequency(self.params.sampling_frequency),
        )?;
        let brate_idx =
            bitrate_index(bitrate_kbps, id_bit).ok_or(EncodeError::UnsupportedBitrate)?;

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
}
