//! MPEG-1 Audio **Layer I** encode (analysis filterbank, scalefactor
//! selection, bit allocation, uniform quantization).
//!
//! Everything in this module is derived solely from ISO/IEC 11172-3
//! (1993), the informative Annex C "The encoding process" plus the
//! normative Layer I clauses:
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
use crate::header::{Bitrate, Mode};
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

/// The §2.4.2.3 `sampling_frequency` field code for a frequency, or
/// `None` if the frequency is not a Layer I value.
fn sampling_code(hz: u32) -> Option<u8> {
    match hz {
        44_100 => Some(0b00),
        48_000 => Some(0b01),
        32_000 => Some(0b10),
        _ => None,
    }
}

/// The §2.4.2.3 `bitrate_index` for a fixed Layer I ladder value in
/// kbit/s, or `None` if the value is not on the ladder.
fn bitrate_index(kbps: u16) -> Option<u8> {
    const LADDER: [u16; 14] = [
        32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448,
    ];
    LADDER
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
    /// samples per channel (`pcm.len() == 384 * nch`). No CRC is written
    /// (`protection_bit == 1`).
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
        let brate_idx = bitrate_index(bitrate_kbps).ok_or(EncodeError::UnsupportedBitrate)?;
        let samp_code = sampling_code(self.params.sampling_frequency).ok_or(
            EncodeError::UnsupportedSamplingFrequency(self.params.sampling_frequency),
        )?;

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
        let has_crc = false;
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
        bw.put(1, 1); // ID = MPEG-1
        bw.put(0b11, 2); // layer I
        bw.put(1, 1); // protection_bit = 1 (no CRC)
        bw.put(brate_idx as u32, 4); // bitrate_index
        bw.put(samp_code as u32, 2); // sampling_frequency
        bw.put(0, 1); // padding_bit = 0
        bw.put(0, 1); // private_bit
        bw.put(self.mode_bits(), 2); // mode
        bw.put(0, 2); // mode_extension
        bw.put(0, 1); // copyright
        bw.put(1, 1); // original/home
        bw.put(0, 2); // emphasis = none

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
}
