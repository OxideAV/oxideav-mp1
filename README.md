# oxideav-mp1

A pure-Rust **MPEG-1 / MPEG-2 LSF Audio** codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.
Decodes both **Layer I** and **Layer II** (mp2) frames; encodes
**Layer I** *and* **Layer II** (see the `make_encoder_layer2` /
`EncodeParams::layer` switch below).

## Status

**Clean-room rebuild — encode + decode round-trip, MPEG-1 and MPEG-2 LSF
(2026-05-24).** The prior
implementation was retired under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/IMPLEMENTOR_ROUND.md):
the provenance of its 512-tap synthesis-window data table could not be
defended as clean-room — the module doc-comment recorded that the
values had been transcribed from an external library's source file
rather than read solely from the ISO/IEC specification. Master history
was fully erased per the Hat-3 cold-enforcement procedure.

The rebuild reads numeric tables **only** from ISO/IEC 11172-3 (1993),
the MPEG-1 audio standard (157-page edition with Annex B), and from
ISO/IEC 13818-3 (1997) §2.4.2.3 for the MPEG-2 LSF (Lower Sampling
Frequencies) redefinitions.

## What works today

The crate decodes and encodes complete MPEG-1 *and* MPEG-2 LSF Audio
**Layer I** frames to/from interleaved S16 PCM, all from ISO/IEC 11172-3
plus the ISO/IEC 13818-3 §2.4.2.3 LSF extension:

- **32-bit frame header** (§2.4.1.3 / §2.4.2.3): all thirteen fields
  decoded into the typed [`FrameHeader`] struct. The `ID` bit selects
  the edition: `ID == 1` uses the MPEG-1 Layer I `bitrate_index` ladder
  (32 … 448 kbit/s) and `sampling_frequency` table (32 / 44.1 / 48 kHz);
  `ID == 0` uses the ISO/IEC 13818-3 §2.4.2.3 LSF Layer I ladder (32 /
  48 / 56 / 64 / 80 / 96 / 112 / 128 / 144 / 160 / 176 / 192 / 224 /
  256 kbit/s) and LSF sampling table (16 / 22.05 / 24 kHz). Forbidden
  (`0b1111`) and reserved (`0b11`) values are rejected, free format is
  recognized. `FrameHeader::is_lsf()` reports which edition was parsed.
- **Frame sync** (§2.4.3.1) and **frame-length computation**
  (§2.4.2.1 / §2.4.3.1): slot count
  `N = floor(12 · bitrate / sampling_frequency) + padding_bit`, four
  bytes per Layer I slot.
- **§2.4.3.1 free-format frame-length probe**: when a header carries
  `bitrate_index == 0b0000`, the §2.4.2.1 N formula is uninvertible
  from the header alone (§2.4.3.1: *"If the bitrate index equals
  '0000', the exact bitrate is not indicated. N can be determined
  from the distance between consecutive syncwords and the value of
  the padding bit."*). The new
  [`detect_free_format_frame_length(header, after_header)`] scans the
  payload byte-by-byte for the next stream-matching syncword (same
  `(ID, layer, sampling_frequency, mode)` — free-format implies the
  bitrate is held constant), subtracts the `padding_bit` slot from
  the discovered slot distance, and reports the recovered base slot
  count `N`, the current frame's byte length, and the back-derived
  bitrate `kbps = N · Fs / (L · 1000)` (Layer I `L = 12`, Layer II
  `L = 144`). Rejects non-free headers with `NotFreeFormat`, missing
  candidates with `NoNextSync`, and a Layer I distance that is not a
  whole-slot multiple with `InconsistentDistance`. Stream-parameter
  mismatches in candidate syncwords are skipped over.
- **CRC `error_check()` verification** (§2.4.1.4 / §2.4.3.1):
  `protection_bit` drives whether a 16-bit CRC word follows;
  `FrameHeader::verify_crc` computes the CRC-16
  (`G(X) = X^16 + X^15 + X^2 + 1`, initial state `0xFFFF`) over the
  Annex B Table 3-B.5 protected fields for Layer I — header bits 16…31
  plus the bit-allocation field — and compares it with the stored word,
  returning `CrcStatus::{Absent, Ok, Mismatch}`. On a mismatch the
  decoder applies a **selectable** §2.4.3.1 concealment.
- **Selectable §2.4.3.1 concealment** (`ConcealmentMode`): the spec
  recommends, verbatim, "muting of the actual frame or repetition of
  the previous frame". Both are implemented. `ConcealmentMode::Mute`
  (the default) emits a silent frame and rings the filterbank history
  out with zeros; `ConcealmentMode::RepeatPrevious` re-synthesizes the
  last successfully-decoded frame's requantized subband samples through
  the (now-advanced) filterbank, reproducing that frame's audio rather
  than a silence drop-out. A concealed frame is never stored as the new
  "previous" frame, so a run of corrupt frames repeats the *last good*
  frame each time instead of chaining repeats-of-repeats; if the very
  first frame is corrupt, RepeatPrevious falls back to muting (there is
  nothing to repeat). The mode is chosen at construction
  (`Mp1Decoder::with_concealment`, `decoder::make_decoder_with_concealment`)
  or switched at runtime (`Mp1Decoder::set_concealment`); `reset` drops
  the repeat history but preserves the configured mode.
- **Layer I audio-data decode** (§2.4.1.5 / §2.4.2.5 / §2.4.3.2):
  per-subband 4-bit bit allocation (mapped to 0/2..15 bits/sample,
  rejecting `0b1111`), 6-bit scalefactor index per allocated subband,
  and per-sample requantization (read `nb` bits, invert the MSB, treat
  as a two's-complement fraction, apply the §2.4.3.2 linear formula).
  Handles mono, stereo, dual-channel, and joint-stereo — the
  joint-stereo upper band `[bound, 32)` shares one allocation and one
  sample stream copied to both channels (intensity_stereo). The decode
  path is sample-rate-agnostic, so MPEG-1 and LSF frames decode through
  the same code once the header has selected the right tables.
- **Scalefactor rescale** (§2.4.3.2): each requantized sample is
  multiplied by `scalefactor[index]` from **Table 3-B.1 "LAYER I, II
  SCALEFACTORS"** (Annex B, PDF page 51), transcribed into
  `tables::SCALEFACTORS` and cross-checked against the closed-form
  `scalefactor[i] = 2^((3−i)/3)`.
- **Polyphase synthesis filterbank** (§2.4.3.2, 3-Annex A Figure
  3-A.2): per channel, a 1024-element `V` FIFO carries overlap-add
  history across frames; each of the 12 sample-slots runs the
  32→64 matrixing (`N_ik = cos[(16+i)(2k+1)π/64]`), builds the 512-tap
  `U` vector, windows it by the **Table 3-B.3** coefficients `D[]`
  (Annex B, PDF pages 56–58), and sums to 32 PCM samples — `12 × 32 =
  384` PCM samples per channel per frame.
- **`oxideav_core::Decoder` + registration**: `Mp1Decoder` turns one
  Layer I (or Layer II — see below) packet into an interleaved S16
  `AudioFrame`; `register` installs it under the WAVE format tag
  `0x0050` and the Matroska codec id `A_MPEG/L1`. `reset` zeroes the
  filterbank history after a seek.
- **Layer II `audio_data()` decode** (§2.4.1.6 / §2.4.2.6 / §2.4.3.3
  + Annex B Tables 3-B.2a..d "Possible quantization per subband" and
  3-B.4 "Layer II classes of quantization"): the decoder routes any
  packet whose `layer` field is `'10'` through the §2.4.1.6 syntax —
  one of the four B.2x bit-allocation tables selected by `(sampling
  frequency, bitrate per channel)`, per-subband 2-bit `scfsi`, 1..3
  six-bit Table 3-B.1 scalefactor indices per subband per `scfsi`
  schedule, and twelve granules of either one grouped `samplecode`
  (degrouped by repeated `% nlevels`, `/ nlevels` per §2.4.3.3.4) or
  three separable `sample` reads. Each triplet's §2.4.3.3.4 linear
  formula `s'' = C · (s''' + D)` (constants from Table 3-B.4) is
  rescaled by the per-part Table 3-B.1 multiplier and fed into the
  same §2.4.3.2 polyphase synthesis filterbank the Layer I path uses
  (36 sample-slots per channel = 1152 PCM samples per channel per
  Layer II frame, §2.4.2.1). The intensity_stereo upper band
  (`[bound, sblimit)` in joint_stereo) reads one shared sample stream
  copied into both channels (§2.4.2.6 / §2.4.3.3). A fixture-driven
  integration test decodes an ffmpeg-encoded mono `.mp2` (440 Hz
  sine, 64 kbit/s, 44.1 kHz, 20 frames) and compares the steady-state
  PCM against ffmpeg's reference S16 decode of the same file:
  **RMS = 0.50 LSB, max|err| = 1 LSB across 20 736 samples** —
  essentially bit-exact, the residual being IEEE-754 ordering. The
  Layer II *encoder* (frame writer + quantization with Table C.6
  (A, B) constants + the §C.1.5.2.7 bit allocator + the Table C.5
  SNR ladder + the 13818-3 Annex B Table B.1 LSF Layer II
  allocation table for Fs ∈ {16, 22.05, 24} kHz) is now in tree
  (see below) and reachable from the trait-object [`Mp1Encoder`]
  via the `LayerSelect` switch / [`make_encoder_layer2`] factory.
  The §C.1.5.2.5 SCFSI selection from Table C.4 remains a
  DOCS-GAP — the encoder emits `scfsi == 0b00` for every allocated
  subband.
- **Layer II §2.4.3.1 CRC-16 `error_check()` verification** over the
  §2.4.3.1 + Annex B Table 3-B.5 Layer II protected fields (header
  bits 16…31 + the §2.4.1.6 bit-allocation field + the §2.4.1.6
  scfsi field). [`verify_layer2_crc`] / [`compute_layer2_crc`] walk
  the per-frame Tables 3-B.2x allocation row widths to size the
  allocation field, parse it to find which subbands carry scfsi
  (one scfsi per channel per non-zero allocation, two bits each, per
  the §2.4.1.6 syntax), and feed the concatenated protected bits
  through the same §2.4.3.1 CRC-16 register (`G(X) = X^16 + X^15 +
  X^2 + 1`, init `0xFFFF`) used by the Layer I path. The
  [`Mp1Decoder`] routes every CRC-protected Layer II packet through
  [`verify_layer2_crc`] and, on a mismatch, applies the configured
  [`ConcealmentMode`]: `Mute` emits a 1152-sample silent frame and
  rings the filterbank history out with zeros, while
  `RepeatPrevious` re-synthesizes the last successfully-decoded
  Layer II frame's 36-slot subband samples (a Layer-II-specific
  history kept alongside the Layer I one, since the two
  `Layer2Subbands` / `SubbandSamples` shapes differ in their
  per-subband sample count). The last-good-only repeat semantics and
  the first-frame `Mute` fallback match Layer I; `reset` drops both
  histories.
- **Polyphase analysis filterbank** (§C.1.3, informative Annex C, figure
  C.4 "Analysis Subband Filter Flow Chart"): per channel, a 512-element
  `X` input FIFO carries windowing history across slots; each slot
  shifts in 32 new samples, windows by the **Table C.1** coefficients
  `C[]` (Annex C, PDF pages 68–69), partial-sums to 64 `Y` values
  (`Y[i] = Σ_{j=0}^{7} Z[i+64j]`), and matrixes to 32 subband samples
  with `M[i][k] = cos[(2i+1)(k-16)π/64]`. The analysis+synthesis pair
  reconstructs a sine to within ~2·10⁻⁵ RMS — a sample-exact check that
  validates the Table C.1 sign pattern end-to-end.
- **Scalefactor selection** (§C.1.5.1.4): per subband, the lowest Table
  3-B.1 value larger than the max-absolute subband sample is chosen.
- **Bit allocation** (§C.1.5.1.6): an iterative loop that raises the
  bit count of the subband with the minimal mask-to-noise ratio until
  the frame's `adb` budget is spent. The clean-room encoder uses a
  **non-psychoacoustic** signal-energy-driven SMR proxy (Annex D's
  psychoacoustic models are not implemented); the §2.4.2.5 4-bit
  allocation codes, the §C.1.5.1.6 step budget (`12·(nb_new − nb_old)`
  sample bits plus +6 scalefactor bits on the 0→2 jump), and the Table
  C.2 SNR ladder per `nb` drive the loop.
- **Layer II §2.4.2.3 frame-header writer**: a clean-room pack of the
  thirteen §2.4.1.3 header fields into the four header bytes for a
  Layer II frame. New `encode::pack_layer2_header(&Layer2HeaderParams)
  -> Result<[u8; 4], Layer2HeaderError>` builds the 32-bit big-endian
  word verbatim from §2.4.1.3 (syncword `0xFFF`, `ID` resolved from
  `sampling_frequency` against the §2.4.2.3 / 13818-3 §2.4.2.3
  table, `layer = 0b10`, `protection_bit` driven by `has_crc`,
  `bitrate_index` resolved against the chosen Layer II ladder, plus
  every other field carried verbatim from the caller). New
  `encode::write_layer2_header(&mut BitWriter, &Layer2HeaderParams)`
  streams the same four bytes MSB-first into a `BitWriter` so the
  caller can immediately continue with the §2.4.1.6 allocation
  field, and new `encode::bitrate_index_layer2(kbps, id_bit)`
  exposes the Layer II bitrate-index lookup standalone — the
  MPEG-1 Layer II ladder `{32, 48, 56, 64, 80, 96, 112, 128, 160,
  192, 224, 256, 320, 384}` kbit/s under `ID == 1`, and the LSF
  Layer II/III ladder `{8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112,
  128, 144, 160}` kbit/s under `ID == 0`.
  - Twelve new lib-tests cover: `bitrate_index_layer2` endpoints on
    both ladders (with a cross-rejection sanity check — 448 kbit/s
    is on Layer I but not Layer II), the known bit-by-bit pack of a
    128 kbit/s mono 44.1 kHz header, the `protection_bit` flip when
    `has_crc` is set, the LSF `ID == 0` bit for the three LSF
    sampling frequencies, off-ladder bitrate rejection, unknown
    sampling-frequency rejection, the **full 14 × 3 × 4 (= 168)
    bitrate × sampling-frequency × mode matrix** for the MPEG-1
    Layer II header, the same 14 × 3 × 4 matrix for the LSF Layer
    II header, the padding / private / copyright / original /
    emphasis / mode_extension carry path (each toggled
    independently and round-tripped through `FrameHeader::parse`),
    and the `BitWriter` streaming variant matching the byte-array
    pack byte-for-byte while keeping subsequent MSB-first writes
    byte-aligned.
  - Total `cargo test -p oxideav-mp1 --lib` count: **146 → 158**.
- **Layer II §2.4.1.6 allocation-field writer**: a clean-room emitter
  for the `allocation[ch][sb]` bits a Layer II frame carries after the
  header. New
  `encode::write_layer2_allocation_field(&mut BitWriter, table, alloc,
  nch, bound) -> Result<(), Layer2AllocationFieldError>` packs MSB-first
  per the §2.4.1.6 syntax: the low band `[0, bound)` writes `nch ·
  Σ nbal[sb]` bits (per-channel `nbal`-bit allocations) and the
  intensity_stereo upper band `[bound, sblimit)` writes one shared
  `nbal[sb]` slot per subband (mirrored into both channels by the
  decoder). The function pre-flights every cell before writing a single
  bit and surfaces every edge case as a typed error
  (`UnsupportedChannelCount`, `BoundExceedsSblimit`,
  `MonoBoundBelowSblimit`, `InvalidAllocationCode` for out-of-`nbal`
  values or `-` cells in the Table 3-B.2x row, `NonZeroAllocationAboveSblimit`
  for the silent-drop guard, and `UpperBandChannelsDisagree` for shared
  upper-band cells whose two channels carry different values).
  Companion helper `encode::layer2_stereo_bound(&FrameHeader, sblimit)`
  exposes the §2.4.1.6 bound resolution (the joint_stereo
  `mode_extension` lookup of `{4, 8, 12, 16}` clamped to `sblimit`,
  `sblimit` for every other mode) so callers driving the writer from
  an already-parsed [`FrameHeader`] don't duplicate the decoder's
  bound-derivation logic.
  - Eleven new lib-tests cover: the bound helper across all four
    [`Mode`] variants and every joint_stereo `mode_extension` code
    (including the `mode_extension == '11'` clamp to `sblimit = 8`
    under B.2c), the all-zero stereo write whose payload size matches
    `nch · Σ nbal[sb]` exactly, a known-bit B.2a low-band pattern at
    48 kHz / 192 kbit/s stereo confirming sb-major / ch-minor packing
    (sb 0 ch 0 in the high nibble of byte 0, sb 0 ch 1 in the low
    nibble), a joint_stereo B.2c write whose total payload is shorter
    than the unshared layout by exactly `Σ_{bound..sblimit} nbal[sb]`,
    every typed rejection path (`UnsupportedChannelCount` at 0 and 3,
    `BoundExceedsSblimit`, `MonoBoundBelowSblimit`,
    `InvalidAllocationCode` for an oversize 3-bit code,
    `NonZeroAllocationAboveSblimit`, and `UpperBandChannelsDisagree`),
    plus a `BitReader` round-trip that re-reads every emitted cell of
    a joint_stereo frame and recovers the exact allocation code.
  - Total `cargo test -p oxideav-mp1 --lib` count: **158 → 169**.
- **Layer II §2.4.1.6 scfsi + scalefactor field writer**: a clean-room
  emitter for the two §2.4.1.6 bitstream regions that follow the
  allocation field — first the per-(ch, sb) 2-bit `scfsi` codes (one
  per non-zero allocation, sb-major / ch-minor over `[0, sblimit)`,
  including the intensity_stereo upper band where the *allocation* is
  shared between channels but `scfsi` is still read once per channel),
  then the 1..3 six-bit Table 3-B.1 scalefactor indices per (ch, sb)
  emitted per the §2.4.2.6 SCFSI schedule (`0b00`: three reads,
  `0b01`: two reads broadcast over parts 0+1 then part 2 alone,
  `0b10`: one read broadcast over all three parts, `0b11`: part 0 alone
  then one read broadcast over parts 1+2). New
  `encode::write_layer2_scalefactor_field(&mut BitWriter, table, alloc,
  input, nch, bound) -> Result<(), Layer2ScalefactorFieldError>` packs
  both phases MSB-first; the function pre-flights every cell before
  writing a single bit and surfaces every edge case as a typed error
  (`UnsupportedChannelCount`, `BoundExceedsSblimit`,
  `MonoBoundBelowSblimit`, `InvalidScfsiCode` for `scfsi ≥ 4`,
  `InvalidScalefactorIndex` for any 6-bit value `≥ 63` — the
  reserved/forbidden index that conformant encoders must not emit —
  and `ScfsiPartsInconsistent01` / `…10` / `…11` for SCFSI codes whose
  collapse rule the caller's per-part array does not already
  satisfy, refusing to silently lose information). New input struct
  `Layer2ScalefactorFieldInput { scfsi, scalefactor_indices }` carries
  the per-(ch, sb) scfsi codes and per-(ch, sb, part) Table 3-B.1
  indices.
  - Thirteen new lib-tests cover: an all-zero allocation writing zero
    bits (the §2.4.1.6 region collapses when no subband is allocated);
    a dense-allocation bit-count check that mixes all four SCFSI codes
    across (ch, sb) and confirms the writer's byte count matches the
    closed-form `2·N_alloc + Σ schedule[scfsi[i]]` total (`schedule =
    {18, 12, 6, 12}` for `scfsi ∈ {0b00, 0b01, 0b10, 0b11}`); a
    `BitReader` round-trip over a B.2a stereo frame that cycles all
    four SCFSI codes per (ch, sb) and recovers every scfsi value and
    every per-part scalefactor index bit-exact; a known-bit two-byte
    trace (mono, scfsi=`0b10`, parts=[9, 9, 9] → `0x89`); a known
    three-byte trace (mono, scfsi=`0b00`, parts=[1, 2, 3] → `0x01 0x08
    0x30`); a sparse-allocation test confirming unallocated subbands
    emit zero bits; every typed rejection path
    (`UnsupportedChannelCount` at 0 and 3, `BoundExceedsSblimit`,
    `MonoBoundBelowSblimit`, `InvalidScfsiCode` for `scfsi = 4`,
    `InvalidScalefactorIndex` for `index = 63`, and each of
    `ScfsiPartsInconsistent01`, `ScfsiPartsInconsistent10`,
    `ScfsiPartsInconsistent11` for diverging per-part inputs).
  - Total `cargo test -p oxideav-mp1 --lib` count: **169 → 182**.
- **Layer II §2.4.1.6 / §2.4.3.3.4 SAMPLES region writer**: a clean-room
  emitter for the §2.4.1.6 SAMPLES region — the audio-data payload that
  immediately follows the scfsi + scalefactor region. New
  `encode::write_layer2_samples_field(&mut BitWriter, table, alloc,
  input, nch, bound) -> Result<(), Layer2SamplesFieldError>` mirrors
  the §2.4.1.6 / §2.4.3.3.4 decoder loop: 12 syntax-granules outer;
  per granule, for each `(ch, sb)` in the low band `[0, bound)` it
  emits one triplet per channel; in the shared upper band `[bound,
  sblimit)` it emits a single triplet sourced from channel 0 (the
  §2.4.2.6 / §2.4.3.3 intensity_stereo rule mirroring into both
  channels at decode time). Each triplet is emitted per the per-`(sb,
  alloc)` quantization-class grouping flag — grouped classes pack
  three `0..nlevels` codes into one `bits_per_codeword`-wide field as
  `s0 + s1·N + s2·N²` (the exact inverse of the decoder's `c % N;
  c /= N` degrouping loop); non-grouped classes emit three separable
  `bits_per_codeword`-wide fields. Subbands with `alloc[ch][sb] == 0`
  and subbands `[sblimit, 32)` emit zero bits (§2.4.3.3.5
  silenced-band rule). New input type
  `encode::Layer2SamplesFieldInput { codes: [[[[u32; 3]; SUBBANDS];
  12]; 2] }` carries the per-`(ch, gr, sb)` triplet of MSB-inverted
  unsigned codes the §2.4.3.3.4 decoder will read. The function
  pre-validates every cell before writing a single bit and surfaces
  five typed errors: `UnsupportedChannelCount`, `BoundExceedsSblimit`,
  `MonoBoundBelowSblimit`, `InvalidAllocationCode` (a non-zero `alloc
  [ch][sb]` pointing at a `-` cell in the per-subband Tables 3-B.2x
  row), and `SampleCodeOutOfRange` (any code `≥ nlevels` for the
  resolved class — the §2.4.3.3.4 MSB-inversion + degrouping math
  would silently corrupt the recovered samples otherwise).
  - Eleven new lib-tests cover: all-zero-allocation emits no bits; a
    dense-allocation total-bit-count check matching the per-class
    `bits_per_codeword` sum across `(ch, sb)` (with a `pad_bits < 8`
    invariant on the trailing partial byte); a known-bit B.2c sb=0
    grouped trace (12 grouped `samplecode = 5` codewords → `0x29 0x4A
    0x52 0x94 0xA5 0x29 0x4A 0x50`); a full alloc + scfsi + scf +
    samples write that decodes back through `decode_layer2_audio_data`
    with every recovered §2.4.3.3.4 sample within `1e-12` of the
    closed-form expectation re-derived from the codes / scalefactors
    written (stereo, B.2a sb=0); a joint-stereo `mode_extension =
    0b00` write whose shared upper band sources triplets from channel
    0 only and the decoder mirrors into both channels (verified
    per-channel against the same closed-form re-derivation, with the
    channel-1 input codes left at zero to confirm the writer ignores
    them); and every typed rejection path (`UnsupportedChannelCount`
    at 0 and 3, `BoundExceedsSblimit`, `MonoBoundBelowSblimit`,
    `InvalidAllocationCode`, `SampleCodeOutOfRange` at `code =
    nlevels`, plus a deep-position rejection at `(ch=1,
    sb=sblimit-1, gr=11)` confirming pre-flight emits zero bytes on
    error). Together with the prior `write_layer2_header`,
    `write_layer2_allocation_field` and
    `write_layer2_scalefactor_field` this completes the four-region
    §2.4.1.6 control + audio-data payload of a Layer II frame; a
    later round wired them behind the top-level
    `encode::encode_layer2_frame` (see below). The remaining Layer II
    encoder followups are the §C.1.5.2.5 / Table C.4 perceptual SCFSI
    selection (still a PDF-image DOCS-GAP, hence the encoder writes
    `scfsi == 0b00` for every allocated subband) and an
    `Mp1Encoder`-style trait-object integration with a
    `EncodeParams::layer` switch surfacing both Layer I and Layer II
    through one `oxideav_core::Encoder`.
  - Total `cargo test -p oxideav-mp1 --lib` count: **182 → 193**.
- **Layer II scalefactor extraction** (§C.1.5.1.4 per part): a clean-room
  encoder-side helper that turns the 36 analysed sub-band samples of
  one Layer II frame into the three Table 3-B.1 scalefactor *indices*
  the §2.4.1.6 scalefactor field consumes per (ch, sb). The 36 slots
  are split into the three §2.4.2.6 / §2.4.3.3.2 scalefactor parts
  (slots `0..12`, `12..24`, `24..36` — "three equal parts of 12 subband
  samples"); for each part the maximum of the absolute value of the 12
  samples is determined, then `select_scalefactor` picks "the lowest
  value in Table B.1 … which is larger than this maximum". Exposed as
  `encode::select_layer2_scalefactors(subbands, nch, sblimit) ->
  Layer2ScalefactorIndices` plus the per-part-peak primitive
  `encode::layer2_subband_peak_per_part(subbands, nch, sblimit) ->
  Layer2SubbandPeaks`; both return shapes drop straight into
  `Layer2ScalefactorFieldInput::scalefactor_indices` so the per-part
  indices flow into `write_layer2_scalefactor_field` unchanged when
  the encoder writes `scfsi == 0b00` (three independent scalefactors —
  the Table C.4 SCFSI collapse remains a PDF-image DOCS-GAP). New
  `Layer2ScalefactorIndices = [[[u8; 3]; SUBBANDS]; 2]` and
  `Layer2SubbandPeaks = [[[f64; 3]; SUBBANDS]; 2]` type aliases name
  the shape callers see.
  - Five new lib-tests cover: the part-windowing split (stamping
    distinct peaks into disjoint 12-slot windows and asserting each
    part recovers only its own peak); the `nch` / `sblimit` masking
    (peaks outside the active rows/columns stay zero even when data is
    present); the per-part agreement with `select_scalefactor` (three
    distinct peaks per part, three distinct indices); the all-zero
    fallback to the tiniest-multiplier index `62` (index `63` is
    §2.4.2.6 reserved and never produced); and a full extractor →
    §2.4.1.6 field-writer → `BitReader` round-trip on a mono single-
    subband fixture (3 bytes / 20 bits — scfsi + three scalefactors,
    recovered bit-exact).
  - Total `cargo test -p oxideav-mp1 --lib` count: **222 → 227**.
- **ISO/IEC 13818-3 Annex B Table B.1 — LSF Layer II allocation
  table**: the §2.4.3.1 substitution rule for MPEG-2 LSF Layer II
  decoding is now spelled out exactly per the spec. Earlier rounds
  aliased the LSF path to MPEG-1 Layer II Table 3-B.2b (`sblimit =
  30`, `Σ nbal = 94`) on the strength of a matching `sblimit`;
  13818-3 §2.4.3.1 actually substitutes **Table B.1 "Possible
  quantisation per subband, Layer II — Sampling frequencies 16; 22,05;
  24 kHz"** (printed p.71 / PDF page 81) for B.2a..d at every LSF
  bitrate, and the per-subband ladder is strictly narrower:
  `nbal = 4` for `sb 0..=3` (15-column ladder `{3, 5, 7, 9, 15, 31,
  63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383}`), `nbal = 3` for
  `sb 4..=10` (7-column ladder `{3, 5, 9, 15, 31, 63, 127}`),
  `nbal = 2` for `sb 11..=29` (3-column ladder `{3, 5, 9}`), and
  `nbal = 0` for `sb 30..=31` (silenced). `Σ nbal = 4·4 + 7·3 + 19·2
  = 75` matches the footer printed below the table. Implemented in
  `tables_layer2::TABLE_LSF` and wired into the
  `layer2_bit_allocation_table(header)` selector — Layer II frames
  with `ID == 0` now resolve to `TABLE_LSF` rather than `TABLE_B2B`,
  so the Layer II decoder, the `error_check()` CRC sizing, and the
  Layer II frame writers (`write_layer2_allocation_field`,
  `write_layer2_scalefactor_field`, `write_layer2_samples_field`)
  all see the correct per-subband `nbal` widths and per-row quant
  classes at 16 / 22.05 / 24 kHz without any further per-region
  writer-logic changes (the writers read `nbal(sb)` and
  `quant_class(sb, alloc)` straight off the `AllocationTable`).
  - Seven new lib-tests cover: the per-subband `nbal` pattern (`{4,
    4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, …, 2, 0, 0}`) and `Σ nbal = 75`
    total across `{16, 22.05, 24} kHz × {8, 64, 144, 160} kbit/s`;
    the `sb 0..=3` row's 15-column quant-class resolution against the
    exact spec sequence; the `sb 4..=10` row's 7-column resolution;
    the `sb 11..=29` row's 3-column resolution; the `sb 30..=31`
    `nbal = 0` silenced behaviour; a structural-distinction test
    showing Table B.1 differs from B.2b at `nbal(5)` (B.1 = 3, B.2b =
    4) and at the column count for `sb = 11` (B.1 has 3, B.2b has 7);
    and a pointer-identity sweep over the full 13818-3 §2.4.2.3 Layer
    II/III LSF bitrate ladder (`{8, 16, 24, 32, 40, 48, 56, 64, 80,
    96, 112, 128, 144, 160}` kbit/s) at each of the three LSF rates
    confirming the §2.4.3.1 "for all bitrates" invariance.
  - Total `cargo test -p oxideav-mp1 --lib` count: **242 → 259**.
- **§2.4.1.6 / §C.1.5.2 Layer II top-level frame encoder**: the four
  §2.4.1.6 region writers, the per-part scalefactor extractor, the
  §C.1.5.2.7 bit allocator and a new §C.1.5.2 / §2.4.3.3.4 per-sample
  quantizer are now wired behind a single
  `encode::encode_layer2_frame(&Layer2HeaderParams, &subbands) ->
  Result<Vec<u8>, Layer2EncodeError>`. Callers supply only the
  §2.4.1.3 header parameters (sampling frequency, Layer II bitrate,
  channel mode, joint-stereo bound, optional §2.4.1.4 CRC opt-in) and
  the analysed `[ch][sb][slot]` sub-band matrix; the function emits a
  complete §2.4.2.1 Layer II frame — header + optional CRC +
  §2.4.1.6 allocation, scfsi, scalefactor and samples regions +
  zero-padded ANC tail — exactly `floor(144 · bitrate / Fs) +
  padding_bit` bytes long. New §2.4.3.3.4 inverse-quantizer
  `encode::quantize_layer2_sample(value, scf, &QuantClass) -> u32`
  computes the raw `nlevels`-level code the §2.4.1.6 SAMPLES writer
  consumes, inverting the decoder's `s'' = C · (s''' + D)` formula
  step-by-step and clamping into `[0, nlevels)` so the writer's
  `SampleCodeOutOfRange` pre-flight is never tripped on a
  scalefactor-saturating signal. In intensity-stereo upper bands
  (`sb ∈ [bound, sblimit)`) per-channel peaks are pre-mirrored before
  the allocator runs, and the resulting per-channel allocations are
  shared post-hoc so the §2.4.1.6 writer's
  `UpperBandChannelsDisagree` invariant always holds. The encoder
  writes `scfsi == 0b00` (three independent scalefactors per
  allocated `(ch, sb)`) on every cell — Table C.4's perceptual
  collapse remains a PDF-image DOCS-GAP and the worst-case
  bookkeeping `allocate_bits_layer2` already reserves matches the
  three-scalefactor cost. New `Layer2EncodeError` enumerates the
  channel-count, header (`Layer2HeaderError` surface),
  channel-mode-mismatch and `FrameTooSmall` rejection paths.
  - Seven new lib-tests cover: a sweep across every Table 3-B.4
    quantization class active in B.2a (48 kHz / 192 kbit/s stereo)
    confirming `quantize_layer2_sample` recovers every legal code on
    every quantizer step (the strict inverse of the decoder's
    `requantize_triplet`); an out-of-range PCM clamp test
    (`±10.0`, `±2.0`, `1e6`) that keeps every emitted code in
    `[0, nlevels)`; a mono 48 kHz / 128 kbit/s end-to-end encode →
    decode round trip whose recovered samples sit within one
    quantizer step of the analysed input on every allocated subband
    (with `peak_err_within_grid` walking each allocated `(sb,
    slot)`); a joint-stereo 44.1 kHz / 192 kbit/s round-trip that
    checks the shared upper band carries matching allocations and the
    decoded ch0/ch1 sample ratio equals the scalefactor ratio (the
    §2.4.1.6 shared-`s_dp` invariant); a §2.4.1.4 CRC opt-in
    round-trip that verifies the encoder's CRC through
    `verify_layer2_crc`; an off-ladder-bitrate rejection surfaced as
    `Layer2EncodeError::Header(UnsupportedBitrate(100))`; and a LSF
    (24 kHz / 64 kbit/s mono) Layer II round-trip exercising the
    `ID == 0` Table B.1 path through the same top-level entry point.
  - Total `cargo test -p oxideav-mp1 --lib` count: **259 → 266**.
- **`Mp1Encoder` §2.4.1.5 / §2.4.1.6 Layer-I / Layer-II top-level
  dispatch switch**: the `oxideav_core::Encoder` trait object the
  registry hands back from [`make_encoder`] (and the direct factory
  twin [`encoder::make_encoder`]) can now drive *either* the Layer I
  [`Mp1FrameEncoder`] *or* the Layer II [`Mp1Layer2FrameEncoder`]
  through one wrapper. The selection rides on a new
  `EncodeParams::layer: LayerSelect` field (default
  `LayerSelect::LayerI`, preserving byte-for-byte compatibility with
  the encoder's pre-switch behaviour), threaded via the
  `EncodeParams::with_layer(LayerSelect)` builder. A new factory pair
  exposes the Layer II branch directly:
  `encoder::make_encoder_layer2(&CodecParameters) ->
  Result<Box<dyn Encoder>, Error>` mirrors [`make_encoder`]'s
  `sample_rate` + `channels` (required) / `bit_rate` (optional)
  contract but defaults its bitrate to a per-rate / per-channel
  midpoint on the §2.4.2.3 Layer II ladder (128 / 192 kbit/s at the
  MPEG-1 rates, 64 / 96 at the 13818-3 §2.4.2.3 LSF rates), drives
  [`Mp1Layer2FrameEncoder`], and consumes 1152 PCM samples per
  channel per `send_frame` (the Layer II §2.4.2.1 frame granularity)
  rather than the Layer I 384. The wrapper threads the per-layer
  granularity through `send_frame` rejection: a Layer I encoder
  rejects a 1152-sample frame and a Layer II encoder rejects a
  384-sample frame, each with the matching count in the error text.
  The §C.1.5.2.5 / Table C.4 perceptual SCFSI selection is still a
  PDF-image DOCS-GAP, so the Layer II branch keeps emitting `scfsi
  == 0b00` (three independent scalefactors) for every allocated
  subband per the inherited [`Mp1Layer2FrameEncoder`] behaviour — the
  dispatch switch is purely a routing change and does not alter the
  per-layer bitstream shape either branch produces.
  - **+4 lib-tests** cover: a regression that the default
    [`make_encoder`] factory keeps emitting Layer I (`layer == 0b11`)
    and rejects a 1152-sample send as off-granularity; a Layer II
    factory check that 48 kHz / 128 kbit/s mono produces a frame
    whose header parses as Layer II (`layer == 0b10`), whose length
    matches the §2.4.2.1 `floor(144 · bitrate / Fs)` value (384
    bytes at this configuration), and which rejects a 384-sample
    send; a 44.1 kHz / 192 kbit/s stereo Layer II encode → decode
    round-trip across six consecutive frames asserting the decoder
    surfaces 1152 samples / 4608 interleaved S16 bytes per frame and
    that the run carries non-silence end-to-end (the synthesis
    filterbank ramps); and a parameter-validation rejection test
    matching the Layer I factory's behaviour for missing
    `sample_rate` / `channels`. Public surface adds:
    `LayerSelect` re-exported at the crate root,
    `EncodeParams::with_layer`, the inner [`Mp1Encoder`]'s
    `samples_per_frame` dispatch, and
    `encoder::make_encoder_layer2`.
  - Total `cargo test -p oxideav-mp1 --lib` count: **278 → 282**.
- **§2.4.1.8 `ancillary_data()` emission on the Layer II encoder
  side**: a new
  `encode::encode_layer2_frame_with_ancillary(&Layer2HeaderParams,
  &subbands, &[u8]) -> Result<Vec<u8>, Layer2EncodeError>` is the
  ancillary-aware companion to `encode_layer2_frame`. The caller's
  `ancillary_bit` payload (bslbf; spec §2.4.1.8) is copied into the
  §2.4.2.1 frame tail that begins immediately after the §2.4.1.6
  audio-data region — `BitWriter::finish` byte-aligns the partial
  trailing byte of the samples region first, so the ancillary tail
  always starts on a whole-byte boundary. Any §2.4.2.1 frame bytes
  the payload does not fill are zero-padded. A payload larger than
  the §2.4.1.8 tail capacity surfaces a typed
  `Layer2EncodeError::AncillaryTooLarge { space, got }`. The
  §2.4.3.1 CRC patch — when `Layer2HeaderParams::has_crc` is true —
  runs after the ancillary copy and continues to verify clean
  through `verify_layer2_crc` because the Annex B Table 3-B.5
  protected region (header bits 16…31 + allocation + scfsi) excludes
  the §2.4.1.8 tail (the CRC word at frame bytes 4..6 is
  bit-identical to the no-ancillary reference frame).
  - On `Mp1Layer2FrameEncoder` a paired `set_pending_ancillary(&[u8])`
    / `pending_ancillary()` / `clear_pending_ancillary()` staging API
    routes a one-shot ancillary payload into the next `encode_frame`
    call (success or `AncillaryTooLarge`). `reset()` also drops a
    staged payload so a seek doesn't leak ancillary bytes into the
    next frame. The default behaviour — no staged payload —
    zero-pads the §2.4.1.8 tail, preserving byte-for-byte
    compatibility with frames produced by the existing
    `Mp1Layer2FrameEncoder` API.
  - Five new lib-tests cover: a `Mp1Layer2FrameEncoder` mono 48 kHz /
    192 kbit/s silence-input frame where the staged payload appears
    at the first byte where the ancillary frame diverges from a
    reference no-ancillary frame, the post-payload bytes are
    zero-padded, and the decoder's §2.4.1.6 allocation map is
    unchanged; an oversized-payload path surfacing the
    `AncillaryTooLarge { space, got }` variant; a
    `set_pending_ancillary` → `encode_frame` → `encode_frame`
    "consumed once" round-trip; a `clear_pending_ancillary` test
    comparing against a fresh encoder; and a CRC + ancillary
    round-trip that confirms the §2.4.3.1 CRC word at frame bytes
    4..6 is bit-identical to the no-ancillary reference and the
    staged payload still lands at the first differing byte. Public
    surface add: re-exported `encode_layer2_frame_with_ancillary` at
    the crate root.
  - Total `cargo test -p oxideav-mp1 --lib` count: **273 → 278**.
- **§C.1.3 stateful Layer II frame encoder `Mp1Layer2FrameEncoder`**:
  a new `encode::Mp1Layer2FrameEncoder` carries one `AnalysisFilter`
  per channel — the same §C.1.3 input-FIFO state the Layer I
  `Mp1FrameEncoder` owns — and exposes
  `encode_frame(pcm: &[f64]) -> Result<Vec<u8>, Layer2EncodeError>`
  which consumes exactly `LAYER2_SAMPLES_PER_FRAME` (= 1152)
  interleaved PCM samples per channel, runs 36 slots × 32-PCM-sample
  analysis to populate the `subbands[ch][sb][slot]` matrix, and
  dispatches to the underlying `encode_layer2_frame`. This is the
  Layer II analogue of `Mp1FrameEncoder` (which packs 12 slots × 32
  sub-bands = 384 PCM samples per channel per Layer I frame): callers
  can now produce complete Layer II frames from time-domain PCM
  without having to run the analysis bank themselves. The §2.4.1.4
  CRC opt-in flows through `Layer2HeaderParams::has_crc` exactly as
  for the underlying frame-encoder, and a new
  `Layer2EncodeError::WrongSampleCount { got }` surfaces a PCM length
  that is not exactly `1152 · channels`. `reset()` zeros the
  per-channel analysis-filter FIFO for a seek / stream restart;
  `channels()` reports the header-implied 1 / 2 channel count;
  `params()` exposes a read-only view of the configured header.
  - Seven new lib-tests cover: a mono 48 kHz / 128 kbit/s round trip
    that confirms the §2.4.2.1 byte count, the `Layer::II` /
    `ID == 1` header re-parse and that the encoder places at least
    one non-zero allocation; a joint-stereo 44.1 kHz / 192 kbit/s
    round trip asserting the §2.4.1.6 shared-upper-band invariant
    (ch0 and ch1 allocations identical for
    `sb ∈ [bound, sblimit)`) and that each channel placed at least
    one allocation; an LSF mono 24 kHz / 64 kbit/s round trip
    through Table B.1 (`ID == 0`); a `WrongSampleCount` rejection on
    a 1024-sample input; an off-ladder
    `Layer2HeaderError::UnsupportedBitrate(100)` surfacing through
    `Layer2EncodeError::Header(..)`; a `reset` test priming one
    encoder with a divergent signal then resetting and re-encoding
    a second signal byte-for-byte identically to a fresh encoder
    given the same signal; and a `has_crc == true` round trip whose
    §2.4.3.1 CRC verifies through `verify_layer2_crc` at unchanged
    §2.4.2.1 byte count. Public surface add: re-exported
    `Mp1Layer2FrameEncoder` at the crate root.
  - Total `cargo test -p oxideav-mp1 --lib` count: **266 → 273**.
- **Annex D Phase-3 — Step 3 `LTq` offset + Model 2 spreading
  pieces**: closed-form helpers from the text-extractable portions of
  Annex D continue to land in the [`psy`] module without touching the
  decoder or the energy-driven allocator. `psy::ltq_offset_db(kbps)`
  encodes the §D.1 Step 3 prose `-12 dB for bit rates >= 96 kbits/s
  and 0 dB for bit rates < 96 kbits/s per channel` — the `kbps`
  argument is the **per-channel** rate, matching the spec wording, and
  the boundary at `96` is inclusive on the `-12 dB` side
  (`ltq_offset_db(95) == 0`, `ltq_offset_db(96) == -12`). The clause
  D.2 Model 2 spreading-function `tmpx` / `x` / post-step `sprdngf`
  triplet is staged as `psy::model2_tmpx(j, i)`, `psy::model2_x(tmpx)`
  and `psy::sprdngf_from_tmpy(tmpy)`: `tmpx = 1.05 · (j − i)`, `x = 8
  · min((tmpx − 0.5)² − 2·(tmpx − 0.5), 0)` (peaks at `tmpx = 0.5`,
  zero again at `tmpx = 2.5`, clamped non-positive everywhere), and
  `sprdngf = 0` when `tmpy < −100 dB` else `10^(tmpy/10)`. The
  intermediate `tmpy = …` line that bridges `x` to `sprdngf` is
  typeset as a PDF image and is the remaining Model 2 DOCS-GAP — the
  legible pieces around it are now in tree so once `tmpy` becomes
  text-extractable the missing step plugs straight in.
  - Fifteen new lib-tests cover: every per-channel rate from 8 to
    448 kbit/s mapped to the right offset, the 95/96 boundary, the
    `tmpx` sign convention (`j > i ⇒ tmpx > 0`), the `model2_x`
    peak / zero-again points, the non-positivity clamp across
    `[-3, 10]`, the `(0.5, 2.5)` strictly-negative interval matching
    the closed form, the `tmpx ≤ 0.5` and `tmpx ≥ 2.5` clamp-to-zero
    regions, the `sprdngf` cutoff for `tmpy < -100`, the `tmpy = -100`
    inclusive boundary (`10^-10`), the `10^(tmpy/10)` conversion at a
    handful of legible levels, and monotonicity of `sprdngf` in `tmpy`
    inside the active region.
  - Total `cargo test -p oxideav-mp1 --lib` count: **227 → 242**.
- **Layer II bit allocation core** (§C.1.5.2.7): the iterative
  Layer-II allocator that walks each subband's Table 3-B.2x column
  ladder (skipping `-` cells), accounting for the §2.4.2.1 frame
  budget `adb = (144·bitrate/Fs)·8 − bhdr − bcrc − bbal` and the
  per-subband bookkeeping cost on the first non-zero allocation (2-bit
  `scfsi` plus three 6-bit scalefactor indices — the worst case under
  the §C.1.5.2.5 / Table C.4 selection, since Table C.4 is rendered
  as a PDF image the text layer cannot extract reliably). Exposed as
  `encode::allocate_bits_layer2(energy, nch, table, budget_bits)` with
  the companion `layer2_frame_payload_bits` / `sum_nbal_per_channel`
  helpers. The per-iteration "minimum-MNR" search uses Table C.5
  (`tables_layer2::layer2_snr_db`) — verbatim from PDF page 76 with
  the three Layer-II-only rows (5 → 11.00, 9 → 20.84, 65535 →
  98.01 dB) and overlap parity against Table C.2 at the shared
  `nlevels` rows.
- **Uniform quantization** (§C.1.5.1.7): each subband sample is
  normalized by its scalefactor, `A·X + B` is computed from the **Table
  C.3** coefficients `A = (2^nb − 1)/2^nb`, `B = −1/2^nb`, scaled to
  signed `nb`-bit two's-complement, then the MSB is inverted — the
  exact inverse of the §2.4.3.2 decoder requantization (each (nb, code)
  round-trips bit-exact through `quantize → requantize`).
- **Frame assembly** (§C.1.5.1.10, figure C.2): HEADER, ALLOC,
  SCALEFACTORS, SAMPLES, ANC packed MSB-first into `12·bitrate/Fs` slots
  of 4 bytes each.
- **`oxideav_core::Encoder` + registration**: `Mp1Encoder` turns a
  384-sample-per-channel interleaved S16 `AudioFrame` into one Layer I
  packet; `register_codecs` installs it alongside the decoder. The
  factory takes `sample_rate`, `channels` and optional `bit_rate` from
  `CodecParameters`, accepting all six Layer I rates — the MPEG-1 32 /
  44.1 / 48 kHz (default 192 kbit/s mono, 256 stereo) and the LSF 16 /
  22.05 / 24 kHz (default 96 kbit/s mono, 128 stereo) — and writing the
  `ID` bit and bitrate-ladder index that match. Validated by
  **self-roundtrip**: a 1 kHz tone encodes + decodes back to PCM at
  RMS < 0.01 (≈ −40 dBFS) at 192 kbit/s mono, and stereo / white-noise /
  LSF (16 / 22.05 / 24 kHz) inputs reconstruct within similar bounds.
- **Optional §2.4.1.4 CRC emission on the encode side**: the new
  `EncodeParams::emit_crc` flag (also reachable via the
  [`EncodeParams::with_emit_crc`] builder) controls whether the encoder
  writes the optional `error_check()` field. When `true` the encoder
  writes `protection_bit == 0` and a 16-bit CRC word computed via
  §2.4.3.1 (`G(X) = X^16 + X^15 + X^2 + 1`, init `0xFFFF`) over the
  Annex B Table 3-B.5 Layer I protected fields (header bits 16…31 + the
  bit-allocation field); when `false` (default) the encoder still emits
  `protection_bit == 1` and no CRC, preserving byte-for-byte
  compatibility with the prior encoder output. The CRC's 16 bits are
  taken out of the per-frame audio-data budget (`adb`), so the §2.4.2.1
  slot count is unchanged — a CRC-on and CRC-off encode of the same
  PCM produce the same byte count. New high-level factories
  [`encoder::make_encoder_with_crc`] (direct API) and
  `codec::make_encoder_with_crc` (registry path) build a boxed
  `Mp1Encoder` with CRC emission enabled; the existing
  [`encoder::make_encoder`] continues to keep `protection_bit == 1`.
- **Dual-API surface**: alongside the registry path (`register` /
  `register_codecs`), the crate exposes the historical *direct* factory
  endpoints — `decoder::make_decoder(&CodecParameters)` and
  `encoder::make_encoder(&CodecParameters)`, each returning a boxed
  `oxideav_core::Decoder` / `Encoder`. They are thin wrappers over the
  exact construction `register_codecs` performs, so a downstream caller
  can build a Layer I codec object without touching the runtime
  registry, mirroring the rest of the workspace. A
  `decoder::make_decoder_with_concealment(&CodecParameters, ConcealmentMode)`
  variant builds the boxed decoder with the §2.4.3.1 concealment
  strategy of the caller's choosing.

266 tests cover both bitrate ladders (MPEG-1 and LSF), every sampling
rate across both editions, all mode / mode_extension / emphasis codes,
padding cases at 44.1 kHz and 22.05 kHz, sync recovery, the §2.4.3.1
CRC-16 (polynomial/init constants, known register steps, protected-
field bit sizing per mode, compute→verify round-trip, corruption
detection in the header and allocation regions for Layer I and now in
the header, allocation **and scfsi** regions for Layer II, and
decoder-level concealment on mismatch — both `ConcealmentMode::Mute`
and `ConcealmentMode::RepeatPrevious`: that repeat reproduces the last
good frame's PCM, does not chain repeats-of-repeats, falls back to
mute on a corrupt first frame, drops its history on `reset` while
preserving the mode, and is switchable at runtime via
`set_concealment` — exercised for both Layer I and Layer II protected
frames), the §2.4.2.5 allocation→bits table, the
MSB-first
bit reader, the §2.4.3.2 requantization formula at several widths,
mono / stereo / joint-stereo audio-data decode, the Table 3-B.1
scalefactor table (spot checks + formula + the 2^-16 quantization of
Table 3-B.3 + its symmetry), the matrixing coefficients, a window
overlap-add unit test, full frame→PCM smoke tests (mono + stereo), the
encoder coverage (the Table C.1 analysis-window endpoints + 2^-21
quantization + magnitude cross-check against Table 3-B.3, the Table C.3
closed-form `A`/`B`, the Table C.2 SNR monotonicity, analysis-matrix
spot values, the round-trip `quantize → requantize` identity at every
(nb, code), scalefactor selection per §C.1.5.1.4, allocator behaviour
under tight budgets, the analysis+synthesis reconstruction test), the
ISO/IEC 13818-3 §2.4.2.3 LSF tables (the LSF sampling-frequency map, the
reserved-rate rejection, the full LSF Layer I ladder, the
MPEG-1-vs-LSF divergence at index 2, and three LSF frame-length cases),
and the self-roundtrip integration tests (silence, mono tone, stereo
tone, white noise, frame shape, output-params, registry round-trip,
plus LSF silence + tone round-trips at 16 / 22.05 / 24 kHz, an LSF
frame-layout / header-round-trip check, LSF stereo silence at all three
LSF rates and an LSF stereo 24 kHz tone round-trip exercising the
§2.4.1.5 sb-major / ch-minor SAMPLES region at the LSF stereo factory
default, and the planar S16P encoder-input branch — mono and stereo
planar inputs produce byte-identical packets to their interleaved
counterparts, with the wrong-plane-count and short-plane rejection
paths covered as well), and the direct-factory
endpoints (`decoder::make_decoder` drives a real mono decode and
`encoder::make_encoder` a real stereo encode through the boxed trait
objects, `decoder::make_decoder_with_concealment` carries the chosen
`ConcealmentMode` through to a real CRC-mismatch repeat while the plain
factory keeps the Mute default, plus the missing-`sample_rate` /
`channels` rejection path), and the §2.4.1.4 optional CRC emission
(default `EncodeParams::emit_crc == false` keeps `protection_bit == 1`;
`with_emit_crc(true)` flips the bit, writes a 16-bit CRC the decoder's
`FrameHeader::verify_crc` accepts as `CrcStatus::Ok`, preserves the
§2.4.2.1 frame byte count, and a bit-flip inside the protected
allocation field is detected as `CrcStatus::Mismatch`; a stereo
encode-with-CRC also verifies clean; the `make_encoder_with_crc`
direct + registry factories produce verifying CRCs and a full
encode-with-CRC → decode loop reaches PCM without tripping the
§2.4.3.1 concealment path). Test bytes are constructed locally from the
§2.4.1 field layouts — no external fixtures.

## Spec gaps (DOCS-GAP)

The staged `ISO_IEC_11172-3-MP3-1993.pdf` (157-page edition) carries
Annex B, so the scalefactor and synthesis-window tables are available;
the §2.4.3.1 CRC-16 polynomial render and Table 3-B.5 protected-field
listing are likewise staged, so CRC verification is now implemented
for **both** Layer I (header bits 16…31 + bit allocation) and Layer II
(header bits 16…31 + bit allocation + scfsi). One gap remains, not
blocking decode to PCM:

1. The **§2.4.3.2 requantization linear formula** and the rescale
   formula are rendered as image regions the text layer does not
   extract; the requantization formula
   `s'' = (2^nb / (2^nb − 1)) · (s''' + 2^(−nb+1))` is corroborated by
   the §2.4.3.2 prose (invert MSB → two's-complement fraction → linear
   formula) and the rescale is the prose's `s' = scalefactor · s''`.

Both §2.4.3.1 CRC-mismatch concealment strategies — *muting* of the
offending frame and *repetition of the previous frame* — are now
implemented and selectable (`ConcealmentMode`). Optional CRC emission
on the encode side is also implemented as of r129
(`EncodeParams::emit_crc` / `encoder::make_encoder_with_crc`); the
default factory still keeps `protection_bit == 1` for byte-compatible
output, opt in to flip the bit and have the encoder write the matching
§2.4.3.1 CRC word.

2. **Annex D psychoacoustic models — partially unblocked, allocator
   still energy-driven.** As of r191 the docs collaborator has
   staged 200-DPI page renders for the dense Annex D tables under
   `docs/audio/mp3/annex-d-renders/`, and the companion text extract
   `docs/audio/mp3/mp3-annex-d-psychoacoustic-extracts.md` carries
   the text-readable portions of §D.1 verbatim. The Phase-2 + Phase-3
   building blocks are now in tree in the [`psy`] module:
   **Tables D.2a–f** (critical-band boundaries for Layer I and
   Layer II at 32 / 44,1 / 48 kHz) as `psy::critical_band_table`,
   the closed-form Step 6 masking-index `av_tm` / `av_nm`, the
   four-piece Step 6 masking-function `vf(dz, X)`, the composite
   Step 6 `LT_{tm,nm}` individual thresholds, the Step 7
   power-domain `LTg` global-threshold sum, the 33-row Table D.5
   coder partition table, the **Step 3 `LTq` bit-rate offset rule**
   (`psy::ltq_offset_db`: `−12 dB` for per-channel rates `≥ 96
   kbit/s`, `0 dB` below), and the **clause D.2 Model 2 spreading
   function** text-extractable pieces (`psy::model2_tmpx`,
   `psy::model2_x`, and the post-step `psy::sprdngf_from_tmpy`
   cutoff `sprdngf = 0` when `tmpy < −100 dB`, else `10^(tmpy/10)`).
   The encoder's bit allocator itself is **still** signal-energy-
   driven — wiring `LTg` into the SMR loop additionally requires
   Tables **D.1a–f** (threshold in quiet `LTq`) and the Model-2
   Tables **D.3a–c** + **D.4a–c** (calculation partition `ωlow /
   ωhigh / bval / minval / TMN` and per-FFT-line absolute threshold),
   and those four-column dense tables still live behind PNG renders
   the text layer does not reliably extract. They are therefore
   DOCS-GAP awaiting a higher-DPI or differently-OCR'd render pass,
   mirroring the existing `annex-b-renders/` PNG → text transcription
   cycle that unblocked Tables B.1 / B.3. The Model 2 `tmpy = …`
   intermediate line that bridges `x` to `sprdngf` is similarly
   typeset as an image in the PDF and is part of the same DOCS-GAP.

## License

MIT — see [LICENSE](./LICENSE).
