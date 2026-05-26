# oxideav-mp1

A pure-Rust **MPEG-1 / MPEG-2 LSF Audio** codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.
Decodes both **Layer I** and **Layer II** (mp2) frames; encodes
**Layer I** only.

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
  essentially bit-exact, the residual being IEEE-754 ordering. A full
  Layer II *encoder* (frame writer + §C.1.5.2.5 SCFSI selection from
  Table C.4 + quantization with Table C.6 (A, B) constants) and a
  transcription of the 13818-3 LSF Layer II allocation table for
  Fs ∈ {16, 22.05, 24} kHz remain followups; the §C.1.5.2.7 bit
  allocator and the Table C.5 SNR ladder are now in tree (see below).
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

160 tests cover both bitrate ladders (MPEG-1 and LSF), every sampling
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

2. The **Annex D psychoacoustic models** are not implemented: the
   encoder's bit allocator is signal-energy-driven rather than
   perceptually driven. The staged 157-page `ISO_IEC_11172-3-MP3-1993.pdf`
   carries the Annex D §D.1 (Psychoacoustic Model 1) prose — the
   nine-step SMR algorithm, the masking-index / masking-function
   formulae, the tonal/non-tonal labelling rules — in readable text,
   but the essential numeric tables (D.1a/b/c "Frequencies, critical
   band rates and absolute threshold" and D.2a/b/c "Critical band
   boundaries" for Layer I) appear **only in an OCR-corrupted text
   layer** where digits and decimal separators are unreliable
   (e.g. index `7` rendered as `I`, `74` as `14`, `375,00` as
   `315,oo`, threshold values mixing comma and period separators).
   Transcribing a perceptual threshold table from that render would
   silently introduce numeric errors with no clean source to validate
   against, so the perceptual model is a DOCS-GAP awaiting clean
   Annex D table renders (mirroring the existing `annex-b-renders/`
   PNGs that unblocked Tables B.1 and B.3).

## License

MIT — see [LICENSE](./LICENSE).
