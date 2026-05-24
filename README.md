# oxideav-mp1

A pure-Rust **MPEG-1 / MPEG-2 LSF Audio Layer I** (MP1) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

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
  decoder applies the §2.4.3.1 *muting* concealment.
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
  Layer I packet into a 384-sample interleaved S16 `AudioFrame`;
  `register` installs it under the WAVE format tag `0x0050` and the
  Matroska codec id `A_MPEG/L1`. `reset` zeroes the filterbank history
  after a seek.
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
- **Dual-API surface**: alongside the registry path (`register` /
  `register_codecs`), the crate exposes the historical *direct* factory
  endpoints — `decoder::make_decoder(&CodecParameters)` and
  `encoder::make_encoder(&CodecParameters)`, each returning a boxed
  `oxideav_core::Decoder` / `Encoder`. They are thin wrappers over the
  exact construction `register_codecs` performs, so a downstream caller
  can build a Layer I codec object without touching the runtime
  registry, mirroring the rest of the workspace.

102 tests cover both bitrate ladders (MPEG-1 and LSF), every sampling
rate across both editions, all mode / mode_extension / emphasis codes,
padding cases at 44.1 kHz and 22.05 kHz, sync recovery, the §2.4.3.1
CRC-16 (polynomial/init constants, known register steps, protected-
field bit sizing per mode, compute→verify round-trip, corruption
detection in the header and allocation regions, and decoder-level mute
concealment on mismatch), the §2.4.2.5 allocation→bits table, the
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
plus LSF silence + tone round-trips at 16 / 22.05 / 24 kHz and an LSF
frame-layout / header-round-trip check), and the direct-factory
endpoints (`decoder::make_decoder` drives a real mono decode and
`encoder::make_encoder` a real stereo encode through the boxed trait
objects, plus the missing-`sample_rate`/`channels` rejection path).
Test bytes are constructed locally from the §2.4.1 field layouts — no
external fixtures.

## Spec gaps (DOCS-GAP)

The staged `ISO_IEC_11172-3-MP3-1993.pdf` (157-page edition) carries
Annex B, so the scalefactor and synthesis-window tables are available;
the §2.4.3.1 CRC-16 polynomial render and Table 3-B.5 protected-field
listing are likewise staged, so CRC verification is now implemented.
One gap remains, not blocking decode to PCM:

1. The **§2.4.3.2 requantization linear formula** and the rescale
   formula are rendered as image regions the text layer does not
   extract; the requantization formula
   `s'' = (2^nb / (2^nb − 1)) · (s''' + 2^(−nb+1))` is corroborated by
   the §2.4.3.2 prose (invert MSB → two's-complement fraction → linear
   formula) and the rescale is the prose's `s' = scalefactor · s''`.

The CRC-mismatch concealment implemented is *muting* of the offending
frame; the §2.4.3.1 alternative "repetition of the previous frame" is a
followup, as is optional CRC emission on the encode side (the encoder
currently always sets `protection_bit == 1`).

## License

MIT — see [LICENSE](./LICENSE).
