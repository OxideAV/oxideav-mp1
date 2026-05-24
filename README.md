# oxideav-mp1

A pure-Rust **MPEG-1 Audio Layer I** (MP1) decoder for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Clean-room rebuild — decode to PCM complete (2026-05-24).** The prior
implementation was retired under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/IMPLEMENTOR_ROUND.md):
the provenance of its 512-tap synthesis-window data table could not be
defended as clean-room — the module doc-comment recorded that the
values had been transcribed from an external library's source file
rather than read solely from the ISO/IEC specification. Master history
was fully erased per the Hat-3 cold-enforcement procedure.

The rebuild reads numeric tables **only** from ISO/IEC 11172-3 (1993),
the MPEG-1 audio standard (157-page edition with Annex B).

## What works today

The crate decodes a complete MPEG-1 Audio **Layer I** frame to
interleaved S16 PCM, all from ISO/IEC 11172-3:

- **32-bit frame header** (§2.4.1.3 / §2.4.2.3): all thirteen fields
  decoded into the typed [`FrameHeader`] struct. The Layer I
  `bitrate_index` ladder and `sampling_frequency` table come straight
  from the §2.4.2.3 tables; forbidden (`0b1111`) and reserved (`0b11`)
  values are rejected, free format is recognized.
- **Frame sync** (§2.4.3.1) and **frame-length computation**
  (§2.4.2.1 / §2.4.3.1): slot count
  `N = floor(12 · bitrate / sampling_frequency) + padding_bit`, four
  bytes per Layer I slot.
- **CRC `error_check()` wiring** (§2.4.1.4): `protection_bit` drives
  whether a 16-bit CRC word follows; `read_crc` returns it as a
  `PresentUnverified` value (see "Spec gaps").
- **Layer I audio-data decode** (§2.4.1.5 / §2.4.2.5 / §2.4.3.2):
  per-subband 4-bit bit allocation (mapped to 0/2..15 bits/sample,
  rejecting `0b1111`), 6-bit scalefactor index per allocated subband,
  and per-sample requantization (read `nb` bits, invert the MSB, treat
  as a two's-complement fraction, apply the §2.4.3.2 linear formula).
  Handles mono, stereo, dual-channel, and joint-stereo — the
  joint-stereo upper band `[bound, 32)` shares one allocation and one
  sample stream copied to both channels (intensity_stereo).
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

55 unit tests cover the whole bitrate ladder, every sampling rate, all
mode / mode_extension / emphasis codes, both padding cases at 44.1 kHz,
sync recovery, the CRC-presence paths, the §2.4.2.5 allocation→bits
table, the MSB-first bit reader, the §2.4.3.2 requantization formula at
several widths, mono / stereo / joint-stereo audio-data decode, the
Table 3-B.1 scalefactor table (spot checks + formula + the 2^-16
quantization of Table 3-B.3 + its symmetry), the matrixing coefficients,
a window overlap-add unit test, and full frame→PCM smoke tests (mono +
stereo). Test bytes are constructed locally from the §2.4.1 field
layouts — no external fixtures.

## Not yet implemented

- A **Layer I encoder** (PCM → bitstream).

## Spec gaps (DOCS-GAP)

The staged `ISO_IEC_11172-3-MP3-1993.pdf` (157-page edition) carries
Annex B, so the scalefactor and synthesis-window tables are available.
Two gaps remain, neither blocking decode to PCM:

1. The **§2.4.3.2 requantization linear formula** and the rescale
   formula are rendered as image regions the text layer does not
   extract; the requantization formula
   `s'' = (2^nb / (2^nb − 1)) · (s''' + 2^(−nb+1))` is corroborated by
   the §2.4.3.2 prose (invert MSB → two's-complement fraction → linear
   formula) and the rescale is the prose's `s' = scalefactor · s''`.
2. The **CRC-16 generator polynomial** `G(X)`, plus Annex B
   Table 3-B.5 and Annex A Figure 3-A.9 — CRC remains wired
   structurally but unverified.

## License

MIT — see [LICENSE](./LICENSE).
