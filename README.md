# oxideav-mp1

A pure-Rust **MPEG-1 Audio Layer I** (MP1) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Clean-room rebuild in progress (started 2026-05-24).** The prior
implementation was retired under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/IMPLEMENTOR_ROUND.md):
the provenance of its 512-tap synthesis-window data table could not be
defended as clean-room — the module doc-comment recorded that the
values had been transcribed from an external library's source file
rather than read solely from the ISO/IEC specification. Master history
was fully erased per the Hat-3 cold-enforcement procedure.

The rebuild reads numeric tables **only** from ISO/IEC 11172-3 (1993),
the MPEG-1 audio standard.

## What works today

The crate parses the MPEG-1 Audio **Layer I** frame header, computes
frame geometry, and decodes the Layer I audio data up to requantized
subband samples — all from ISO/IEC 11172-3:

- **32-bit frame header** (§2.4.1.3 / §2.4.2.3): all thirteen fields —
  syncword, `ID`, `layer`, `protection_bit`, `bitrate_index`,
  `sampling_frequency`, `padding_bit`, `private_bit`, `mode`,
  `mode_extension`, `copyright`, `original/copy`, `emphasis` — decoded
  into the typed [`FrameHeader`] struct. The Layer I `bitrate_index`
  ladder and `sampling_frequency` table are read straight from the
  §2.4.2.3 tables; forbidden (`0b1111`) and reserved (`0b11`) values
  are rejected, free format is recognized.
- **Frame sync** (§2.4.3.1): `find_sync` scans a buffer for the first
  position whose four bytes form a valid Layer I header.
- **Frame-length computation** (§2.4.2.1 / §2.4.2.3 / §2.4.3.1): slot
  count `N = floor(12 · bitrate / sampling_frequency) + padding_bit`,
  each Layer I slot being four bytes, yielding total frame bytes. Free
  format returns `None` (length is recovered from syncword distance).
- **CRC `error_check()` wiring** (§2.4.1.4): `protection_bit` drives
  whether a 16-bit CRC word follows the header; `read_crc` returns it
  as a `PresentUnverified` value (see "Spec gaps").
- **Layer I audio-data decode** (§2.4.1.5 / §2.4.2.5 / §2.4.3.2):
  `decode::decode_audio_data` reads the per-subband 4-bit bit
  allocation (mapped to 0/2..15 bits/sample via the §2.4.2.5 table,
  rejecting the invalid `0b1111`), the 6-bit scalefactor index per
  allocated subband, and the per-sample requantization (read `nb`
  bits, invert the MSB, treat as a two's-complement fraction, apply
  the §2.4.3.2 linear formula). It produces the **32 × 12 requantized
  subband samples** per channel ([`SubbandSamples`]) for mono,
  stereo, dual-channel, and joint-stereo — the joint-stereo upper
  band `[bound, 32)` shares one allocation and one sample stream that
  is copied to both channels (intensity_stereo).

38 unit tests cover the whole bitrate ladder, every sampling rate,
every mode / mode_extension / emphasis code, both padding cases at
44.1 kHz, sync recovery, the CRC-presence paths, the §2.4.2.5
allocation→bits table, the MSB-first bit reader, the §2.4.3.2
requantization formula at several widths, and mono / stereo /
joint-stereo audio-data decode. Test bytes are constructed locally
from the §2.4.1 field layouts — no external fixtures.

## Not yet implemented

- The **final rescale by the Annex B Table 3-B.1 scalefactor
  multiplier** (see "Spec gaps") — the decode stops at the §2.4.3.2
  requantized value `s''` and stores the raw 6-bit scalefactor index.
- The **polyphase synthesis filterbank** (§2.4.3.2 "Synthesis subband
  filter", with the Annex B Table 3-B.3 window coefficients), which
  turns the 32 × 12 subband samples into 384 PCM samples.

Because the decode does not yet reach PCM, the crate registers no
decoder into the runtime context. `register` is a no-op until a
`Decoder` lands in a later round.

## Spec gaps (DOCS-GAP)

The staged `ISO_IEC_11172-3-MP3-1993.pdf` is **missing its normative
Tables annex** and several display equations:

1. **3-Annex B (normative) Tables is absent.** The 46-page document
   body ends mid-§2.4.3.4 with no Annex pages; every "3-Annex B"
   reference is a cross-reference, never the table itself. In
   particular **Table 3-B.1 "LAYER I, II SCALEFACTORS"** (the 64-entry
   scalefactor multiplier table) and Table 3-B.3 (synthesis-window
   coefficients) cannot be read. The audio-data decode therefore
   stores the 6-bit scalefactor *index* and exposes the requantized
   `s''`; the final `s' = s'' · scalefactor[index]` rescale and the
   synthesis filterbank are blocked until Annex B is staged.
2. The **§2.4.3.2 requantization linear formula** and the rescale
   formula are rendered as blank image regions in the PDF. The
   requantization formula
   `s'' = (2^nb / (2^nb − 1)) · (s''' + 2^(−nb+1))` was supplied in
   the round brief and is corroborated by the §2.4.3.2 prose
   (invert MSB → two's-complement fraction → linear formula).
3. The **CRC-16 generator polynomial** `G(X)`, plus Annex B
   Table 3-B.5 and Annex A Figure 3-A.9 — CRC remains wired
   structurally but unverified (carried over from the header round).
4. The display form of the Layer I frame-length equation, recovered
   from the §2.4.2.3 padding-method prose.

## License

MIT — see [LICENSE](./LICENSE).
