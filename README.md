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

The first foundation layer parses the MPEG-1 Audio **Layer I** frame
header and computes frame geometry, all from ISO/IEC 11172-3:

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

22 unit tests cover the whole bitrate ladder, every sampling rate,
every mode / mode_extension / emphasis code, both padding cases at
44.1 kHz, sync recovery, and the CRC-presence paths. Test bytes are
constructed locally from the §2.4.1.3 field layout — no external
fixtures.

## Not yet implemented

Audio-data decode (bit allocation, scalefactors, requantization,
subband synthesis — §2.4.2.5 / §2.4.3.2) is **not** wired up, so the
crate registers no decoder into the runtime context yet. `register` is
a no-op until a `Decoder` lands in a later round.

## Spec gaps (DOCS-GAP)

In the staged `ISO_IEC_11172-3-MP3-1993.pdf`, two §2.4.3.1 display
equations are rendered as missing glyphs and could not be read:

1. The **CRC-16 generator polynomial** `G(X)`, plus Annex B
   Table 3-B.5 (the set of header/side-info bits fed into the check)
   and Annex A Figure 3-A.9 (the shift-register diagram). The CRC is
   therefore wired structurally (presence/absence from `protection_bit`,
   the stored word read MSB-first) but **not yet verified**. The
   polynomial must be supplied from the spec before verification can be
   implemented clean-room.
2. The display form of the Layer I frame-length equation. Its content
   was recoverable from the §2.4.2.3 padding-method prose
   (`dif = (12 · bitrate) % sampling_frequency`) plus the §2.4.2.1 slot
   definition, so frame-length computation is complete despite the
   blank display line.

## License

MIT — see [LICENSE](./LICENSE).
