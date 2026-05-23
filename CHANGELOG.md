# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Clean-room MPEG-1 Audio **Layer I** frame-header foundation, derived
  solely from ISO/IEC 11172-3 (1993):
  - `header::FrameHeader` typed struct with all thirteen §2.4.1.3 /
    §2.4.2.3 header fields, and `FrameHeader::parse` for the 32-bit
    big-endian header word (syncword check, Layer I selector,
    forbidden / reserved value rejection, free-format recognition).
  - The §2.4.2.3 Layer I `bitrate_index` ladder and
    `sampling_frequency` table, read only from the standard.
  - `header::find_sync` frame synchronization (§2.4.3.1) and
    `FrameHeader::frame_length_bytes` / `slot_count` frame-length
    computation (§2.4.2.1 / §2.4.2.3 / §2.4.3.1).
  - Structural wiring for the optional 16-bit CRC `error_check()`
    (§2.4.1.4): `FrameHeader::has_crc` / `read_crc` and `CrcStatus`.
    CRC *verification* is deferred — see "Spec gaps" in the README.
  - Layer I **audio-data decode** up to requantized subband samples
    (§2.4.1.5 / §2.4.2.5 / §2.4.3.2) in the new `decode` module:
    `decode_audio_data` reads the per-subband 4-bit bit allocation
    (`allocation_bits` maps it to 0/2..15 bits/sample, rejecting the
    invalid `0b1111`), the 6-bit scalefactor index per allocated
    subband, and the per-sample `requantize` (read `nb` bits, invert
    the MSB, two's-complement fraction, §2.4.3.2 linear formula). It
    produces the 32 × 12 requantized subband samples per channel
    (`SubbandSamples` / `Subband`) for mono, stereo, dual-channel,
    and joint-stereo (upper-band intensity_stereo sharing). A
    MSB-first `BitReader` and a `DecodeError` type are also added.
    The final rescale by the Annex B Table 3-B.1 scalefactor
    multiplier is deferred — that table is absent from the staged
    ISO PDF (see "Spec gaps" in the README).
  - 16 further unit tests (38 total) built from locally-constructed
    bitstream bytes — allocation table, bit reader, requantization
    at several widths, and mono / stereo / joint-stereo decode.

### Erased

- Prior master history was force-erased on **2026-05-24** under
  Hat-3 cold enforcement of the workspace clean-room policy
  (`docs/IMPLEMENTOR_ROUND.md`). The retired implementation's
  synthesis-window data table carried a module doc-comment stating
  the values were transcribed from an external library's source
  file rather than read solely from the ISO/IEC specification.

### Reset

- Crate reduced to a minimal `oxideav_core::register!` stub. Every
  public API returns `Error::NotImplemented`. The crates.io version
  (`0.0.6`) is preserved on the new master to avoid breaking
  downstream version pins; the published versions on crates.io will
  be yanked by the maintainer.

### Next

- Clean-room re-implementation against the staged ISO/IEC 11172-3
  Layer I specification (numeric tables read only from the standard)
  in a future round.
