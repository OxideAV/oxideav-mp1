# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- VBR (variable bit-rate) encoder mode driven by a per-subband
  masking model (`psy::SubbandMask`). Two-phase allocator: mask-driven
  upgrades until every band is masked, then quality-scaled target-fill
  bounded by a rolling-average controller against `vbr_target_kbps`.
  Per-frame `bitrate_index` floats over the standard Layer I ladder;
  the long-term average converges on the user-supplied target.
  Selected via `vbr_target_kbps` and/or `vbr_quality` (0..=9) on
  `CodecParameters::options`. ffmpeg cross-decode interop verified.

## [0.0.4](https://github.com/OxideAV/oxideav-mp1/compare/v0.0.3...v0.0.4) - 2026-04-25

### Other

- drop oxideav-codec/oxideav-container shims, import from oxideav-core
- drop Cargo.lock — this crate is a library
- bump oxideav-core / oxideav-codec dep examples to "0.1"
- bump to oxideav-core 0.1.1 + codec 0.1.1
- migrate register() to CodecInfo builder
- bump oxideav-core + oxideav-codec deps to "0.1"
- bump oxideav-core to 0.0.5
- migrate to oxideav_core::bits shared BitReader / BitWriter
- stereo + multi-bitrate encode roundtrip, accurate feature matrix
- add 'Quick use' example for standalone decode/encode
- loosen oxideav-* pins to '0.0' (accept any 0.0.x)
