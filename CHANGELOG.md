# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.6](https://github.com/OxideAV/oxideav-mp1/compare/v0.0.5...v0.0.6) - 2026-05-06

### Other

- drop dead `linkme` dep
- auto-register via oxideav_core::register! macro (linkme distributed slice)
- unify entry point on register(&mut RuntimeContext) ([#502](https://github.com/OxideAV/oxideav-mp1/pull/502))
- replace never-match regex with semver_check = false

### Changed

- **`register` entry point unified on `RuntimeContext`** (task #502).
  The legacy `pub fn register(reg: &mut CodecRegistry)` is renamed to
  `register_codecs` and a new `pub fn register(ctx: &mut
  oxideav_core::RuntimeContext)` calls it internally. Breaking change
  for direct callers passing a `CodecRegistry`; switch to either the
  new `RuntimeContext` entry or the explicit `register_codecs` name.

## [0.0.5](https://github.com/OxideAV/oxideav-mp1/compare/v0.0.4...v0.0.5) - 2026-05-02

### Other

- migrate to centralized OxideAV/.github reusable workflows
- add VBR mode driven by per-subband masking
- adopt slim VideoFrame/AudioFrame shape
- pin release-plz to patch-only bumps

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
