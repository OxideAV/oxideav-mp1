# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.7](https://github.com/OxideAV/oxideav-mp1/compare/v0.0.6...v0.0.7) - 2026-05-23

### Other

- dual-channel mode (mode=10, §2.4.2.3)
- joint-stereo encode (§2.4.2.3 / §2.4.1.5)
- CRC-16 protection (§2.4.3.1) on encode + decode

### Added

- Dual-channel encode (ISO/IEC 11172-3 §2.4.2.3, `mode = 10`). New
  `dual_channel` (bool) encoder option, gated on `channels == 2` and
  ignored when `joint_stereo` is also set (joint wins, since it
  materially changes the bit allocation while dual-channel only
  relabels the 2-bit `mode` field). The wire layout is identical to
  plain stereo — per-channel allocation, scalefactor, and 12-sample
  payload across all 32 subbands — so the encoder reuses the existing
  CBR/VBR allocators unchanged and emits the same byte count at the
  same CBR slot. The semantic difference is downstream: the two
  channels represent independent programs (e.g. two languages) rather
  than a stereo pair. Validated black-box: ffmpeg's MPEG-audio decoder
  accepts our `mode = 10` streams and recovers each program's tone
  cleanly; the self-roundtrip preserves channel separation (`>= 15 dB`
  inter-channel SNR delta on uncorrelated tones).
- Joint-stereo encode (ISO/IEC 11172-3 §2.4.2.3 / §2.4.1.5). New
  `joint_stereo` (bool) and `js_bound` (4 / 8 / 12 / 16) encoder
  options. When enabled on a stereo input the encoder emits
  `mode = 01` (joint_stereo) with a `mode_extension` selecting the
  `bound`, and shares the upper subbands `[bound..32)` between the two
  channels: one allocation field + one quantised sample stream per
  shared subband, but a scalefactor per channel. The shared stream is
  the per-sample mid `M = (L + R) / 2`, so the decoder reconstructs
  `L = R = M` above the bound (Layer I has no intensity scaling). The
  CBR/VBR allocators are now bound-aware (`upgrade_cost_bits_bound`):
  a shared subband's 0 → non-zero transition pays one scalefactor per
  channel on top of one shared 12-sample stream, and the freed
  upper-band bits are redistributed to the loudest below-bound bands.
  Validated black-box: ffmpeg's MPEG-audio decoder accepts our
  joint-stereo streams (74 dB tone SNR) and the self-roundtrip keeps
  bit-identical L/R for identical input.
- CRC-16 protection (ISO/IEC 11172-3 §2.4.3.1) on both paths. New
  `crc::Crc16` accumulator (polynomial `x^16 + x^15 + x^2 + 1`, init
  `0xFFFF`, MSB-first) and `crc::layer1_crc` helper. The encoder gains a
  `crc_check` option that clears the `protection_bit` and inserts the
  16-bit CRC word after the header, covering the header tail (bytes 2–3)
  plus the entire bit-allocation field (Table 3-B.5). The decoder now
  verifies the CRC word when `protection_bit == 0` and rejects frames
  whose protected field is corrupted (previously the word was skipped).
  Validated by round-trip and black-box: ffmpeg's MPEG-audio decoder
  accepts our CRC streams and reports "CRC mismatch" only on deliberate
  corruption.

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
