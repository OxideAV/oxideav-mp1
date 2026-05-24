# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Layer I decode to PCM complete**, derived solely from ISO/IEC
  11172-3 (1993) Annex B (now staged in the 157-page PDF):
  - `tables::SCALEFACTORS` — the 63 Table 3-B.1 "LAYER I, II
    SCALEFACTORS" multipliers (PDF page 51), transcribed and
    cross-checked against the closed-form `2^((3−i)/3)`.
  - `tables::SYNTHESIS_WINDOW` — the 512 Table 3-B.3 "COEFFICIENTS Di
    OF THE SYNTHESIS WINDOW" taps (PDF pages 56–58), transcribed
    verbatim from the table renders.
  - `Subband::rescaled_samples` / `SubbandSamples::slot` apply the
    §2.4.3.2 rescale `s' = scalefactor[index] · s''`.
  - `synthesis::SynthesisFilter` — the §2.4.3.2 polyphase synthesis
    subband filter (3-Annex A Figure 3-A.2): per-channel 1024-element
    `V` FIFO with cross-frame overlap-add, 32→64 matrixing
    (`N_ik = cos[(16+i)(2k+1)π/64]`), the 512-tap `D[]` windowing, and
    the 16-term summation to 32 PCM samples per slot. `to_s16`
    converts the reconstruction to clamped S16.
  - `codec::Mp1Decoder` implements `oxideav_core::Decoder`:
    packet → 384-sample-per-channel interleaved S16 `AudioFrame`,
    with `reset` zeroing the filterbank history. `register` installs
    it under WAVE format tag `0x0050` and Matroska id `A_MPEG/L1`.
  - 17 further unit tests (55 total): scalefactor table spot-checks +
    formula + Table 3-B.3 quantization/symmetry, matrixing
    coefficients, a window overlap-add test, and full frame→PCM smoke
    tests (mono + stereo + reset).
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

### Next

- A Layer I **encoder** (PCM → bitstream).
- CRC-16 verification, once the generator polynomial / Table 3-B.5 are
  recoverable from the staged spec (see "Spec gaps" in the README).
