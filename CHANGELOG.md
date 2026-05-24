# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Layer I encoder (PCM → bitstream)**, derived solely from ISO/IEC
  11172-3 (1993) informative Annex C plus the normative Layer I clauses:
  - `tables::ANALYSIS_WINDOW` — the 512 Table C.1 "Coefficients Ci of
    the Analysis Window" taps (Annex C, PDF pages 68–69), transcribed
    verbatim. A unit test cross-checks magnitudes against
    `SYNTHESIS_WINDOW / 32` (the prototype-filter relationship the two
    tables share) so a transcription slip in either table is caught.
  - `tables::QUANT_A`, `tables::QUANT_B` — the 14 Table C.3 "Layer I
    Quantization Coefficients" (Annex C, PDF page 72), indexed by `nb`.
    Values cross-checked against the closed forms
    `A = (2^nb − 1)/2^nb`, `B = −1/2^nb`.
  - `tables::SNR_DB` — the 16 Table C.2 "Layer I Signal-to-Noise
    Ratios" (Annex C, PDF page 72), indexed by `nb`.
  - `encode::AnalysisFilter` — the §C.1.3 polyphase analysis filterbank
    (figure C.4): per-channel 512-element `X` FIFO with cross-frame
    history, the §C.1.3 windowing by `C[]`, the partial sum
    `Y[i] = Σ_{j=0..7} Z[i+64j]`, and the matrixing
    `S[i] = Σ_{k=0..63} M[i][k]·Y[k]` with
    `M[i][k] = cos[(2i+1)(k-16)π/64]`.
  - `encode::select_scalefactor` — the §C.1.5.1.4 pick: the lowest
    Table 3-B.1 value larger than the max-absolute subband sample.
  - `encode::quantize` — the §C.1.5.1.7 linear quantizer (Table C.3 `A,
    B`, scaled `nb`-bit two's-complement, MSB-inverted). Exact inverse
    of the decoder's `requantize`, verified bit-exact per (nb, code).
  - `encode::allocate_bits` — the §C.1.5.1.6 iterative allocator. The
    clean-room encoder uses a non-psychoacoustic signal-energy SMR
    proxy (Annex D models intentionally not implemented), and respects
    the `adb` budget including the +6 scalefactor cost on the 0→2 jump.
  - `encode::Mp1FrameEncoder` — frame-level encoder that ties analysis
    + scalefactor pick + allocation + quantization + frame assembly
    (HEADER, ALLOC, SCALEFACTORS, SAMPLES, ANC, MSB-first per figure
    C.2) into one Layer I packet.
  - `codec::Mp1Encoder` implements `oxideav_core::Encoder`:
    interleaved-S16 `AudioFrame` (384 samples/channel) → Layer I
    packet. Factory takes `sample_rate`, `channels` and optional
    `bit_rate` (192 kbit/s mono, 256 stereo default). `register_codecs`
    installs encoder alongside decoder.
  - 14 further unit tests (69 total): analysis-window endpoints +
    2^-21 quantization + magnitude cross-check, Table C.3 closed-form
    A/B, Table C.2 SNR monotonicity + spot values, analysis-matrix
    spot values, `quantize ∘ requantize` identity for every (nb, code),
    `select_scalefactor` lowest-larger-value semantics, allocator
    behaviour at tight / zero budgets, and an analysis+synthesis
    reconstruction test that proves the Table C.1 signs are correct
    end-to-end (RMS < 1e-3 against the original sine).
  - 8 new integration tests in `tests/roundtrip.rs`: silence, mono and
    stereo tones, white noise, frame layout, output-params, registry
    round-trip — driving the full
    `CodecParameters → first_encoder → send_frame → receive_packet →
    first_decoder → send_packet → receive_frame` path that a real
    container would take.
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
