# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Layer II `audio_data()` decode** (ISO/IEC 11172-3 (1993) §2.4.1.6
  / §2.4.2.6 / §2.4.3.3 + Annex B Tables 3-B.2a..d "Possible
  quantization per subband" and 3-B.4 "Layer II classes of
  quantization"). The frame-header parser now accepts the `layer ==
  '10'` codepoint (alongside `'11'` Layer I) and selects the correct
  bitrate ladder for either edition (MPEG-1 Layer II:
  32/48/56/64/80/96/112/128/160/192/224/256/320/384 kbit/s; MPEG-2
  LSF Layer II / III shared ladder: 8/16/24/32/40/48/56/64/80/96/
  112/128/144/160). Layer II framing uses 1-byte slots and the
  `slot_count = floor(144 · bitrate / Fs) + padding` formula
  (§2.4.2.1), producing 1152 samples per channel per frame.
  - New `decode_layer2` module + `decode_layer2_audio_data` entry
    point: reads the per-subband nbal-bit allocation (table picked by
    `(Fs, kbps-per-channel)` per the §B.2 per-table headers), the
    per-subband 2-bit `scfsi`, the 1..3 six-bit Table 3-B.1
    scalefactor indices per §2.4.3.3.2 scfsi schedule, and 12
    syntax-granules of either one grouped `samplecode` (degrouped by
    `c % nlevels; c /= nlevels` × 3) or three separable `sample`
    reads. Each triplet's §2.4.3.3.4 linear formula
    `s'' = C · (s''' + D)` (constants from Table 3-B.4) is rescaled
    by the per-part Table 3-B.1 multiplier; 36 sample-slots per
    subband per channel feed the existing §2.4.3.2 polyphase synthesis
    filterbank.
  - Intensity_stereo upper band (`[bound, sblimit)` in joint_stereo)
    shares one allocation and one sample stream copied into both
    channels (§2.4.2.6 / §2.4.3.3).
  - `Mp1Decoder` routes Layer II packets through the new audio_data
    + a 36-slot synthesis loop that emits 1152 samples per channel
    per frame (vs the Layer I path's 384).
  - 4 Tables-3-B.2x allocation tables transcribed verbatim from PDF
    pages 52-55 page renders; 17 Table 3-B.4 quantization classes
    transcribed from PDF page 59 (formulas C = 2^n/(2^n-1),
    D = 2^(-n+1) cross-checked against every non-grouped row).
  - **Fixture-driven integration test**
    (`tests/layer2_mono.rs`): decodes an ffmpeg-encoded mono `.mp2`
    (440 Hz sine, 64 kbit/s, 44.1 kHz, 20 frames) and compares the
    steady-state PCM against ffmpeg's reference S16 decode of the
    same file. **RMS = 0.50 LSB, max|err| = 1 LSB across 20 736
    samples** — essentially bit-exact, the residual being IEEE-754
    multiplication-order differences.
  - New public types/items: `Layer`, `Layer2Subband`, `Layer2Subbands`,
    `LAYER2_SAMPLES_PER_FRAME`, `LAYER2_SAMPLES_PER_SUBBAND`,
    `decode_layer2_audio_data`, `tables_layer2::QuantClass`,
    `tables_layer2::AllocationTable`,
    `tables_layer2::layer2_bit_allocation_table`,
    `tables_layer2::QUANT_CLASSES`.
  - New `FrameHeader::layer` field carries the parsed `Layer::{I, II}`
    value. `HeaderError::NotLayer1` is retained for API stability but
    now only fires on Layer III (`'01'`) and the reserved value
    (`'00'`).
  - 16 new unit tests across `decode_layer2` (all-unallocated mono
    decode through B.2a, requantization endpoints, grouped degroup)
    and `tables_layer2` (B.4 formula cross-check, grouped-class
    bits-per-sample, B.2a/B.2c/B.2d shapes + sum-of-nbal footers,
    quant-class resolution, stereo bitrate-per-channel division).
    3 new integration tests in `tests/layer2_mono.rs`. Total now 126
    tests (108 unit + 16 integration + 2 doc).
  - **Followups**: Layer II encoder; §2.4.3.1 CRC-16 verification
    over the Layer II protected fields (header bits 16…31 + bit
    allocation + scfsi per Table 3-B.5); transcription of the
    13818-3 LSF Layer II allocation table for Fs ∈ {16, 22.05, 24}
    kHz (currently mapped onto the 11172-3 B.2b table as a defensive
    default).

- **Selectable §2.4.3.1 CRC-mismatch concealment** (`ConcealmentMode`),
  derived solely from ISO/IEC 11172-3 (1993) §2.4.3.1, which recommends
  "muting of the actual frame or repetition of the previous frame".
  Both strategies are now implemented and chosen by the caller:
  - `ConcealmentMode::Mute` (default, unchanged behaviour) emits a
    silent frame and rings the filterbank history out with zeros.
  - `ConcealmentMode::RepeatPrevious` re-synthesizes the last
    successfully-decoded frame's requantized subband samples through the
    advancing synthesis filterbank, reproducing that frame's audio
    instead of a silence drop-out. A concealed frame is never stored as
    the new "previous" frame, so a run of corrupt frames repeats the
    *last good* frame each time (no chaining of repeats-of-repeats); a
    corrupt first frame falls back to muting (nothing to repeat).
  - Selection points: `Mp1Decoder::with_concealment` (builder),
    `Mp1Decoder::set_concealment` (runtime), and the new direct-API
    `decoder::make_decoder_with_concealment(&CodecParameters, ConcealmentMode)`.
    `Mp1Decoder::new`, `Mp1Decoder::concealment` and
    `SubbandSamples::silent` are now public. `reset` drops the repeat
    history while preserving the configured mode.
  - 8 new tests (6 in `codec`: default-is-mute + builder, repeat
    reproduces the last good frame's PCM, no chaining, first-frame
    fallback to mute, runtime `set_concealment`, frame-shape preserved,
    reset clears history; 2 in `decoder`: the direct concealment factory
    repeats, the plain factory keeps the Mute default). 109 tests total
    (96 unit + 13 integration; doc-tests unchanged).
- **§2.4.3.1 CRC-16 `error_check()` verification**, derived solely from
  ISO/IEC 11172-3 (1993) §2.4.3.1 + Annex B Table 3-B.5 (the generator
  polynomial recovered from the page-36 typeset equation render). The
  previously-deferred CRC spec gap is now closed:
  - `header::FrameHeader::verify_crc(header_bytes, after_header)`
    computes the CRC-16 (`G(X) = X^16 + X^15 + X^2 + 1`, initial state
    `0xFFFF`) over the Table 3-B.5 protected fields for Layer I — header
    bits 16…31 plus the bit-allocation field — and compares it with the
    stored word, returning `CrcStatus::{Absent, Ok, Mismatch}`.
  - `header::FrameHeader::compute_crc(header_bytes, allocation)` exposes
    the same value an encoder would write (encoder-side CRC emission is
    a followup; the encoder still always sets `protection_bit == 1`).
  - `CrcStatus` replaces the old `PresentUnverified` placeholder with a
    verified `Ok(word)` / `Mismatch { stored, computed }` pair plus an
    `is_good()` helper.
  - The `Mp1Decoder` verifies the CRC of every protected frame and, on
    a mismatch, applies the §2.4.3.1 *muting* concealment: it emits a
    correctly-shaped frame and rings the synthesis filterbank history
    out with zeros (no overlap-add discontinuity).
  - 10 new tests (7 in `header`: polynomial/init constants, known
    short-vector register steps, allocation-field-bit sizing across
    modes, compute→verify round-trip, corruption detection in both
    protected regions, and truncation handling; 3 in `codec`: a
    protected frame decoding on a CRC match, a corrupt frame muted as
    silence, and history ring-out after valid frames). 102 tests total
    (87 unit + 13 integration + 2 doc).
- **Direct factory API endpoints** (`decoder` / `encoder` modules),
  completing the workspace dual-API convention so the crate exposes both
  the registry path and the historical direct API:
  - `decoder::make_decoder(&CodecParameters) -> Result<Box<dyn Decoder>, Error>`
    and `encoder::make_encoder(&CodecParameters) -> Result<Box<dyn Encoder>, Error>`
    are thin public wrappers over the exact `Mp1Decoder` / `Mp1Encoder`
    construction `register_codecs` performs; a downstream caller can now
    do `oxideav_mp1::decoder::make_decoder(&params)` /
    `oxideav_mp1::encoder::make_encoder(&params)` like every other codec
    crate. The `register` / `register_codecs` registry path is
    unchanged.
  - 3 new unit tests: a real mono decode through the boxed decoder, a
    real stereo encode through the boxed encoder, and the
    missing-`sample_rate`/`channels` rejection path. 92 tests total.
- **MPEG-2 LSF (Lower Sampling Frequencies) Layer I support**, derived
  solely from ISO/IEC 13818-3 (1997) §2.4.2.3 (read from the staged
  `ISO_IEC_13818-3-MPEG2-audio-1997.pdf`), extending decode + encode to
  the three additional sampling rates 16 / 22.05 / 24 kHz when the
  header `ID` bit is `0`:
  - `header::Id::Mpeg2Lsf` — the `ID == 0` variant (previously
    `Reserved`). `FrameHeader::is_lsf()` reports it.
  - `FrameHeader::parse` selects the §2.4.2.3 LSF `sampling_frequency`
    table (`0b00 → 22.05`, `0b01 → 24`, `0b10 → 16` kHz) and the LSF
    Layer I `bitrate_index` ladder (32 / 48 / 56 / 64 / 80 / 96 / 112 /
    128 / 144 / 160 / 176 / 192 / 224 / 256 kbit/s) when `ID == 0`,
    keeping the MPEG-1 tables for `ID == 1`. The Layer I slot-count
    formula `N = floor(12·bitrate/Fs) + padding` is unchanged.
  - `encode` writes the `ID` bit per the requested sample rate and uses
    the matching bitrate ladder; the factory accepts the LSF rates and
    defaults to 96 kbit/s mono / 128 kbit/s stereo for LSF inputs.
  - 7 new unit tests (LSF sampling table, reserved-rate rejection, the
    full LSF bitrate ladder, MPEG-1-vs-LSF ladder divergence at index 2,
    and three LSF frame-length cases) plus 5 new integration tests (LSF
    silence + tone round-trips across 16 / 22.05 / 24 kHz and an LSF
    frame-layout / header-round-trip check). 89 tests total.
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
