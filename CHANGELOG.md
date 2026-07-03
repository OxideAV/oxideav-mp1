# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **Layer II intensity_stereo encode quality — shared upper-band stream
  is now the per-slot channel average `(L+R)/2`.** Per §2.4.1.6 / Annex
  B ("Requantization of subband samples"), the joint_stereo upper band
  `[bound, sblimit)` codes ONE shared samplecode stream the decoder
  rescales into both channels with each channel's own Table 3-B.1
  scalefactor. The encoder previously sourced that shared stream from
  channel 0 alone, discarding the right channel's signal shape. It now
  forms the stream from the per-slot channel average `(L+R)/2`, so both
  channels' contributions survive into the coded samplecode while the
  per-channel scalefactors restore each channel's loudness (intensity
  positioning by level). Channel 0's upper-band scalefactor is now
  selected from the combined signal's per-part peak — a peak the average
  can never exceed (`|(L+R)/2| ≤ max(|L|,|R|)`), guaranteeing the shared
  samplecode never overflows quantization; channel 1 keeps the
  scalefactor selected from its own per-part peak. The new conformance
  guard `encode_layer2_joint_stereo_shared_stream_is_channel_average`
  drives two distinct comparable-amplitude waveforms through a shared
  subband and asserts the decoded shared stream tracks `(L+R)/2` closer
  than channel 0 alone (the test fails on the prior channel-0-only
  combine).

### Added

- **Model 1 alternative Step 2 SPL selectable on the driver.**
  `Model1::with_alternative_spl(true)` switches the Step 2 spectral
  term from the per-line maximum to the spec's alternative power sum
  `X_spl(n) = 10·log10(Σ 10^(X(k)/10))` (offered in clause D.1 as "a
  potential for better encoder performance", untested formally per the
  spec's own note); `uses_alternative_spl()` reports the selection and
  the default remains the primary MAX form. **+1 lib-test**: with
  identical thresholds the alternative never lowers any subband's SMR
  (power sum ≥ max) and strictly raises a two-partial subband's by the
  expected ≈ 3 dB.
- **Model 1 end-to-end conformance integration tests**
  (`tests/model1_psychoacoustic.rs`). Four public-surface contracts
  for the clause D.1 encode paths: (1) a Model 1 Layer I **stereo**
  multi-frame encode → decode loop with Goertzel single-bin
  discrimination (each decoded channel dominated by its own tone by
  ≥ 100×); (2) a Model 1 Layer II mono round-trip at 48 kHz where the
  decoded spectrum stays tone-dominated by ≥ 1000× and the tone level
  survives within ±3 dB; (3) byte-exact determinism of two identical
  Model 1 encoders across frames on both layers; (4) a
  model-has-effect guard — on a three-tone signal the Model 1 frames
  differ from the energy-proxy frames inside the identical §2.4.2.1
  envelope, and both streams decode cleanly.
- **Model 1 psychoacoustic allocation wired into the Layer II
  encoder (+ VBR / top-level composition).**
  `Mp1Layer2FrameEncoder::with_psy_model(PsyModel)` selects the Annex
  D model behind `with_psychoacoustic(true)` — order-independent
  (selecting after enabling rebuilds the driver) — and
  `active_psy_model()` reports the running model. Under Model 1 the
  Layer II encoder feeds the shared 1024-sample sliding window through
  `model1::Model1` (Layer II's Model 1 FFT length equals Model 2's),
  derives the Step 2 `scf_max(n)` from the maximum of the three
  Table 3-B.1 multipliers the frame will be coded with, and keys the
  Step 3 offset on the per-channel bitrate. Composes with per-frame
  VBR (the rung selection reuses the Model 1 SMR) and with the
  §2.4.1.8 ancillary path unchanged. The top-level `Mp1Encoder`
  forwards `EncodeParams::psy_model` to the inner Layer II encoder, so
  the registry-facing trait object honours the model switch for both
  layers. LSF half-rates fall back to the proxy allocator
  byte-identically. **+4 lib-tests**: a Model 1 Layer II
  encode → decode round-trip (exact §2.4.2.1 length, ≥ 1 allocation,
  builder order-independence), the byte-identical LSF fallback, the
  VBR composition (near-silence never selects a higher rung than loud
  wide-band content; every header a real ladder rung), and a top-level
  `Mp1Encoder` Model 1 Layer II PCM round-trip.
- **Model 1 psychoacoustic allocation wired into the Layer I
  encoder — selectable Annex D model (`PsyModel`).** New
  `EncodeParams::psy_model` field / `with_psy_model(PsyModel)` builder
  selects which Annex D example model drives psychoacoustic allocation
  when `with_psychoacoustic(true)` is set: `PsyModel::Model1` (clause
  D.1) or the default `PsyModel::Model2` (clause D.2, byte-identical
  to the pre-switch behaviour). Under Model 1 the `Mp1FrameEncoder`
  maintains a per-channel **512-sample** sliding window (Model 1's
  Layer I FFT length, vs Model 2's 1024), feeds the Step 2
  scalefactor term from the frame's actually-selected Table 3-B.1
  multipliers, keys the Step 3 LTq offset on the per-channel bitrate,
  and allocates via the same `allocate_bits_psy` MNR loop.
  `Mp1FrameEncoder::active_psy_model()` reports which model runs;
  the LSF half-rates fall back to the energy proxy byte-identically
  (no Annex D tables). **+5 lib-tests**: the default/builder/inert
  selection matrix, a Model 1 encode → decode round-trip asserting the
  tone subband is coded while silent top subbands draw no bits, the
  byte-identical LSF fallback, the Model 1 / Model 2 same-frame-
  envelope check (identical slot count + header), and stereo
  reset-restores-fresh-state.
- **Annex D Model 1 per-frame driver — Steps 5–9 complete + the
  assembled `Model1` driver.** The clause D.1 chain now runs end to
  end. `decimate_maskers` performs Step 5: components below the
  threshold in quiet at their frequency (`X >= LTq(k)`, Table D.1x
  value + Step 3 bit-rate offset) are removed, components above the
  Table D.1x coverage (possible for tonal lines between the table top
  and the Step 4 examinable limit) are removed, and a 0,5-Bark sliding
  window keeps only the strongest of any run of close tonal
  components; each survivor is assigned its nearest Table D.1x row
  (the Step 6 "index i that most closely corresponds to the frequency"
  rule) as a typed `Masker`. `global_thresholds_db` evaluates Steps
  6+7 — the individual-threshold forms with the −8…+3 Bark reduction,
  power-summed with LTq — at every subsampled Table D.1x line (the new
  `psy::ltq_table(layer, fs)` accessor dispatches all six D.1a–f
  tables). `min_threshold_per_subband` is Step 8 (`LTmin(n) = MIN
  LTg(i), f(i) in subband n`; top subbands beyond the table coverage
  adopt the edge line's threshold), and `smr_per_subband` Step 9
  (`SMR_sb(n) = Lsb(n) − LTmin(n)`). The **`Model1`** struct ties
  Steps 1–9 into one `process(window, scf_max, kbps_per_channel)`
  call — stateless (Model 1 has no inter-frame prediction history,
  unlike Model 2), `new(layer, fs)` returning `None` for the LSF
  rates. **+16 lib-tests**: the six-table `ltq_table` dispatch,
  audible-kept / sub-threshold-dropped decimation, the Step 3 offset
  moving the decimation gate at the 96 kbit/s boundary, the
  above-coverage drop (line 245 vs 240 at 32 kHz), the 0,5-Bark
  window keeping the strongest in both orders plus the sliding-chain
  collapse, the no-masker LTg == LTq_used floor and its exact −12 dB
  offset shift, the loud-masker locality (raised within ±0,25 Bark,
  untouched outside the −3…+8 window), a brute-force Step 8
  cross-check, LSF/length rejections, silence → all-−∞ SMR,
  tone-demands-bits-with-self-masking, masking lowering a neighbour
  probe's SMR by 10+ dB, a Layer II 1024-point end-to-end run, and the
  wrong-window-length panic contract.
- **Annex D Model 1 per-frame driver — Step 4 (tonal / non-tonal
  component identification).** `find_tonal_components` labels the
  clause D.1 Step 4 a) local maxima (`X(k) > X(k−1)` and
  `X(k) >= X(k+1)`), applies the Step 4 b) 7 dB criterion over the
  per-layer / per-region examined-neighbour sets (`tonal_search_offsets`:
  `±2` for `2 < k < 63`, `±2…±3` for `63 <= k < 127`, `±2…±6` up to
  k = 250 / 254 for Layer I / II, and the Layer-II-only `±2…±12` for
  `255 <= k <= 500`), forms the three-line power sum `X_tm(k)`, and
  erases each extracted component's examined frequency range to `−∞`
  in the residual spectrum. `find_non_tonal_components` then sums each
  Table D.2x critical band's remaining line powers into one `X_nm(k)`
  placed at the line nearest the band's geometric mean (Step 4 c),
  rejecting the MPEG-2 LSF rates (no Table D.2x). A documented
  numerical **squelch floor** (`SPECTRUM_SQUELCH_DB = −120 dB`, 30 dB
  below 16-bit quantization silence and 100+ dB below every Annex D
  threshold) maps double-precision FFT round-off crumbs to `−∞` so
  they cannot register as spurious local maxima — restoring the
  exact-arithmetic picture in which a bin-centred sinusoid excites
  exactly three Hann lines. The Step 4 c) band ranges absorb the D.2x
  print truncation (tops printed to three decimals, e.g. 258,398 Hz
  for the exact 3·Fs/512 at 44,1 kHz) with a 1e-3-line epsilon.
  **+9 lib-tests**: the four offset-region boundaries per layer
  (including the ±1-never-examined and symmetry invariants), the
  single-sine one-component extraction with the ≈97,76 dB `X_tm`
  three-line sum and residual erasure, two-distant-sines, the
  adjacent-equal-partials 7 dB rejection, wrong-length rejection,
  the every-band-covered-once tiling + total-power-conservation
  property at all six (layer, rate) pairs, the D.2a band-8 geometric
  mean placement (`sqrt(16·18) → 17`), tonal-extraction leaving the
  tone's own band non-tonal-empty, and LSF-rate rejection.
- **Annex D Model 1 per-frame driver — Steps 1 + 2 (clause D.1
  spectral analysis + per-subband SPL).** New `model1` module opening
  the second Annex D example model: `power_spectrum` runs the printed
  Step 1 Hann window `h(i) = √(8/3)·0,5·(1 − cos(2πi/N))` and
  `X(k) = 10·log10 |(1/N)·Σ h(l)s(l)e^(−j2πkl/N)|²` at the per-layer
  transform length (512-point for Layer I, 1024-point for Layer II —
  Model 1 is the model whose FFT length differs by layer), normalized
  to the 96 dB SPL reference via a fixed full-scale calibration
  (`calibration_offset_db`, documented against the Step 2 absolute
  scalefactor term and the Table D.4 ±32760-sine anchor).
  `spl_per_subband` forms the Step 2 sound pressure level
  `Lsb(n) = MAX[X(k) in subband n, 20·log10(scf_max·32768) − 10]`,
  and `spl_per_subband_alt` the spec's alternative power-sum variant
  `X_spl(n)`. **+13 lib-tests**: the per-layer FFT lengths / line
  counts, full-scale-sine 96 dB peak at both lengths, the −6,02 dB
  half-amplitude check, Hann-skirt / far-leakage shape, silence → −∞,
  the calibration closed form, wrong-length rejection, the Step 2
  MAX semantics (tone vs scalefactor floor), the silence
  scalefactor-term exactness, the alt-variant ≥ max-variant bound,
  and the Layer I / Layer II grid correspondence.
- **Layer II automatic intensity_stereo bound selection.** New
  `select_layer2_intensity_bound` chooses the §2.4.2.3 `mode_extension`
  (intensity bound `{4, 8, 12, 16}`) from a frame's per-subband
  two-channel signal: it finds the deepest subband whose normalised
  inter-channel difference energy exceeds a relative threshold (the
  deepest band that must stay in full stereo) and returns the smallest
  bound covering it. A near-mono frame collapses to the smallest bound
  (share aggressively, save bits); a wide-stereo frame keeps more
  subbands in full stereo (preserve the stereo image). The stateful
  `Mp1Layer2FrameEncoder::with_auto_intensity_bound(rel_threshold)`
  recomputes `mode_extension` per frame so the bound tracks the content;
  it composes with psychoacoustic, VBR, and ancillary. Four tests cover
  the selector (mono → smallest, wide high-band → largest, threshold
  monotonicity) and the stateful per-frame tracking.
- **Layer II psychoacoustic + ancillary-data in one frame.** New
  `encode_layer2_frame_psy_with_ancillary` assembles a frame with BOTH
  the Annex D Model 2 perceptual allocation AND a §2.4.1.8
  `ancillary_data()` tail (the two concerns are independent — SMR drives
  allocation, ancillary fills the leftover tail). The stateful
  `Mp1Layer2FrameEncoder` now routes its psychoacoustic + pending-
  ancillary case through this entry point (and composes with VBR) instead
  of dropping back to the signal-energy proxy allocator — closing the
  prior "ancillary + psychoacoustic are not yet a single entry point"
  limitation. Two new tests: the combined entry point applies both, and
  the stateful encoder's psy+ancillary path preserves the SMR-driven
  allocation while emitting the payload.
- **Top-level `Mp1Encoder` Layer II now honours psychoacoustic + VBR.**
  The registry-facing trait-object encode path previously ignored
  `EncodeParams::psychoacoustic` for Layer II output (it was documented
  as Layer-I-only); it now forwards both `psychoacoustic` and the new
  `EncodeParams::vbr_target_mnr_db` (set via `EncodeParams::with_vbr`) to
  the inner `Mp1Layer2FrameEncoder`, so `with_layer(LayerSelect::LayerII)
  .with_psychoacoustic(true).with_vbr(t)` drives Model 2 allocation and
  per-frame variable bitrate through the same `Mp1Encoder` the registry
  builds. A new round-trip test exercises the full PCM → encode → decode
  loop on that path.
- **Layer II per-frame variable-bitrate (VBR) encoding.**
  `Mp1Layer2FrameEncoder::with_vbr(target_mnr_db)` selects, for every
  frame, the smallest §2.4.2.3 bitrate-ladder rung whose Annex D Model 2
  allocation pushes every non-masked subband's mask-to-noise ratio to at
  least `target_mnr_db` dB (falling back to the highest rung when no rung
  suffices). The header's `bitrate_index` therefore varies frame-to-frame
  — the decoder re-reads it per frame, so the stream stays
  spec-conformant. VBR takes effect only together with
  `with_psychoacoustic(true)` (it needs Model 2 SMR to make a decision);
  with the model off it is inert and the fixed `params.bitrate_kbps` is
  used. New public building blocks: `select_layer2_vbr_bitrate` (the
  per-frame rung selector), `layer2_min_mnr_db` (the post-allocation
  perceptual quality floor), and `layer2_bitrate_ladder` (the ascending
  §2.4.2.3 ladder for an `ID` bit). `target_mnr_db == 0.0` targets
  transparency by the model; positive values trade bits for headroom,
  negative values trade audible noise for a smaller stream.
- **Layer II joint_stereo PCM round-trip integration tests**
  (`tests/layer2_joint_stereo.rs`). Two distinct per-channel tones are
  encoded through `Mp1Layer2FrameEncoder` in `Mode::JointStereo` and
  decoded with `Mp1Decoder`, asserting per-channel energy ratios and
  Goertzel single-bin spectral discrimination (the left output is
  dominated by its own tone, the right output by its own). A second
  test puts the audible content in the **shared upper band** (≈14 / 15
  kHz tones above the intensity bound) and asserts the right channel's
  reconstructed energy survives — a guard that fails outright (right
  energy → 0) under the prior channel-0-only intensity combine.
- **Annex D Model 2 psychoacoustic bit allocation wired into the
  Layer II encoder.** `allocate_bits_layer2_psy` runs the §C.1.5.2.7
  iterative allocator on the Annex D Model 2 signal-to-mask ratios
  (`MNR = SNR(nlevels) − SMR`) instead of the signal-energy proxy: fully
  masked subbands (`SMR == −∞`) never draw bits, an infinitely-audible
  subband (`SMR == +∞`) is always most urgent, and the most perceptually
  exposed subbands are served first. The new top-level entry point
  `encode_layer2_frame_psy(&params, &subbands, &smr)` mirrors
  `encode_layer2_frame` but drives allocation from caller-supplied SMR
  (the intensity-stereo upper band's SMR is pre-mirrored so the §2.4.1.6
  shared-allocation invariant holds). `Mp1Layer2FrameEncoder::with_psychoacoustic(true)`
  opts the stateful Layer II encoder into the model: it maintains a
  per-channel 1024-sample sliding FFT window, runs the per-channel
  `Model2State` each 1152-sample frame, and allocates against the
  resulting SMR via `encode_layer2_frame_psy`. Honoured at 32 / 44.1 /
  48 kHz; the MPEG-2 LSF half-rates (no Annex D table) silently fall back
  to the proxy allocator (byte-identical to the default encoder).
  `Mp1Layer2FrameEncoder::is_psychoacoustic` reports whether the model is
  active. When an ancillary payload is staged the proxy allocator is used
  for that frame (the Model 2 history is still advanced). **+9 lib-tests**
  cover the allocator (budget compliance, masked-subband skipping,
  highest-SMR-first preference, legal-cell-only selection, zero-budget)
  plus the wired encoder (psy frame round-trip with masked subbands left
  unallocated, end-to-end `with_psychoacoustic` encode, LSF-rate
  byte-identical fallback, frame-envelope parity, and a reset-clears-Model
  2-history check). Public surface adds `allocate_bits_layer2_psy`,
  `encode_layer2_frame_psy`, `Mp1Layer2FrameEncoder::with_psychoacoustic`
  / `is_psychoacoustic`. Closes the README "Not yet supported" Layer II
  Model 2 frontier item. Total `cargo test -p oxideav-mp1 --lib` count:
  **460 → 470**.

- **Annex D Tables D.1d–f — complete Layer II threshold-in-quiet (LTq)
  tables (32 / 44,1 / 48 kHz).** The last untranscribed Annex D table
  family is now in tree: the 132 / 130 / 126 absolute-threshold rows
  from `docs/audio/mp3/annex-d-table-D1{d,e,f}-threshold-*-LayerII.csv`
  are carried in `LTQ_L2_32K` / `LTQ_L2_44K1` / `LTQ_L2_48K` as the same
  four-column `LtqRow` (1-based index, FFT-line frequency, critical-band
  rate in Bark, pre-offset `ltq_db`) the Layer I D.1a–c family uses. The
  Layer II tables sit on the **finer** 1024-point FFT-line grid: the
  D.1d head row is 31,25 Hz (half the D.1a 62,50 Hz step), so the printed
  tables run longer than their Layer I twins (132/130/126 vs 108/106/102).
  Direct slot accessors `ltq_layer2_32k` / `ltq_layer2_44k1` /
  `ltq_layer2_48k` (`None` for `i == 0` and above each table's printed
  end) and the Step 3 composers `ltq_layer2_*_used(i, kbps)` mirror the
  Layer I pairs. Carries the as-printed values including the documented
  minima (`-4.97 dB` at i = 75 for D.1d, `-4.98 dB` at i = 63 / 59 for
  D.1e/D.1f) and the 68,00 dB ceiling saturation (from i = 119 in D.1e,
  i = 115 in D.1f; D.1d's 16 kHz Nyquist never reaches it). **+12
  lib-tests** cover row counts and the 132 > 130 > 126 ordering, dense
  1-based numbering, strictly-monotonic freq/Bark across all three,
  verbatim head + final rows for each table, the documented minima and
  ceilings, the 31,25 Hz head-grid spacing, the lookup hit/miss sets,
  Step 3 offset composition with `None` propagation past each table end,
  and that the Layer II row 1 sits at half the Layer I row-1 frequency
  with the row-2/row-1 LTq matching across the families. This closes the
  last "staged-but-untranscribed LTq family" note — the **entire Annex D
  table family is now transcribed**. Total `cargo test -p oxideav-mp1
  --lib` count: **448 → 460**.

- **Model 2 psychoacoustic bit allocation wired into the Layer I
  encoder.** `allocate_bits_psy` runs the §C.1.5.1.6 iterative allocator
  on the Annex D Model 2 signal-to-mask ratios (`MNR = SNR(nb) − SMR`)
  instead of the signal-energy proxy: fully masked subbands
  (`SMR == −∞`) never draw bits, and the most perceptually exposed
  subbands are served first. `EncodeParams::with_psychoacoustic(true)`
  opts the Layer I `Mp1FrameEncoder` into the model — the encoder
  maintains a per-channel 1024-sample sliding FFT window, runs the
  per-channel `Model2State` each frame, and allocates against the
  resulting SMR. Honoured at 32 / 44.1 / 48 kHz; the MPEG-2 LSF
  half-rates (no Annex D table) silently fall back to `allocate_bits`.
  `Mp1FrameEncoder::is_psychoacoustic` reports whether the model is
  active. 7 new tests: budget compliance, masked-subband skipping,
  highest-SMR-first preference, an end-to-end psychoacoustic encode that
  round-trips to a parseable Layer I frame, LSF-rate fallback, the
  builder flag, and frame-size parity between the two allocators. The
  stale lib-level "Annex D is a DOCS-GAP" note is replaced — every Model
  2 table is now in tree and the model is wired.

- **Annex D Model 2 per-frame driver — SMR computation + streaming
  state (clause D.2 step n).** `smr_per_subband` maps the per-line
  energies / thresholds onto the 32 coder partitions of Table D.5 and
  forms `SMR_n = 10·log10(epart_n / npart_n)`, where the noise term
  `npart_n` sums the per-line thresholds for a psychoacoustically narrow
  band (`width_n == 1`) but takes the minimum line threshold × the band
  width for a wide band (`width_n == 0`) so one quiet line can't drop a
  wide band's floor. `Model2State` ties the whole clause-D.2.4 chain
  together with the two-block prediction history the step-c prediction
  needs across successive frames: `Model2State::process` runs one
  1024-sample analysis window (FFT → unpredictability → partition
  energy/tonality → line thresholds → SMR) and slides the history.
  `Model2State::new` returns `None` for the MPEG-2 LSF half-rates (no
  Annex D Model 2 table). 9 new tests: silent-subband `-∞` SMR, the
  narrow-band threshold-sum and wide-band min×width `npart` formulas,
  LSF-rate rejection, finite positive SMR for a tone, the
  level-invariance of SMR above the absolute-threshold floor and the LTq
  cap below it, and that a steady tone's SMR does not fall as the
  prediction history fills.

- **Annex D Model 2 per-frame driver — spectral-analysis core (clause
  D.2.4 a/b).** New `model2` module assembling the static `psy` tables
  into the iterative per-frame procedure. This step lands the 1024-point
  Hann analysis window (`hann_window`, the `√(8/3)` amplitude
  correction), a textbook radix-2 decimation-in-time FFT
  (`fft_in_place`), and the polar `(r_ω, f_ω)` spectrum
  (`Spectrum::analyze`) for the 513 usable FFT lines (DC..Nyquist).
  `Model2Rate` resolves a sampling frequency to its D.3/D.4 table set
  (`bmax` 49 / 57 / 58 at 32 / 44,1 / 48 kHz) and exposes the
  calculation-partition and per-line absolute-threshold accessors the
  later steps consume. Tests cross-check the FFT against a naïve DFT to
  1e-9, verify the Hann window energy correction (windowed mean-square
  gain ≈ 1), the spreading-function self-spread peak, and the
  contiguous calc-partition tiling to the Nyquist line. Formulas read
  verbatim from ISO/IEC 11172-3 (1993) Annex D clause D.2.3 / D.2.4
  (PDF pages 135–138, printed 129–132).

- **Annex D Tables D.4a–c — complete Model 2 per-FFT-line
  absolute-threshold tables (32 / 44,1 / 48 kHz).** The last untranscribed
  Model 2 Annex D table family is now in tree: the 132 / 130 / 126
  line-range rows from
  `docs/audio/mp3/annex-d-table-D4{a,b,c}-absolute-threshold-*.csv` are
  carried in `ABSTHR_D4A_32K` / `ABSTHR_D4B_44K1` / `ABSTHR_D4C_48K` as
  `AbsThrRange { line_low, line_high, absthr_db }` rows (each row maps a
  contiguous range of 1024-point-FFT lines to a single absolute
  threshold). A binary-search per-line lookup — `absthr_for_line_32k`,
  `absthr_for_line_44k1`, `absthr_for_line_48k` — resolves the threshold
  for any covered FFT line in O(log n), `None` outside. The consts carry
  the **as-printed D.4** values, including the documented divergences
  from the D.1 Layer II twins: D.4a's 51,03 dB final range, D.4b's
  distinctive **69,13 dB** ceiling (vs the Model-1 68,00 dB), and D.4c's
  68,00 dB match. Tests verify row counts, contiguous FFT-line tiling
  from line 1, verbatim head rows, the documented ceilings/minima, and
  that the binary-search lookup agrees with a brute-force scan over
  **every** covered FFT line at all three rates.

- **Annex D Tables D.3b / D.3c — complete Model 2 calculation-partition
  tables + 44,1 / 48 kHz spreading operators.** Following the 32 kHz
  D.3a table the `psy` module now carries the full 57-partition
  (44,1 kHz, `CALC_PARTITION_44K1`) and 58-partition (48 kHz,
  `CALC_PARTITION_48K`) calculation-partition tables transcribed from
  `docs/audio/mp3/annex-d-table-D3b-calc-partition-44k1Hz.csv` /
  `…-D3c-…-48kHz.csv` (partition index, inclusive ωlow/ωhigh FFT-line
  bounds, median Bark `bval`, `minval` floor, `TMN` offset), with
  `calc_partition_44k1` / `calc_partition_48k` accessors. With the
  `bval` columns in hand, the clause D.2.3 partition-domain spreading
  operator is now derivable at both higher rates: new
  `model2_spread_weight_44k1/48k`, `model2_spreading_matrix_44k1/48k`,
  `model2_spread_normalization_44k1/48k`, and the energy-conserving
  `model2_spread_energy_44k1/48k`, all sharing a rate-generic
  `bval`-parameterised core. Tests verify the 49 < 57 < 58 partition
  counts, dense 1-based numbering, contiguous FFT-line tiling to the
  Nyquist line 513, strictly-monotone `bval` with the printed
  25,33 / 25,81 endpoints, the errata-noted `minval` drop to 3,5 dB on
  the final partition(s), and that the higher-rate spreading operators
  reproduce the 32 kHz operator's diagonal-≈1 / energy-conservation /
  unit-impulse-normalisation properties. This leaves only the per-line
  absolute-threshold Tables D.4a–c staged-but-untranscribed in the
  Model 2 Annex D family.

- **Annex D Table D.1c — complete 102-row threshold-in-quiet table
  (Layer I, Fs = 48 kHz).** Completing the Layer I D.1 family
  (D.1a 32 kHz / D.1b 44,1 kHz / D.1c 48 kHz), the `psy` module now
  carries the 48 kHz absolute-threshold (LTq) table transcribed from
  `docs/audio/mp3/annex-d-table-D1c-threshold-48kHz.csv` into the new
  `LTQ_L1_48K` const (102 rows: 1-based index, FFT-line frequency,
  critical-band rate in Bark, absolute-threshold dB). The 48 kHz grid
  is the coarsest per Bark of the three Layer I rates, so the printed
  table is the shortest (102 < 106 < 108). New accessors
  `ltq_layer1_48k(i)` (direct slot lookup, `None` for `i == 0` and
  `i > 102`) and `ltq_layer1_48k_used(i, kbps)` (Step 3 LTq offset
  composition) mirror the 32/44,1 kHz pairs. Tests verify dense
  1-based numbering, strictly-monotonic frequency/Bark, the
  non-monotonic LTq column's global minimum of `-4.98 dB` at `i = 35`,
  the 68,00 dB ceiling saturation from `i = 91`, the 93,75 Hz head
  grid, cross-checks against every Table D.2c band boundary, and the
  strict 48k > 44,1k > 32k frequency ordering at every shared index.
  The Layer II tables (D.1d–f) remain staged as CSVs.

- **Annex D Table D.1b — complete 106-row threshold-in-quiet table
  (Layer I, Fs = 44,1 kHz).** Following the 32 kHz D.1a table, the
  `psy` module now carries the 44,1 kHz absolute-threshold (LTq)
  sibling transcribed from
  `docs/audio/mp3/annex-d-table-D1b-threshold-44k1Hz.csv` into the new
  `LTQ_L1_44K1` const (106 rows: 1-based index, FFT-line frequency,
  critical-band rate in Bark, and the absolute-threshold dB). The
  44,1 kHz FFT-line grid is coarser per Bark than the 32 kHz grid, so
  the printed table ends two rows short of D.1a's 108. New accessors
  `ltq_layer1_44k1(i)` (direct slot lookup `LTQ_L1_44K1[i - 1]`,
  `None` for `i == 0` and `i > 106`) and `ltq_layer1_44k1_used(i,
  kbps)` (Step 3 LTq offset composition) mirror the 32 kHz pair. Tests
  verify dense 1-based numbering, strictly-monotonic frequency/Bark,
  the non-monotonic LTq column's global minimum of `-4.98 dB` at
  `i = 39`, the 68,00 dB ceiling saturation from `i = 95`, the
  ≈86,13 Hz head grid through `i = 48`, cross-checks against every
  Table D.2b band boundary (with the last-digit rounding divergence
  between the D.1 and D.2 extractions documented), and that the
  44,1 kHz grid sits above the 32 kHz grid at every shared index. The
  48 kHz Layer I table (D.1c) and the Layer II tables (D.1d–f) remain
  staged as CSVs, not yet transcribed.

- **Annex D Table D.1a — complete 108-row threshold-in-quiet table
  (Layer I, Fs = 32 kHz).** The `psy` module previously carried this
  absolute-threshold (LTq) table as a 6-row partial anchor (rows 1–5 +
  108), with the body `i = 6..=107` marked DOCS-GAP behind a PNG render.
  The docs collaborator's Annex-D extraction round (`docs` #129) staged
  the full table as text in
  `docs/audio/mp3/annex-d-table-D1a-threshold-32kHz.csv`; all 108 rows
  (1-based index, FFT-line frequency, critical-band rate in Bark, and
  the absolute-threshold dB) are now transcribed into the `LTQ_L1_32K`
  const. `ltq_layer1_32k(i)` resolves every printed index `1..=108` by
  direct slot lookup (`LTQ_L1_32K[i - 1]`) instead of returning `None`
  for the former gap body. New tests verify dense 1-based numbering,
  strictly-monotonic frequency/Bark, the non-monotonic LTq column's
  global minimum of `-4.97 dB` at `i = 51`, the 62,5 Hz head grid
  through `i = 48`, and full cross-checks against every Table D.2a band
  boundary. The 44,1 / 48 kHz Layer I tables (D.1b/c) and the Layer II
  tables (D.1d–f) are staged as CSVs but not yet transcribed.

- **§C.1.5.2.5 / §C.1.5.2.6 / Table C.4 — Layer II perceptual SCFSI
  selection.** The Layer II encoder previously emitted `scfsi == 0b00`
  (three independent scalefactors) for every allocated subband. It now
  implements the ISO/IEC 11172-3 §C.1.5.2.5 scalefactor coding:
  `select_layer2_scfsi([u8; 3]) -> (u8, [u8; 3])` classifies the two
  successive scalefactor-index differences `dscf1 = scf0−scf1`,
  `dscf2 = scf1−scf2` into the five §C.1.5.2.5 difference classes,
  looks up Table C.4 ("Layer II scalefactor transmission patterns",
  transcribed from the 400-DPI render of the in-repo ISO PDF page 82)
  for the "scalefactors used in encoder" pattern (where the spec's "4"
  selects the maximum-multiplier / minimum-index of the three) and the
  2-bit `scfsi` "selection information" code, and collapses the three
  scalefactors to one or two where the pattern allows. The collapsed
  indices drive **both** sample quantization (so the decoder reconstructs
  with the same scalefactor the encoder quantized against) and the
  §2.4.1.6 scfsi+scalefactor writer; the writer's consistency pre-flight
  is satisfied by construction. The §C.1.5.2.7 allocator continues to
  reserve the worst-case three-scalefactor cost, so the budget fit-check
  stays sound while the emitted frame collapses redundant scalefactors.
  **+5 lib-tests** cover the §C.1.5.2.5 class boundaries, the 2-bit
  range of every Table C.4 entry, the writer-invariant (collapsed parts
  pre-equalised) across a 9³ probe sweep, six hand-verified Table C.4
  rows, and a write→read round-trip confirming the decoder recovers the
  selected per-part indices losslessly. Closes the README "Not yet
  supported" §C.1.5.2.5 / Table C.4 frontier item.

- **Annex D clause D.2.3 — Model 2 spread-excitation application
  (Fs = 32 kHz).** `psy::model2_spread_energy_32k(&[f64]) -> Option<Vec<f64>>`
  applies the energy-conserving spreading operator (the already-landed
  `model2_spreading_matrix_32k` × `model2_spread_normalization_32k`) to a
  per-partition source-energy vector, returning the spread excitation
  `eb[d] = Σ_s (energy[s]·rnorm[s])·sprdngf[d][s]` per destination
  partition. This is the matrix–vector product the eventual Model 2
  threshold step consumes; it carries no `minval` floor or TMN/NMT
  tonality offset (clause D.2.4 combination rule), which remain a
  DOCS-GAP — the noise-masking-tone offset and tonality-index blend are
  not transcribed in the staged Annex D extract. Returns `None` on a
  length mismatch against `MODEL2_PARTITIONS_32K`. **+6 lib-tests**
  cover the length guard, exact agreement with the explicit matrix
  product, energy conservation (`Σ eb == Σ source`), unit-impulse
  column recovery, the zero-source case, and operator linearity
  (additivity + homogeneity). Total `cargo test -p oxideav-mp1 --lib`
  count: **355 → 361**.

- **Annex D clause D.2.3 — Model 2 partition-domain spreading operator
  (Fs = 32 kHz).** The per-pair spreading function `model2_sprdngf` is
  now composed over the **complete** 49-row Table D.3a (`bval` column,
  `CALC_PARTITION_32K`) into the full partition-domain spreading
  operator Model 2 evaluates — the matrix that takes per-partition
  energy and spreads it across the calculation partitions. New `psy`
  surface:

  * `psy::MODEL2_PARTITIONS_32K` — partition count (`bmax = 49`).
  * `psy::model2_spread_weight_32k(into, from) -> Option<f64>` — the
    per-pair power-domain weight `model2_sprdngf(bval[into],
    bval[from])` for 1-based partition indices (the destination's
    median Bark is the spread-into `j` argument, the source's is the
    masker `i` argument); `None` for `0` / out-of-range indices.
  * `psy::model2_spreading_matrix_32k() -> Vec<Vec<f64>>` — the dense
    `49 × 49` matrix `[d][s] = model2_sprdngf(bval[d], bval[s])`.
    Non-symmetric (Bark spreading is steeper below the masker than
    above), every entry a non-negative power weight that never exceeds
    the `≈ 1` diagonal.
  * `psy::model2_spread_normalization_32k() -> Vec<f64>` — the clause
    D.2.3 `rnorm[s] = 1 / Σ_d sprdngf[d][s]` per-source-partition
    normalisation factors that make the operator energy-conserving.

  The 32 kHz operator is fully derivable in tree because D.3a is the
  one calculation-partition table transcribed complete; the 44,1 /
  48 kHz operators stay DOCS-GAP until Tables D.3b / D.3c are
  transcribed off their PNG renders. No allocator wiring changed —
  this assembles the spreading operator the eventual Model 2 allocator
  will use. **+11 lib-tests** cover the partition count, the `≈ 1`
  diagonal, per-pair / matrix agreement, out-of-range `None`, the
  never-amplify bound, the down-vs-up asymmetry, off-diagonal decay on
  the shallow side, and the `rnorm` finite-positive + energy-
  conservation properties (a unit source impulse scaled by `rnorm[s]`
  and spread over every destination sums back to exactly 1). Total
  `cargo test -p oxideav-mp1 --lib` count: **344 → 355**.

- **Free-format Layer I *encoding* (§2.4.2.3 `bitrate_index == 0b0000`).**
  The Layer I encoder can now emit free-format frames via the new
  `EncodeParams::free_format_kbps` field / `EncodeParams::with_free_format(kbps)`
  builder. When set, the header's four-bit `bitrate_index` is written as
  `0b0000` while the frame is sized to a fixed, possibly off-ladder rate
  via the same §2.4.2.1 slot formula `N = floor(12 · kbps / Fs)` the
  fixed ladder uses. Targets whose slot count would exceed the §2.4.3.1
  512-slot Layer I limit (or a zero-rate target) are rejected with
  `EncodeError::UnsupportedBitrate`. This closes the encode-side
  counterpart to the existing decode-side
  `detect_free_format_frame_length` probe: a two-frame round trip
  confirms the probe recovers the same `N`, byte length and back-derived
  bitrate the encoder used, and a free-format frame still decodes to PCM
  through the standard §2.4.3.2 audio-data path. Free format composes
  with the optional §2.4.1.4 CRC (`with_emit_crc`), which still verifies.

- **`cargo-fuzz` decode harness (round 296 depth-mode lane).** A new
  self-contained `fuzz/` sub-crate (not a workspace member) carries a
  libFuzzer `decode` target over the registered
  [`oxideav_core::Decoder`] surface. It splits attacker bytes into a
  packet stream of raw slices and crafted frames (valid 4-byte header,
  every field — ID / layer I-or-II / bitrate / sample-rate / mode /
  mode-ext / CRC / padding — attacker-chosen, body sized to the
  header-implied frame length), driving `send_packet` → `receive_frame`
  plus mid-stream `reset` / `flush` and a forced double-flush. The
  contract under test is panic-freedom on arbitrary input across both
  the Layer I and Layer II decode chains and the §2.4.3.1 Mute /
  RepeatPrevious concealment paths. The current run is clean — no crash
  artifacts across ~160k executions. No decoder code changed; the fuzzer
  found no panic / overflow / OOB to fix.

- **Annex D Table D.3a (Model 2 calculation partition, Fs = 32 kHz)
  completed — all 49 rows.** The docs extract
  (`mp3-annex-d-psychoacoustic-extracts.md`) now transcribes the
  complete 49-partition 32 kHz table, cross-checked against the
  authoritative PNG render, with the partition count corrected from
  the earlier 63 OCR miscount to the printed `bmax = 49` (`docs`
  #129). The `psy` module's prior 20-row partial anchor is replaced
  by the full table:

  * `psy::CALC_PARTITION_32K: [CalcPartition; 49]` — the complete
    Table D.3a (`ωlow`, `ωhigh`, `bval`, `minval`, `TMN` per row);
    replaces the removed `CALC_PARTITION_32K_PARTIAL`.
  * `psy::CALC_PARTITION_32K_FULL_LEN` corrected `63 → 49`.
  * `psy::calc_partition_32k(n)` now resolves every partition
    `n ∈ 1..=49` to `Some(row)` (no more DOCS-GAP tail); `n == 0`
    and `n > 49` return `None`.

  The final partition `[497, 513]` reaches the Nyquist FFT line
  513 of the 1024-point Model 2 analysis FFT, with `bval` rising
  to 24,07 Bark, `minval` settled at 4,5 dB and `TMN` climbing to
  38,6 dB. Tests reworked to the full 49-row table: contiguity to
  Nyquist with widths tiling lines 1..=513, strictly-increasing
  `bval`, the `TMN` head-plateau / monotone tail, `minval`
  settling to 4,5 dB from partition 17, the first/last row spec
  anchors, and the lookup resolving every partition. The D.3b
  (44,1 kHz, `bmax = 57`) / D.3c (48 kHz, `bmax = 58`) tables
  remain staged as PNG renders.

- **Annex D Table D.1a (Layer I, 32 kHz) threshold-in-quiet partial
  anchor + Step 3 composition.** The docs extract transcribes rows
  `i = 1..=5` plus the final row `i = 108` of Table D.1a
  ("Frequencies, critical band rates and absolute threshold") as a
  render-cross-checked text anchor; those six rows now land in the
  `psy` module:

  * `psy::LtqRow` — typed four-column row (1-based `index`,
    `freq_hz`, `bark_z`, pre-offset `ltq_db`).
  * `psy::LTQ_L1_32K_PARTIAL: [LtqRow; 6]` — the anchored rows,
    plus `LTQ_LAYER1_FULL_LEN = 108` / `LTQ_LAYER2_FULL_LEN = 132`
    printed-table lengths from the clause D.1 prose.
  * `psy::ltq_layer1_32k(i) -> Option<LtqRow>` — `Some` on the
    anchor, `None` on the 1-based underflow, the DOCS-GAP body
    `i ∈ 6..=107`, and `i > 108` (mirrors the `calc_partition_32k`
    contract so allocator wiring can fall back through the
    energy-driven path).
  * `psy::ltq_layer1_32k_used(i, kbps) -> Option<f64>` — the Step 3
    composition `LTq_table(i) + ltq_offset_db(per-channel rate)`,
    feeding the first real Table D.1x data through the previously
    data-starved `step3_apply_ltq_offset` site.

  Eleven new lib-tests cover the verbatim rows, the anchor/full
  lengths, strict monotonicity, the 62,5 Hz head-row line grid vs
  the decimated row 108, the head-descending / tail-rising LTq
  shape, frequency/Bark agreement with every Table D.2a boundary
  whose `index F&CB` is anchored (1 / 3 / 5 / 108), the lookup hit
  and miss sets, Step-3 agreement with `step3_apply_ltq_offset`
  across rates including the 95/96 boundary with `None`
  propagation, and Step-4 placement of each anchored line into its
  expected D.2a critical band. Lib test count 327 → 338.

- **Model 2 spreading function complete — `tmpy` backbone + full
  `sprdngf` composition (DOCS-GAP closed).** The clause D.2.3 `tmpy`
  line, previously typeset as an equation image the PDF text layer
  could not extract, is fully legible in a 150-DPI render of the
  staged ISO/IEC 11172-3:1993 PDF page 135 (printed p.129):

  * `psy::model2_tmpy(tmpx) -> f64` — the printed
    `tmpy = 15,811389 + 7,5(tmpx + 0,474) − 17,5(1,0 + (tmpx +
    0,474)²)^0,5`. Asymptote slopes `25` dB per tmpx unit on the
    steep (below-masker) side and `−10` on the shallow side
    (26.25 / 10.5 dB per Bark after the `tmpx = 1.05·(j − i)`
    scaling); maximum `0 dB` (within `10⁻⁶`) at `tmpx ≈ 0`.
  * `psy::model2_sprdngf(j_bark, i_bark) -> f64` — the complete
    printed-form per-pair spreading function: `tmpx → x → tmpy`,
    then `sprdngf = 0` when `tmpy < −100` (cutoff on `tmpy`
    **alone**), else `10^((x + tmpy)/10)`. The same render shows
    the printed exponent carries the `x +` term that the PDF text
    layer drops (extraction reads `10^(tmpy/10)`); the
    previously-staged one-argument `sprdngf_from_tmpy` is therefore
    redocumented as the exact `x = 0` reduction, valid outside the
    `model2_x_is_active` window where the `x` clamp engages.

  Eleven new lib-tests cover the printed-form spot values (including
  the exact `tmpy(−0.474) = −1.688611` collapse), the `~0 dB` peak /
  global non-positivity / unimodality / both asymptote slopes of the
  backbone, `sprdngf(z, z) ≈ 1`, full-grid agreement with the
  recomposed chain, the never-amplifies bound, the `tmpy`-alone
  cutoff at both Bark crossings (`j − i ≈ −4.79` / `+10.51`), the
  `x = 0` reduction agreement outside the active window, the strict
  in-window sharpening, and the piecewise decay shape with the
  genuine printed-form dip (`tmpx ≈ 2.049`) / crest (`tmpx = 2.5`)
  pair at the `x`-window exit.

  Total `cargo test -p oxideav-mp1 --lib` count: **316 → 327**.

- **Model 2 spreading-function `x` term — per-pair adapter +
  active-region predicate.** Two narrow helpers close the
  `(j_bark, i_bark)` composition gap between the already-staged
  [`model2_tmpx`] (`tmpx = 1.05 · (j − i)`) and [`model2_x`]
  (`x = 8 · min((tmpx − 0.5)² − 2·(tmpx − 0.5), 0)`) text-extracted
  pieces of clause D.2:

  * `psy::model2_x_for_pair(j_bark, i_bark) -> f64` — one-line
    composition `model2_x(model2_tmpx(j_bark, i_bark))`. Mirrors
    the existing `step3_apply_ltq_offset` adapter for
    `ltq_offset_db` — exposes the per-pair spreading-function `x`
    contribution as a single call so callers walking calculation
    partition pairs do not have to re-derive the composition.
  * `psy::model2_x_is_active(j_bark, i_bark) -> bool` — `true`
    iff the `x` term is strictly negative for this pair, i.e.
    `tmpx ∈ (0.5, 2.5)` (equivalently `(j − i) ∈
    (0.5/1.05, 2.5/1.05) ≈ (0.476, 2.381)` Bark). Outside that open
    interval `model2_x_for_pair` is exactly `0.0` (the `min(_, 0)`
    clamp engages); the predicate lets callers short-circuit the
    still-DOCS-GAP per-pair `tmpy` evaluation site for inactive
    pairs.

  Nine new lib-tests cover the `_for_pair` adapter (two-step
  composition agreement on points spanning every region, sign
  clamp `<= 0` on a 26×26 integer grid, `j == i → 0`, strict
  negativity inside the active window, exact `0.0` on both
  outside-window directions including the closed endpoints), and
  the `_is_active` predicate (fine-grid agreement with the
  numerical `< 0` test of `_for_pair`, exclusive open-interval
  endpoints, `false` at `j == i`, `false` for `j < i`).

  Lib test count: **307 → 316**.

- **Annex D Step 4 + Step 3 + Step 7 composer helpers.** Three
  closed-form adapter helpers join the existing Annex D building
  blocks into the per-line composition sites the eventual perceptual
  allocator wires:

  * `psy::critical_band_for_line(layer, fs, line_index_fcb) ->
    Option<usize>` — Step 4 FFT-line → critical-band lookup. Given an
    FFT-line index (1-based into the matching Table D.1x `index F&CB`
    column), returns the 0-based critical-band number from the
    matching Table D.2x list. Rejects `line == 0` (the spec is
    1-based), unsupported `(layer, fs)` combinations (anything outside
    `{Layer::I, Layer::II} × {32 000, 44 100, 48 000}` Hz — Annex D is
    MPEG-1-only), and lines above `bands.last().index_fcb`. The
    boundary column is the **upper** edge of each band, so a line
    equal to a band's top maps to that band (k) not the next (k+1).
  * `psy::step3_apply_ltq_offset(ltq_table_db,
    bit_rate_per_channel_kbps) -> f64` — Step 3 per-line composition
    site. One-line adapter that adds `ltq_offset_db(kbps)` (already
    staged: `-12 dB` for `kbps ≥ 96`, `0 dB` otherwise) to a Table
    D.1x absolute-threshold value to yield `LTq_used(i)`. The
    function exists ahead of the still-DOCS-GAP D.1x render so the
    composition site is fixed.
  * `psy::global_threshold_db_from_maskers(z_i, ltq_used_db,
    tonal_maskers, non_tonal_maskers) -> f64` — Step 6 + Step 7
    composite per-line global-threshold convenience entry point that
    takes raw `(z_j, X_db)` masker pairs and an evaluation-line Bark
    `z_i`, evaluates each masker through the matching
    `individual_threshold_*` helper, drops any masker the `vf`
    spreading-function window (`-3 <= dz < 8`) ignores via the `None`
    return, and feeds the survivors into the same power-domain Step 7
    sum the existing `global_threshold_db` performs. Removes the
    caller-side trap of having to pre-filter the slices manually
    against the spreading-function window.

  Fifteen new lib-tests cover the Step 4 helper (zero-line rejection,
  unsupported-rate rejection, above-top return-`None`, brute-force
  cross-check against the boundary walk for every line up to each
  table's top, the band-top edge convention `line == bands[k].index_fcb
  → band k`, and known D.2a / D.2e anchors), the Step 3 helper (high-
  rate `-12 dB` subtraction, low-rate pass-through, the 95/96
  inclusive boundary), and the Step 7 composer (empty maskers yield
  LTq, out-of-window maskers are dropped, the result matches the
  pre-filtered `global_threshold_db` call on the same input, mixed
  in/out-of-window maskers reduce to the only-inside call, and a loud
  in-window masker raises LTg by ≫ 30 dB above LTq).

  Lib test count: **292 → 307**.

- **Annex D Table D.3a (Fs = 32 kHz) calculation-partition partial
  anchor.** Clause D.2 prints "Psychoacoustic Model 2 calculation
  partition table" at the 32 kHz sampling rate as a 63-row block
  (partition index `n`, FFT-line span `[ωlow_n, ωhigh_n]`, median
  Bark value `bval`, minimum masking-spread floor `minval` dB and
  tone-masking-noise offset `TMN` dB); the dense page is staged
  behind a PNG render in
  `docs/audio/mp3/annex-d-renders/Table-D.3a-calc-partition-32kHz-p133.png`
  and the docs extract transcribes the first 20 rows as text.
  Those 20 rows now land in `psy` as a typed five-column const
  `CALC_PARTITION_32K_PARTIAL: [CalcPartition; 20]`, with a
  `CalcPartition::width()` accessor for the implicit
  `ωhigh − ωlow + 1` FFT-line count. The printed full-table length
  lands as `CALC_PARTITION_32K_FULL_LEN = 63` and the lookup helper
  `calc_partition_32k(n)` returns `Some(row)` for the anchor
  `n ∈ 1..=20` and `None` for the still-DOCS-GAP tail
  `n ∈ 21..=63`, so downstream consumers wiring the Annex D
  allocator can branch on `None` to fall back through the
  signal-energy-driven path until the remainder is transcribed.
  This mirrors the prior Table D.5 (`CODER_PARTITIONS`) staging
  shape — typed const, explicit 1-based numbering, DOCS-GAP
  boundary surfaced in the type. Nine new lib-tests cover the
  anchor row count, the dense 1-based `index` field, the
  contiguous FFT-line tiling, the `width()` accessor, the
  partition-1 anchor, the strictly-monotonic Bark axis, the
  head-region TMN plateau (24.5 dB through partition 14) plus the
  strictly-increasing TMN tail (partitions 15..=20 starting at the
  24.8 dB anchor), the settled-region `minval == 4.5` dB for
  partitions 17..=20, the `calc_partition_32k(n)` lookup helper
  (anchor → `Some`, both 1-based underflow and DOCS-GAP tail →
  `None`), and the per-row widths matching the extract (1 line for
  partition 1, 3 lines for partitions 2..=13, 4 lines for
  partitions 14..=20). Public surface: new `CalcPartition` struct,
  `CALC_PARTITION_32K_PARTIAL` const, `CALC_PARTITION_32K_FULL_LEN`
  const and `calc_partition_32k` helper in the `psy` module.
  Total `cargo test -p oxideav-mp1 --lib` count: **282 → 292**.

- **`Mp1Encoder` §2.4.1.5 / §2.4.1.6 Layer-I / Layer-II top-level
  dispatch switch.** The `oxideav_core::Encoder` trait object the
  registry hands back from `make_encoder` (and the direct factory twin
  `encoder::make_encoder`) can now drive either the Layer I
  `Mp1FrameEncoder` or the Layer II `Mp1Layer2FrameEncoder` through one
  wrapper. The selection rides on a new `EncodeParams::layer:
  LayerSelect` field (default `LayerSelect::LayerI`, preserving
  byte-for-byte compatibility with the encoder's pre-switch
  behaviour), set via the `EncodeParams::with_layer(LayerSelect)`
  builder. A new factory pair exposes the Layer II branch directly:
  `encoder::make_encoder_layer2(&CodecParameters) ->
  Result<Box<dyn Encoder>, Error>` mirrors `make_encoder`'s
  `sample_rate` + `channels` (required) / `bit_rate` (optional)
  contract but defaults its bitrate to a per-rate / per-channel
  midpoint on the §2.4.2.3 Layer II ladder (128 / 192 kbit/s at the
  MPEG-1 rates, 64 / 96 at the 13818-3 §2.4.2.3 LSF rates), drives
  `Mp1Layer2FrameEncoder`, and consumes 1152 PCM samples per channel
  per `send_frame` (the Layer II §2.4.2.1 frame granularity) rather
  than the Layer I 384. The wrapper threads the per-layer granularity
  through `send_frame` rejection: a Layer I encoder rejects a
  1152-sample frame and a Layer II encoder rejects a 384-sample
  frame, each with the matching count in the error text. The
  §C.1.5.2.5 / Table C.4 perceptual SCFSI selection is still a
  PDF-image DOCS-GAP, so the Layer II branch keeps emitting `scfsi ==
  0b00` (three independent scalefactors) for every allocated subband
  per the inherited `Mp1Layer2FrameEncoder` behaviour — the dispatch
  switch is purely a routing change and does not alter the per-layer
  bitstream shape either branch produces. Public surface adds:
  `LayerSelect` re-exported at the crate root, `EncodeParams::layer`
  + `EncodeParams::with_layer`, and `encoder::make_encoder_layer2`.
  - **+4 lib-tests** cover: a regression that the default
    `make_encoder` factory keeps emitting Layer I (`layer == 0b11`)
    and rejects a 1152-sample send as off-granularity; a Layer II
    factory check that 48 kHz / 128 kbit/s mono produces a frame
    whose header parses as Layer II (`layer == 0b10`), whose length
    matches the §2.4.2.1 `floor(144 · bitrate / Fs)` value (384 bytes
    at this configuration), and which rejects a 384-sample send; a
    44.1 kHz / 192 kbit/s stereo Layer II encode → decode round-trip
    across six consecutive frames asserting the decoder surfaces 1152
    samples / 4608 interleaved S16 bytes per frame and that the run
    carries non-silence end-to-end (the synthesis filterbank ramps);
    and a parameter-validation rejection test matching the Layer I
    factory's behaviour for missing `sample_rate` / `channels`.
  - Total `cargo test -p oxideav-mp1 --lib` count: **278 → 282**.
- **§2.4.1.8 `ancillary_data()` emission on the Layer II encoder side.**
  A new `encode::encode_layer2_frame_with_ancillary(&Layer2HeaderParams,
  &subbands, &[u8]) -> Result<Vec<u8>, Layer2EncodeError>` is the
  ancillary-aware companion to `encode_layer2_frame`: it copies the
  caller-supplied §2.4.1.8 `ancillary_bit` payload into the §2.4.2.1
  frame tail that begins immediately after the §2.4.1.6 audio-data
  region (the trailing partial byte of the samples region is
  byte-aligned by `BitWriter::finish` before the ancillary tail is
  written, so ancillary always starts on a whole-byte boundary).
  When the payload exceeds the available tail space the call surfaces
  a new typed `Layer2EncodeError::AncillaryTooLarge { space, got }`
  variant. The §2.4.3.1 CRC patch — when `Layer2HeaderParams::has_crc`
  is enabled — runs after the ancillary copy and continues to verify
  clean through `verify_layer2_crc` because the Annex B Table 3-B.5
  protected region (header bits 16…31 + allocation + scfsi) does not
  cover the §2.4.1.8 tail.
  - `Mp1Layer2FrameEncoder::set_pending_ancillary(&[u8])` /
    `pending_ancillary()` / `clear_pending_ancillary()` stage a
    one-shot ancillary payload that the next `encode_frame` call
    consumes (success or `AncillaryTooLarge`). `reset()` also drops
    a staged payload so a seek doesn't leak ancillary bytes into the
    next frame. The default behaviour — no staged payload — keeps
    the §2.4.1.8 tail zero-padded, preserving byte-for-byte
    compatibility with frames produced by the existing
    `Mp1Layer2FrameEncoder` API.
  - **+5 lib-tests** cover: a `Mp1Layer2FrameEncoder` mono 48 kHz /
    192 kbit/s silence-input frame where the staged payload appears at
    the first byte where the ancillary frame diverges from a reference
    no-ancillary frame and the post-payload bytes are zero-padded; the
    decoder's §2.4.1.6 allocation map is unchanged by the ancillary
    payload; an oversized-payload path that surfaces the
    `AncillaryTooLarge { space, got }` variant; the
    `set_pending_ancillary` → `encode_frame` → `encode_frame`
    "consumed once" semantics, verified by comparing frame 1 of an
    ancillary-staging encoder against frame 1 of a second encoder
    that ran the same one-shot sequence; a `clear_pending_ancillary`
    test that compares against a fresh encoder; and a CRC + ancillary
    round-trip that confirms the §2.4.3.1 CRC word at bytes 4..6 is
    bit-identical to the no-ancillary reference and the staged
    payload still lands at the first differing byte. Public surface
    add: re-exported `encode_layer2_frame_with_ancillary` at the
    crate root. Total `cargo test -p oxideav-mp1 --lib` count:
    **273 → 278**.

- **§C.1.3 stateful Layer II frame encoder (`Mp1Layer2FrameEncoder`).**
  A new `encode::Mp1Layer2FrameEncoder` carries one `AnalysisFilter`
  per channel — the same §C.1.3 input-FIFO state the Layer I
  `Mp1FrameEncoder` owns — and exposes a single
  `encode_frame(pcm: &[f64]) -> Result<Vec<u8>, Layer2EncodeError>`
  call that consumes exactly `LAYER2_SAMPLES_PER_FRAME` (= 1152)
  interleaved PCM samples per channel, runs 36 slots × 32-PCM-sample
  analysis to build the `subbands[ch][sb][slot]` matrix, and
  dispatches to [`encode_layer2_frame`]. This is the Layer II
  analogue of `Mp1FrameEncoder` (which packs 12 slots × 32 sub-bands =
  384 PCM samples per channel per Layer I frame), closing the
  long-standing followup that the previous top-level Layer II
  encoder entry point [`encode_layer2_frame`] only accepted
  pre-analysed sub-band matrices and gave the caller no built-in
  way to carry §C.1.3 analysis-filter history across frames.
  - `Mp1Layer2FrameEncoder::new(Layer2HeaderParams)` builds an
    encoder with fresh (zeroed) analysis history.
    `reset()` zeros the per-channel `AnalysisFilter` input FIFO for
    a seek / stream restart, `channels()` reports the
    header-implied channel count (1 or 2), and `params()` exposes a
    read-only view of the configured header.
  - The §2.4.1.4 CRC opt-in flows through `Layer2HeaderParams::has_crc`
    exactly as for the underlying [`encode_layer2_frame`] — when
    enabled, the emitted §2.4.3.1 CRC verifies clean through
    `verify_layer2_crc`, and the §2.4.2.1 byte count is unchanged
    (the CRC's 16 bits come out of the audio-data budget).
  - New `Layer2EncodeError::WrongSampleCount { got }` variant
    surfaces a PCM length that is not exactly `1152 · channels`.
    Off-ladder bitrates and unsupported sampling frequencies are
    surfaced as `Layer2EncodeError::Header(...)` on the first
    `encode_frame` call, matching `Mp1FrameEncoder`'s lazy-
    validation contract.
  - **+7 lib-tests** cover: a mono 48 kHz / 128 kbit/s end-to-end
    PCM → encoded-frame → decoded-sub-bands round trip that
    confirms the §2.4.2.1 byte count, the §2.4.1.3 `Layer::II` /
    `ID == 1` header re-parse, and that the encoder places at least
    one non-zero allocation; a joint-stereo 44.1 kHz / 192 kbit/s
    round trip that asserts the §2.4.1.6 shared-upper-band
    invariant (ch0 and ch1 allocations identical for
    `sb ∈ [bound, sblimit)`) and that each channel placed at least
    one allocation; an LSF mono 24 kHz / 64 kbit/s round trip
    routing through Table B.1 (`ID == 0`); a `WrongSampleCount`
    rejection at a 1024-sample input (1152 required); an
    off-ladder `Layer2HeaderError::UnsupportedBitrate(100)` surfacing
    through `Layer2EncodeError::Header(..)`; a `reset` test that
    primes an encoder with one signal then resets and re-encodes a
    second signal, comparing byte-for-byte against a fresh encoder
    given the same second signal (confirming the analysis-filter
    FIFO is fully zeroed); and a `has_crc == true` round trip whose
    §2.4.3.1 CRC verifies through `verify_layer2_crc` and whose
    §2.4.2.1 byte count matches the no-CRC variant. Public surface
    add: re-exported [`Mp1Layer2FrameEncoder`] at the crate root.
    Total `cargo test -p oxideav-mp1 --lib` count: **266 → 273**.

- **§2.4.1.6 / §C.1.5.2 top-level Layer II frame encoder.** A new
  `encode::encode_layer2_frame(&Layer2HeaderParams, &subbands) ->
  Result<Vec<u8>, Layer2EncodeError>` wires together every Layer II
  encoder piece already in tree (per-part scalefactor extraction via
  `select_layer2_scalefactors`, the §C.1.5.2.7 iterative bit allocator
  `allocate_bits_layer2`, the §2.4.1.6 four-region writers, and a
  new §C.1.5.2 / §2.4.3.3.4 per-sample quantizer
  `quantize_layer2_sample`) behind a single call that takes only the
  §2.4.1.3 header parameters and the analysed per-(ch, sb) sub-band
  matrix. The function emits a complete §2.4.2.1 Layer II frame —
  header + optional §2.4.1.4 CRC + §2.4.1.6 allocation, scfsi,
  scalefactor and samples regions + zero-padded ANC tail — exactly
  `floor(144 · bitrate / Fs) + padding_bit` bytes long.
- **§2.4.3.3.4 inverse quantizer.** New
  `encode::quantize_layer2_sample(value, scf, &QuantClass) -> u32`
  computes the raw `nlevels`-level code the §2.4.1.6 SAMPLES writer
  consumes, inverting the decoder's `s'' = C · (s''' + D)` formula
  step-by-step (normalise by the scalefactor, solve for s''', scale to
  a signed `bits_per_sample`-bit integer, reinterpret unsigned, XOR the
  MSB) and clamping the result to `[0, nlevels)` so the writer's
  `SampleCodeOutOfRange` pre-flight is never tripped. Forms the exact
  inverse of the decoder's `requantize_triplet`: for every legal
  `(class, code)` the decoder reconstruction round-trips back to the
  same code through the new encoder.
- **§2.4.1.6 intensity-stereo allocation mirroring.** In the top-level
  Layer II encoder the per-channel bit allocator runs against
  pre-mirrored peaks for `sb ∈ [bound, sblimit)` (each cell gets the
  channel max) and the resulting per-channel allocations are merged in
  the shared upper band so the §2.4.1.6 writer's
  `UpperBandChannelsDisagree` invariant always holds.
- **§2.4.1.4 CRC emission on the Layer II encode side.** When
  `Layer2HeaderParams::has_crc == true`, `encode_layer2_frame` reserves
  a 16-bit `error_check()` placeholder immediately after the header,
  then patches it with the §2.4.3.1 CRC-16 computed off the just-
  written allocation + scfsi region via `compute_layer2_crc`. The
  decoder's `verify_layer2_crc` accepts the encoded frame's CRC as
  `Ok`. §2.4.2.1 frame byte count is unchanged.
- **5 lib-tests** for the new wiring: a full sweep across every Table
  3-B.4 quantization class in B.2a confirming the
  `quantize_layer2_sample` round-trip recovers every legal code; an
  out-of-range PCM clamp test that keeps the writer's pre-flight
  happy; a mono 48 kHz / 128 kbit/s end-to-end encode → decode round
  trip whose recovered samples sit within one quantizer step of the
  analysed input on every allocated subband; a joint-stereo 44.1 kHz /
  192 kbit/s round-trip that checks the shared upper band carries
  matching allocations and the decoded ch0/ch1 sample triplet is
  identical pre-scalefactor (the §2.4.1.6 shared-`s_dp` invariant);
  a §2.4.1.4 CRC opt-in round-trip that verifies the encoder's CRC
  through `verify_layer2_crc`; an off-ladder-bitrate rejection path
  surfaced as `Layer2EncodeError::Header(UnsupportedBitrate)`; and a
  LSF (24 kHz / 64 kbit/s) Layer II mono round-trip exercising the
  `ID == 0` Table B.1 path through the same top-level entry point.
  Total `cargo test -p oxideav-mp1 --lib` count: **259 → 266**.
- **ISO/IEC 13818-3 Annex B Table B.1 LSF Layer II allocation table
  transcribed.** Previously this crate aliased the MPEG-2 LSF Layer II
  (`Fs ∈ {16, 22.05, 24} kHz`) decode path to MPEG-1 Layer II Table
  3-B.2b (sblimit = 30, Σ nbal = 94). ISO/IEC 13818-3:1997 §2.4.3.1
  "Audio Decoding Layer I, II" (printed p.49) substitutes **Table B.1
  "Possible quantisation per subband, Layer II — Sampling frequencies
  16; 22,05; 24 kHz"** (printed p.71 / PDF page 81) for all of B.2a..d
  at every LSF bitrate. The replacement table has `sblimit = 30` but a
  strictly narrower per-subband ladder: `nbal = 4` for `sb 0..=3`
  (15-column ladder `{3, 5, 7, 9, 15, 31, 63, 127, 255, 511, 1023,
  2047, 4095, 8191, 16383}`), `nbal = 3` for `sb 4..=10` (7-column
  ladder `{3, 5, 9, 15, 31, 63, 127}`), `nbal = 2` for `sb 11..=29`
  (3-column ladder `{3, 5, 9}`), and `nbal = 0` for `sb 30..=31`
  (silenced). `Σ nbal = 4·4 + 7·3 + 19·2 = 75` exactly matches the
  footer printed below the table. Implemented in
  `tables_layer2::TABLE_LSF` and wired into the
  `layer2_bit_allocation_table(header)` selector — Layer II frames with
  `ID == 0` now resolve to `TABLE_LSF` rather than `TABLE_B2B`, so the
  Layer II decoder, the Layer II `audio_data()` / `error_check()` CRC
  sizing, and the Layer II frame writers (`write_layer2_allocation_field`,
  `write_layer2_scalefactor_field`, `write_layer2_samples_field`) all
  see the correct per-subband `nbal` widths and quantisation classes at
  16 / 22.05 / 24 kHz with no further changes to the per-region writer
  logic.
  - **+7 unit tests** cover: the per-subband `nbal` pattern (`{4, 4, 4,
    4, 3, 3, 3, 3, 3, 3, 3, 2, …, 2, 0, 0}`) and `Σ nbal = 75` total
    across `{16, 22.05, 24} kHz × {8, 64, 144, 160} kbit/s`; the
    `sb 0..=3` row's 15-column quant-class resolution against the exact
    spec sequence; the `sb 4..=10` row's 7-column resolution; the
    `sb 11..=29` row's 3-column resolution; `sb 30..=31` `nbal = 0`
    silenced behaviour; a structural-distinction test that shows
    Table B.1 differs from B.2b at `nbal(5)` (B.1 = 3, B.2b = 4) and at
    the column count for `sb = 11` (B.1 has 3, B.2b has 7); and a
    pointer-identity sweep over the full 13818-3 §2.4.2.3 Layer II/III
    LSF bitrate ladder (`{8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112,
    128, 144, 160}` kbit/s) at each of the three LSF rates confirming
    the §2.4.3.1 "for all bitrates" invariance.
  - Total `cargo test -p oxideav-mp1 --lib` count: **242 → 259**.

- **§2.4.3.1 free-format frame-length probe.** The previously-deferred
  free-format (`bitrate_index == 0b0000`) frame-length recovery is now
  implemented. ISO/IEC 11172-3 (1993) §2.4.3.1 states verbatim: *"If
  the bitrate index equals '0000', the exact bitrate is not indicated.
  N can be determined from the distance between consecutive syncwords
  and the value of the padding bit."* New
  [`header::detect_free_format_frame_length(header, after_header) ->
  Result<FreeFormatFrameLength, FreeFormatProbeError>`] inverts the
  §2.4.3.1 N formula: it scans `after_header` for the next position
  whose four bytes parse as a header on the same stream
  (`(ID, layer, sampling_frequency, mode)` all match — free-format
  holds the bitrate constant across consecutive frames), computes the
  byte-distance from the start of the current header, subtracts the
  `padding_bit` slot, and recovers `N`, the current frame's byte
  length, and the back-derived bitrate `kbps = N · Fs / (L · 1000)`
  (Layer I: `L = 12`, slot = 4 bytes; Layer II: `L = 144`, slot = 1
  byte, both per §2.4.2.1 / §2.4.3.1). Returns
  [`FreeFormatProbeError::NotFreeFormat`] for non-free headers,
  [`…::NoNextSync`] when no stream-matching candidate is found, and
  [`…::InconsistentDistance`] for a Layer I distance that is not a
  whole-slot multiple. New public items:
  [`header::detect_free_format_frame_length`],
  [`header::FreeFormatFrameLength`],
  [`header::FreeFormatProbeError`], all re-exported at the crate root.
  - **+10 unit tests** cover: the `NotFreeFormat` rejection on a fixed
    header; the `NoNextSync` reject when no candidate sync is present;
    a Layer I 32 kHz / 128 kbit/s frame (`N = 48` slots / 192 bytes)
    recovered against a stream-matching next header; the
    `padding_bit == 1` slot-subtract on the Layer I path (196 byte
    distance → `N = 48`); a Layer II 32 kHz / 96 kbit/s frame
    (`N = 432` bytes, slot = 1 byte) recovered against a Layer II
    next header; the stream-parameter mismatch rejection (a 32 kHz
    candidate must not be accepted for a 44.1 kHz current frame);
    Layer I `InconsistentDistance` on a 191-byte distance (not a
    multiple of 4); the boundary case of a four-byte degenerate
    frame (`N = 1`) at 48 kHz; the LSF Layer II / 24 kHz / 64 kbit/s
    padded case (385 byte distance → `N = 384`); and a payload
    cycling a non-matching candidate (32 kHz) followed by a real
    matching candidate (44.1 kHz) confirming the byte-by-byte scan
    lands on the right next-syncword. Total
    `cargo test -p oxideav-mp1 --lib` count: **242 → 252**.

- **Annex D Phase-3 — Step 3 `LTq` offset rule + Model 2 spreading
  function pieces.** The text-extractable Annex D building blocks
  beyond Step 6/7 are now staged in [`psy`]:
  - **`psy::ltq_offset_db(bit_rate_per_channel_kbps)`** — Annex D
    Step 3 offset applied to the §D.1 threshold-in-quiet `LTq(i)`
    column: `-12 dB` for per-channel rates `>= 96 kbits/s`, `0 dB`
    below. The spec wording draws the line at the per-channel rate
    inclusive of 96, exactly matching the rule's literal text.
  - **`psy::model2_tmpx(j_bark, i_bark)`** — clause D.2
    Psychoacoustic Model 2 spreading-function primary term
    `tmpx = 1.05 · (j − i)` (text-extracted verbatim).
  - **`psy::model2_x(tmpx)`** — clause D.2 secondary term
    `x = 8 · min((tmpx − 0.5)^2 − 2·(tmpx − 0.5), 0)`. Peaks at
    `tmpx = 0.5` (returns 0), is zero again at `tmpx = 2.5`, and is
    clamped non-positive everywhere.
  - **`psy::sprdngf_from_tmpy(tmpy_db)`** — clause D.2 post-step
    `sprdngf(i,j) = 0` when `tmpy < −100 dB`, else `10^(tmpy/10)`
    (text-extracted verbatim).
  - The intermediate `tmpy = …` line that bridges `x` to `sprdngf`
    is typeset as an image in the PDF and remains a DOCS-GAP; the
    legible pieces around it are now in tree so once the docs
    collaborator captures `tmpy` it can be plugged in without
    further changes to the cutoff.
  - **+15 unit tests** pin the LTq offset boundary (95 → 0, 96 →
    −12), the `model2_tmpx` sign convention, the `model2_x` clamp
    region (`(−∞, 0.5] ∪ [2.5, +∞) → 0`; strictly negative inside
    `(0.5, 2.5)`), and the `sprdngf` `tmpy = −100` boundary
    (inclusive on the active side, i.e. `10^−10`).

- **§C.1.5.1.4 Layer II scalefactor extraction** — new
  [`select_layer2_scalefactors`] and supporting
  [`layer2_subband_peak_per_part`] helpers compute the per-(ch, sb,
  part) Table 3-B.1 scalefactor indices an encoder needs to populate
  [`Layer2ScalefactorFieldInput::scalefactor_indices`]. The 36
  analysed sub-band samples are split into three 12-slot §2.4.2.6
  scalefactor parts (slots 0..12 / 12..24 / 24..36); for each part
  the maximum of the absolute value is taken and fed through
  [`select_scalefactor`] (§C.1.5.1.4: "the lowest value in Table
  B.1 … which is larger than this maximum"). New
  [`Layer2ScalefactorIndices`] and [`Layer2SubbandPeaks`] type aliases
  match the §2.4.1.6 scalefactor-field input shape so the extracted
  indices flow into [`write_layer2_scalefactor_field`] unchanged. The
  SCFSI Table C.4 collapse remains a DOCS-GAP; callers continue to
  emit `scfsi == 0b00` (three independent scalefactors). Five new
  unit tests pin the part windowing, the `nch`/`sblimit` masking, the
  per-part agreement with [`select_scalefactor`], the all-zero
  fallback to the tiniest-multiplier index `62`, and a full extractor
  → §2.4.1.6 field-writer → [`BitReader`] round-trip.

## [0.0.6](https://github.com/OxideAV/oxideav-mp1/releases/tag/v0.0.6) - 2026-05-30

### Other

- Annex D Phase-2 — §D.1 critical-band tables + Step 6/7 psy formulae
- §2.4.1.6 / §2.4.3.3.4 Layer II SAMPLES region writer
- §2.4.1.6 Layer II scfsi + scalefactor field writer
- §2.4.1.6 Layer II allocation-field writer
- §2.4.2.3 Layer II frame-header writer
- §C.1.5.2.7 Layer II bit-allocation core + Table C.5 SNR ladder
- LSF stereo round-trip + planar S16P encoder input coverage
- optional §2.4.1.4 CRC emission on the Layer I encode side
- verify §2.4.3.1 CRC-16 for Layer II (header bits 16…31 + alloc + scfsi)
- Layer II (mp2) audio_data decode — PCM-match vs ffmpeg
- selectable §2.4.3.1 CRC concealment (mute / repeat-previous)
- verify §2.4.3.1 CRC-16 error_check + mute concealment
- add direct factory API (decoder/encoder modules)
- MPEG-2 LSF Layer I support (16 / 22.05 / 24 kHz)
- clean-room Layer I encoder + self-roundtrip tests
- complete Layer I decode to PCM (rescale + synthesis filterbank)
- Layer I audio-data decode to requantized subband samples
- fix redundant intra-doc link on Error::HeaderError ref
- clean-room Layer I frame-header foundation (ISO/IEC 11172-3)
- orphan rebuild — clean-room reset 2026-05-24

### Added

- **Annex D — Psychoacoustic Model 1 building blocks**. New
  `psy` module stages the text-extractable portions of ISO/IEC
  11172-3:1993 Annex D for a future perceptually-driven encoder
  allocator: **Tables D.2a–f** (critical-band boundaries — fully
  text-extracted; 24/25/26 bands for Layer I at 32/44,1/48 kHz,
  25/27/27 bands for Layer II at the same rates) exposed as
  `psy::critical_band_table(Layer, fs) -> Option<&'static
  [CriticalBand]>`; the **Step 6 closed-form masking-index**
  `av_tm = -1.525 − 0.275·z − 4.5 dB` /
  `av_nm = -1.525 − 0.175·z − 0.5 dB` as
  `psy::masking_index_tonal(z)` / `psy::masking_index_non_tonal(z)`;
  the **Step 6 four-piece masking-function** `vf(dz, X)` (low-far,
  low-near, high-near, high-far) as `psy::masking_function(dz, x_db)
  -> Option<f64>` returning `None` outside the spec's
  `-3 ≤ dz < 8` Bark window; the composite individual thresholds
  `psy::individual_threshold_tonal` / `…_non_tonal`; the
  **Step 7 global-threshold power-domain sum**
  `LTg = 10·log10(10^(LTq/10) + Σ 10^(LT/10))` as
  `psy::global_threshold_db(ltq, tonal, non_tonal)`; and **Table
  D.5** (the Layer I / Layer II coder partition table) as the 33-row
  `psy::CODER_PARTITIONS` array. **Tables D.1a–f, D.3a–c and D.4a–c
  are still PNG-only renders** and are deliberately NOT in tree —
  Annex D's threshold-in-quiet `LTq` and the Model 2 calculation
  partition + per-line absolute-threshold tables remain DOCS-GAP
  pending high-DPI text-extractable renders, and the existing
  signal-energy-driven [`encode::allocate_bits`] path is therefore
  unchanged. 29 new lib-tests cover: D.2 band counts matching the
  §D.1 prose (24/25/26 + 25/27/27), strict monotonicity of every
  table on each of the three columns, endpoint values for D.2a /
  D.2c / D.2d transcribed verbatim from the staged extract, every
  band's top-edge frequency below the sampling-frequency Nyquist,
  Annex D's MPEG-1-only scope (LSF rates 16 / 22.05 / 24 kHz are
  rejected with `None`), each `av` formula at `z = 0` against the
  closed-form constants, `av_tonal < av_non_tonal` for every
  `z ≥ 0`, monotonic decay of both `av` functions over the band
  range, classification of `dz` into each of the four `vf` branches
  at every interval boundary (including the exact `dz = -1` / `0` /
  `1` corners), the high-near branch matching `-17 · dz`
  irrespective of `X`, the low-near branch passing through the
  origin with slope `(0.4·X + 6)`, the low-far and high-far branches
  matching their level-dependent closed forms, the masker-ignored
  region returning `None`, the composite `LT_{tm,nm} = X + av + vf`
  identity, the global-threshold reducing to `LTq` with no maskers,
  two equal-dB powers summing to the textbook
  `10·log10(2) ≈ 3.0103 dB` headroom, and the loudest-term-dominance
  bound; plus Table D.5's 33-row length, `width = 0` for
  partitions 0–12 / `width = 1` for 13–32, strictly monotonic
  boundaries, and the +16 boundary step from `n = 1`. Total
  `cargo test -p oxideav-mp1 --lib` count: **193 → 222**.
- **Layer II §2.4.1.6 / §2.4.3.3.4 SAMPLES region writer**. New
  `encode::write_layer2_samples_field(&mut BitWriter, &AllocationTable,
  &Layer2Allocation, &Layer2SamplesFieldInput, nch, bound) ->
  Result<(), Layer2SamplesFieldError>` emits the §2.4.1.6 SAMPLES
  region — the bitstream region that immediately follows the scfsi +
  scalefactor region written by `write_layer2_scalefactor_field`. The
  writer mirrors the §2.4.1.6 / §2.4.3.3.4 decoder loop: 12
  syntax-granules outer; per granule, for each `(ch, sb)` in the low
  band `[0, bound)` it emits one triplet per channel; in the shared
  upper band `[bound, sblimit)` it emits a single triplet sourced from
  channel 0 (the §2.4.2.6 / §2.4.3.3 "intensity_stereo" rule
  mirroring into both channels at decode time). Each triplet is
  emitted per the per-`(sb, alloc)` `QuantClass` grouping flag —
  grouped classes pack three `0..nlevels` codes into one
  `bits_per_codeword`-wide field as `s0 + s1·N + s2·N²` (the exact
  inverse of the decoder's `c % N; c /= N` degrouping loop);
  non-grouped classes emit three separable `bits_per_codeword`-wide
  fields. Subbands with `alloc[ch][sb] == 0` and subbands `[sblimit,
  32)` emit zero bits (the §2.4.3.3.5 silenced-band rule). New input
  type `encode::Layer2SamplesFieldInput { codes: [[[[u32; 3]; SUBBANDS];
  12]; 2] }` carries the per-`(ch, gr, sb)` triplet of MSB-inverted
  unsigned codes the §2.4.3.3.4 decoder will read. The function
  pre-validates every cell before writing a single bit and surfaces
  five typed errors: `UnsupportedChannelCount` (`nch ∉ {1, 2}`),
  `BoundExceedsSblimit`, `MonoBoundBelowSblimit` (mono frames must use
  `bound == sblimit`), `InvalidAllocationCode` (a non-zero `alloc[ch]
  [sb]` that points at a `-` cell in the per-subband Tables 3-B.2x
  row), and `SampleCodeOutOfRange` (a code `≥ nlevels` for the
  resolved class — the decoder's MSB-inversion + degrouping math
  would silently corrupt the recovered samples otherwise). Together
  with the prior `write_layer2_header`, `write_layer2_allocation_field`
  and `write_layer2_scalefactor_field` this completes the four-region
  §2.4.1.6 control + audio-data payload of a Layer II frame. The
  remaining Layer II encoder followups are top-level `Mp1Encoder`
  integration of the four writers, a Layer-II `EncodeParams::layer`
  switch, and the §C.1.5.2.5 / Table C.4 perceptual SCFSI selection
  (still a PDF-image DOCS-GAP, hence the writer takes the SCFSI codes
  as caller input). Eleven new lib-tests cover: all-zero-allocation
  emits no bits; a dense-allocation total-bit-count check that walks
  the per-class `bits_per_codeword` and matches the writer's byte
  count (with a `pad_bits < 8` invariant on the trailing partial
  byte); a known-bit B.2c sb=0 grouped trace (12 grouped
  `samplecode = 5` codewords → `0x29 0x4A 0x52 0x94 0xA5 0x29 0x4A
  0x50`); a full alloc + scfsi + scalefactor + samples write that
  decodes back through `decode_layer2_audio_data` with every
  recovered §2.4.3.3.4 sample within `1e-12` of the closed-form
  expectation re-derived from the codes / scalefactors written; a
  joint-stereo `mode_extension = 0b00` write whose shared upper band
  sources triplets from channel 0 only and the decoder mirrors into
  both channels (verified per-channel against the same closed-form
  re-derivation, with the channel-1 input codes left at zero to
  confirm the writer ignores them); and every typed rejection path
  (`UnsupportedChannelCount` at 0 and 3, `BoundExceedsSblimit`,
  `MonoBoundBelowSblimit`, `InvalidAllocationCode`,
  `SampleCodeOutOfRange` at `code = nlevels`, plus a deep-position
  rejection at `(ch=1, sb=sblimit-1, gr=11)` confirming pre-flight
  emits zero bytes on error). Total `cargo test -p oxideav-mp1 --lib`
  count: **182 → 193**.
- **Layer II §2.4.1.6 scfsi + scalefactor field writer**. New
  `encode::write_layer2_scalefactor_field(&mut BitWriter,
  &AllocationTable, &Layer2Allocation, &Layer2ScalefactorFieldInput,
  nch, bound) -> Result<(), Layer2ScalefactorFieldError>` emits the
  two §2.4.1.6 bitstream regions that immediately follow the
  allocation field — the per-(ch, sb) 2-bit `scfsi` codes (one per
  non-zero allocation, sb-major / ch-minor over `[0, sblimit)`,
  including one scfsi per channel in the intensity_stereo upper band
  where the *allocation* is shared but `scfsi` is still read per
  channel), followed by the 1..3 six-bit Table 3-B.1 scalefactor
  indices per (ch, sb) emitted per the §2.4.2.6 SCFSI schedule
  (`0b00`: three reads; `0b01`: two reads — part 0 broadcast over
  parts 0+1, then part 2; `0b10`: one read broadcast over all three
  parts; `0b11`: part 0 then one read broadcast over parts 1+2). New
  input type `encode::Layer2ScalefactorFieldInput { scfsi: [[u8;
  SUBBANDS]; 2], scalefactor_indices: [[[u8; 3]; SUBBANDS]; 2] }`
  carries the per-(ch, sb) scfsi codes and per-(ch, sb, part) Table
  3-B.1 indices. The function pre-validates every cell before writing
  a single bit and surfaces:
  `UnsupportedChannelCount` (`nch ∉ {1, 2}`),
  `BoundExceedsSblimit`, `MonoBoundBelowSblimit` (mono frames must
  use `bound == sblimit`), `InvalidScfsiCode` (scfsi value `≥ 4`),
  `InvalidScalefactorIndex` (any 6-bit value `≥ 63` — the
  reserved/forbidden index that conformant encoders must not emit,
  per §2.4.3.2 prose), and three SCFSI-collapse checks
  (`ScfsiPartsInconsistent01`, `ScfsiPartsInconsistent10`,
  `ScfsiPartsInconsistent11`) for SCFSI codes whose collapse rule
  the caller's per-part array does not already satisfy — refusing to
  silently lose information that the decoder would not be able to
  recover. Together with the prior `write_layer2_header` and
  `write_layer2_allocation_field` this completes the §2.4.1.6 control
  region of a Layer II frame; the remaining Layer II encode followup
  is the §2.4.1.6 SAMPLES region (12 syntax-granules of triplets per
  (sb, ch)) and the §C.1.5.2.5 / Table C.4 SCFSI selection (still a
  PDF-image DOCS-GAP, hence the writer takes scfsi as caller input).
  Round-trip tested against the decode-path `BitReader` for every
  (ch, sb) scfsi and per-part index of a B.2a stereo frame cycling
  all four SCFSI codes, with per-error-case rejection coverage and
  two known-bit hand-traces (mono `scfsi=0b10` → 1 byte `0x89`,
  mono `scfsi=0b00` parts=[1,2,3] → 3 bytes `0x01 0x08 0x30`).
  Thirteen new lib-tests; `cargo test -p oxideav-mp1 --lib` count:
  **169 → 182**.

- **Layer II §2.4.1.6 allocation-field writer**. New
  `encode::write_layer2_allocation_field(&mut BitWriter, &AllocationTable,
  &Layer2Allocation, nch, bound) -> Result<(), Layer2AllocationFieldError>`
  emits the §2.4.1.6 per-(ch, sb) `allocation` bits MSB-first into an
  in-progress `BitWriter`, sized exactly per the Table 3-B.2x `nbal[sb]`
  widths: low band `[0, bound)` carries `nch · Σ nbal[sb]` bits
  (per-channel allocations) and the intensity_stereo upper band
  `[bound, sblimit)` carries one `nbal[sb]` slot per subband (shared
  between channels). The function pre-validates every cell before
  writing a single bit and surfaces:
  `UnsupportedChannelCount` (`nch ∉ {1, 2}`),
  `BoundExceedsSblimit`, `MonoBoundBelowSblimit` (mono frames must use
  `bound == sblimit`), `InvalidAllocationCode` (allocation does not fit
  in `nbal[sb]` bits or selects a `-` cell of the Table 3-B.2x row),
  `NonZeroAllocationAboveSblimit` (silent-drop guard for `sb ≥
  sblimit`), and `UpperBandChannelsDisagree` (shared upper-band cells
  whose two channels carry different values). Companion helper
  `encode::layer2_stereo_bound(&FrameHeader, sblimit)` exposes the
  §2.4.1.6 bound resolution (mode_extension lookup for joint_stereo,
  `sblimit` for every other mode, clamped to `sblimit`) so callers that
  already parsed a `FrameHeader` can drive the writer without
  duplicating the decode-side logic. Round-trip tested against the
  decode-path `BitReader` for every (sb, ch) cell of a joint_stereo
  frame, with per-error-case rejection coverage. Eleven new lib-tests;
  `cargo test -p oxideav-mp1 --lib` count: **158 → 169**.

- **Layer II §2.4.2.3 frame-header writer**. New
  `encode::pack_layer2_header(&Layer2HeaderParams) -> Result<[u8; 4],
  Layer2HeaderError>` packs the thirteen §2.4.1.3 header fields into
  the four big-endian header bytes for a Layer II frame; new
  `encode::write_layer2_header(&mut BitWriter, …)` streams the same
  bytes MSB-first into an in-progress `BitWriter`; new
  `encode::bitrate_index_layer2(kbps, id_bit)` exposes the §2.4.2.3
  Layer II bitrate-index lookup standalone, covering the MPEG-1
  Layer II ladder (`32 / 48 / 56 / 64 / 80 / 96 / 112 / 128 / 160 /
  192 / 224 / 256 / 320 / 384` kbit/s under `ID == 1`) and the
  shared LSF Layer II/III ladder (`8 / 16 / 24 / 32 / 40 / 48 / 56 /
  64 / 80 / 96 / 112 / 128 / 144 / 160` kbit/s under `ID == 0`).
  `Layer2HeaderParams::new(sampling_frequency, bitrate_kbps, mode)`
  builds a minimal parameter set (no padding, no CRC, no copyright,
  `original = 1`, `emphasis = None`); the `padding`, `private`,
  `copyright`, `original`, `emphasis`, `mode_extension` and
  `has_crc` fields are then set freely. The implied `ID` bit is
  resolved from `sampling_frequency` against the MPEG-1 (44.1 / 48
  / 32 kHz) and 13818-3 §2.4.2.3 LSF (16 / 22.05 / 24 kHz) tables;
  the `bitrate_index` is then resolved against the matching
  ladder. The writer emits no CRC word — the caller reserves a
  16-bit placeholder after the header when `has_crc` is set and
  patches it once the §2.4.1.6 allocation/scfsi region has been
  written, mirroring the Layer I `Mp1FrameEncoder::encode_frame`
  CRC patch path.
  - Twelve new lib-tests: `bitrate_index_layer2` ladder endpoints
    on both MPEG-1 and LSF (with the 448-kbit/s Layer-I-only and
    256-kbit/s LSF-Layer-I-only cross-rejection sanity checks); a
    known bit-by-bit pack of the canonical 128 kbit/s mono 44.1
    kHz header against the §2.4.1.3 field offsets; the
    `protection_bit` flip when `has_crc` is set and the
    `FrameHeader::has_crc` parse-back round-trip; the LSF `ID == 0`
    bit for the three LSF sampling frequencies; off-ladder bitrate
    rejection (`UnsupportedBitrate`) and unknown sampling-frequency
    rejection (`UnsupportedSamplingFrequency`); the full
    **14 × 3 × 4 = 168 bitrate × sampling-frequency × mode matrix**
    for both the MPEG-1 Layer II ladder and the LSF Layer II/III
    ladder, each entry packed and re-parsed through
    `FrameHeader::parse` to confirm every field round-trips
    bit-exact; the padding / private / copyright / original /
    emphasis / mode_extension carry path (each toggled
    independently); and the `BitWriter` streaming variant matching
    the byte-array pack byte-for-byte with subsequent MSB-first
    writes staying byte-aligned.
  - Total `cargo test -p oxideav-mp1 --lib` count: **146 → 158**.

- **Layer II §C.1.5.2.7 bit-allocation core** plus the §C.1.5.2 /
  Table C.5 "Layer II Signal-to-Noise Ratios" table needed to drive
  it. New `tables_layer2::layer2_snr_db(nlevels) -> Option<f64>`
  transcribes Table C.5 (ISO/IEC 11172-3 (1993) Annex C, PDF page 76),
  cross-checked against Table C.2 at the shared `nlevels` rows
  (3 → 7.00, 7 → 16.00, 15 → 25.28 dB, … all the way to
  32767 → 92.01 dB), and adds the three Layer-II-only rows
  (5 → 11.00, 9 → 20.84, 65535 → 98.01 dB). New `encode::
  allocate_bits_layer2(energy, nch, table, budget_bits)` runs the
  §C.1.5.2.7 iterative allocator (find subband with minimal MNR,
  raise to next legal Table-3-B.2x column, skipping `None` cells)
  with the Annex-D-DOCS-GAP signal-energy proxy in place of the
  perceptual SMR. Companion helpers `layer2_frame_payload_bits` and
  `sum_nbal_per_channel` make the §2.4.2.1 `adb = bytes·8 − bhdr −
  bcrc − bbal` budget directly testable.
  - Eleven new lib-tests in `encode.rs` + `tables_layer2.rs` cover
    the SNR table (known rows, monotonicity, Layer-I-overlap parity),
    the per-class bit cost (grouped vs non-grouped), the §2.4.2.1
    payload-bits formula (mono / stereo / CRC), the allocator's
    budget-fit / louder-first / silent-stays-zero / `sblimit` /
    legal-column properties, and the zero-budget no-op edge case.
    Total `cargo test -p oxideav-mp1 --lib` count: **135 → 146**.

- **MPEG-2 LSF stereo round-trip coverage and planar-S16P encoder
  input coverage** in `tests/roundtrip.rs`. No new spec material —
  every code path exercised is already covered by ISO/IEC 11172-3
  (1993) §2.4.1.5 and ISO/IEC 13818-3 (1997) §2.4.2.3; the additions
  pin uncovered behaviour of the existing `Mp1Encoder` /
  `Mp1Decoder` against regressions.
  - `lsf_stereo_tone_roundtrips_with_bounded_error_24khz` — two
    independent tones (one per channel) at LSF 24 kHz / 128 kbit/s
    (the LSF stereo factory default) confirm the §2.4.1.5 sb-major /
    ch-minor SAMPLES region encodes + decodes round-trip cleanly
    with `rms < 0.05` of full scale.
  - `lsf_stereo_silence_roundtrips_to_near_silence` — stereo silence
    at all three LSF rates (16 / 22.05 / 24 kHz, all at 128 kbit/s
    per the LSF stereo factory default) decodes to exact zero bytes,
    confirming the allocator drops every subband at the LSF stereo
    ladder.
  - `lsf_stereo_encoded_frame_carries_lsf_id_bit` — header round-trip
    check: a stereo 22.05 kHz encode emits `ID == 0` (LSF), the
    correct §2.4.2.3 LSF `sampling_frequency` code, and the
    `Mode::Stereo` field.
  - `encoder_accepts_planar_s16p_layout` and
    `encoder_accepts_planar_s16p_stereo_layout` — the
    `Mp1Encoder::frame_to_pcm` planar branch (codec.rs,
    `frame.data.len() == nch`) was uncovered. Both tests build the
    same PCM in interleaved and planar shapes and confirm the
    encoder produces byte-identical packets across the two layouts,
    pinning the planar deinterleave loop.
  - `encoder_rejects_wrong_plane_count` and
    `encoder_rejects_wrong_plane_size` — pin the typed-error
    rejection paths in `frame_to_pcm` when the caller passes neither
    `1` nor `nch` planes, or a plane whose byte count does not equal
    `samples * 2`.
  - 160 tests total (135 unit + 20 integration roundtrip + 3 integration
    layer2_mono + 2 doc), up from 153 (+7 from the planar / LSF-stereo
    coverage).
- **Optional §2.4.1.4 CRC `error_check()` emission on the Layer I
  encode side**, closing the long-standing followup recorded against
  the prior CRC verification work. The encoder previously always wrote
  `protection_bit == 1` and never emitted a CRC; it now does both on
  opt-in. Derived solely from ISO/IEC 11172-3 (1993) §2.4.1.4 / §2.4.2.3
  for the `protection_bit` semantics and §2.4.3.1 + Annex B Table 3-B.5
  for the protected-field set and CRC-16 generator polynomial — all
  three were already in the staged PDF for the Layer I / Layer II
  verification work.
  - `encode::EncodeParams` grows an `emit_crc: bool` field (default
    `false`, preserving the historical no-CRC encoder output) plus an
    `EncodeParams::with_emit_crc(bool)` builder and an
    `EncodeParams::new(bitrate, sampling_frequency, mode)` constructor
    so callers no longer need a struct literal to build params.
  - `encode::Mp1FrameEncoder::encode_frame` honours `emit_crc`: it
    writes `protection_bit == if emit_crc { 0 } else { 1 }`, leaves a
    16-bit placeholder immediately after the header when CRC is
    requested, finishes the audio data normally, and then patches the
    placeholder with the §2.4.3.1 CRC-16 over the Table 3-B.5 Layer I
    protected fields (header bits 16…31 + the bit-allocation field).
    `FrameHeader::compute_crc` is reused to drive the protected-field
    sizing, keeping the encoder and decoder CRC paths bit-identical.
  - `frame_payload_bits` already accounted for `has_crc`; passing the
    `emit_crc` flag through to it deducts the CRC's 16 bits from the
    §C.1.5.1.6 `adb` budget so the §2.4.2.1 slot count remains
    `N = floor(12 · bitrate / Fs) + padding` — a CRC-on and CRC-off
    encode of the same PCM produce byte-identical frame lengths.
  - `codec::Mp1Encoder` (registry path) and `encoder::make_encoder`
    (direct API) keep their default no-CRC behaviour. Two new
    factories — `codec::make_encoder_with_crc` and
    `encoder::make_encoder_with_crc` — produce a boxed `Mp1Encoder`
    with `emit_crc == true`, re-exported at the crate root as
    `oxideav_mp1::make_encoder_with_crc`. The two existing
    `make_encoder` factories continue to validate `sample_rate` /
    `channels` exactly as before; the with-crc variants share that
    validation through a common `make_encoder_inner` helper.
  - 13 new tests: 7 in `encode` (the `EncodeParams` default and builder,
    `protection_bit == 1` on the default encoder, `protection_bit == 0`
    + verifying CRC on the opt-in encoder, byte-count parity with and
    without CRC, mid-allocation-field corruption is detected as
    `CrcStatus::Mismatch` for a stereo frame, and a stereo
    encode-with-CRC round-trips clean), 4 in `codec` (registry-side:
    default `make_encoder` emits `protection_bit == 1`,
    `make_encoder_with_crc` emits a CRC the decoder accepts and a full
    encode-with-CRC → decode loop reaches non-silent PCM, plus the
    missing-`sample_rate`/`channels` rejection path on the new
    factory), and 2 in `encoder` (the direct-API `make_encoder_with_crc`
    round-trip, confirming the plain `make_encoder` still defaults to
    no CRC and both factories produce the same frame byte count). 153
    tests total (135 unit + 16 integration + 2 doc).
- **Layer II §2.4.3.1 CRC-16 `error_check()` verification**, closing
  the long-standing Layer II CRC followup from the §2.4.1.6 work.
  Derived solely from ISO/IEC 11172-3 (1993) §2.4.3.1 + Annex B Table
  3-B.5 (the Layer II protected fields: header bits 16…31 + the
  §2.4.1.6 bit-allocation field + the §2.4.1.6 scfsi field). The
  generator polynomial and shift-register initial state were already
  recovered for the Layer I CRC work and are now shared by both
  layers.
  - `decode_layer2::compute_layer2_crc(&FrameHeader, header_bytes, alloc_and_scfsi)`
    and `decode_layer2::verify_layer2_crc(&FrameHeader, header_bytes,
    after_header)` walk the per-frame Table 3-B.2x allocation row
    widths to size the bit-allocation field, parse it to find which
    subbands carry scfsi (one 2-bit scfsi per channel per non-zero
    allocation, including the §2.4.1.6 shared upper band where the
    allocation is shared but `scfsi` is still read once per channel),
    then feed the concatenated protected bits through the same
    §2.4.3.1 CRC-16 register (`G(X) = X^16 + X^15 + X^2 + 1`, init
    `0xFFFF`) used by the Layer I path. The CRC-16 helper is now
    shared `pub(crate)` between `header` and `decode_layer2` to avoid
    duplication.
  - The `Mp1Decoder` routes every CRC-protected Layer II packet
    through `verify_layer2_crc` and, on a mismatch, applies the
    selected `ConcealmentMode` to a Layer II concealment frame: the
    new `Mp1Decoder::conceal_layer2_frame` builds a 1152-sample
    output through the synthesis filterbank, either from zero
    subbands (`Mute` — `Layer2Subbands::silent`) or from the last
    successfully-decoded Layer II frame's 36-slot subband samples
    (`RepeatPrevious`). Because Layer II's `Layer2Subbands` carries 36
    sample-slots per subband (vs Layer I's 12) the two cannot share
    history storage, so a new `Mp1Decoder::last_layer2_subbands`
    field holds the Layer II repeat state alongside the existing
    Layer I `last_subbands`. The last-good-only repeat semantics, the
    first-frame `Mute` fallback, and `reset` dropping the repeat
    state all match the Layer I path; `reset` now clears both
    histories.
  - `Layer2Subbands::silent(channels)` is exposed publicly (mirroring
    `SubbandSamples::silent`) to give callers the §2.4.3.1 muting
    shape for Layer II (a `sblimit = 0` `Layer2Subbands` that
    `slot(ch, slot)` zeros out completely).
  - 8 new unit tests in `decode_layer2`: allocation-field bit count
    sizing for mono B.2a, scfsi field bit count when no subband is
    allocated and when several are, the full
    `compute_layer2_crc → verify_layer2_crc` round-trip, mismatch
    detection inside the allocation field, mismatch detection in
    header bits 16…31 (different `bitrate_index` produces a different
    CRC over the same allocation), mismatch detection inside the
    scfsi field (different scfsi bits produce a different CRC), and
    the `Absent` / truncated-buffer edge cases.
  - 5 new decoder-level tests in `codec`: a CRC-matching protected
    Layer II frame decodes to 1152 samples; a CRC-failing Layer II
    frame with the default Mute concealment produces a 1152-sample
    silent frame; `RepeatPrevious` on a CRC-failing Layer II frame
    reproduces exactly the PCM the previous good Layer II frame's
    subband samples would produce on an in-sync decoder; the
    first-Layer-II-frame RepeatPrevious case falls back to mute
    (nothing to repeat); and `reset` drops the Layer II repeat
    history while preserving the configured mode.
  - **Followups**: Layer II encoder; a transcription of the 13818-3
    LSF Layer II allocation table for Fs ∈ {16, 22.05, 24} kHz
    (currently mapped onto the 11172-3 B.2b table as a defensive
    default); optional CRC emission on the encode side (the encoder
    still always sets `protection_bit == 1`).
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
