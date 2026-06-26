# oxideav-mp1

A pure-Rust **MPEG-1 / MPEG-2 LSF Audio** codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.
Decodes and encodes both **Layer I** and **Layer II** (MP2) frames,
across the MPEG-1 and MPEG-2 LSF (Lower Sampling Frequencies) editions.

## Status

Clean-room implementation. Every numeric table is read only from
ISO/IEC 11172-3 (1993) — the MPEG-1 audio standard with Annex B — and
from ISO/IEC 13818-3 (1997) §2.4.2.3 for the MPEG-2 LSF redefinitions.

## What works today

**Decode** — Layer I and Layer II frames to interleaved S16 PCM, MPEG-1
and MPEG-2 LSF:

- **Frame header** (§2.4.1.3 / §2.4.2.3): all thirteen fields decoded
  into a typed `FrameHeader`. The `ID` bit selects the edition — MPEG-1
  uses the 32–448 kbit/s Layer I ladder and 32 / 44.1 / 48 kHz sampling
  table; LSF uses the §2.4.2.3 Layer I ladder and the 16 / 22.05 / 24 kHz
  table. Forbidden and reserved values are rejected; free format is
  recognized.
- **Frame sync and sizing** (§2.4.3.1 / §2.4.2.1), plus a free-format
  frame-length probe that recovers `N` from the distance between
  consecutive syncwords when `bitrate_index == 0`.
- **CRC-16 verification** (§2.4.1.4 / §2.4.3.1) over the Annex B Table
  3-B.5 protected fields, with selectable concealment on mismatch
  (`ConcealmentMode::Mute` or `RepeatPrevious`).
- **Layer I audio data** (§2.4.1.5 / §2.4.3.2): bit allocation,
  scalefactors, requantization, and all four channel modes (mono,
  stereo, dual-channel, joint-stereo intensity sharing).
- **Layer II audio data** (§2.4.1.6 / §2.4.3.3): the B.2x bit-allocation
  tables, scfsi schedules, grouped/separable sample decode, and the
  13818-3 Annex B Table B.1 LSF allocation table for the LSF rates.
- **Polyphase synthesis filterbank** (§2.4.3.2, Annex A): the 32→64
  matrixing, the 512-tap Table 3-B.3 window, and cross-frame overlap-add
  — 384 PCM samples/channel per Layer I frame, 1152 per Layer II frame.

**Encode** — Layer I and Layer II:

- **Layer I encoder**: header writer, per-sample quantization, and an
  iterative bit allocator, with optional CRC and free-format support.
- **Layer II encoder**: the §C.1.3 polyphase analysis filterbank,
  per-part scalefactor extraction, the §C.1.5.2.5 / Table C.4 perceptual
  SCFSI selection (classifies the two successive scalefactor-index
  differences, looks up the transmission pattern, and collapses the
  three scalefactors to one/two where the pattern allows), the
  §C.1.5.2.7 bit allocator, the §2.4.3.3.4 quantizer, the four §2.4.1.6
  region writers, and a top-level `encode_layer2_frame` / stateful
  `Mp1Layer2FrameEncoder`, with optional CRC and §2.4.1.8 ancillary-data
  emission.

The Layer I encoder can drive bit allocation from the **Annex D
Psychoacoustic Model 2** — set `EncodeParams::with_psychoacoustic(true)`
and the `Mp1FrameEncoder` runs the full clause-D.2.4 per-frame procedure
each frame (assembled in the `model2` module): a 1024-point Hann-windowed
FFT, the two-block unpredictability/tonality estimate, per-partition
energy accumulation, spreading-function convolution and renormalization,
the required-SNR / power-ratio / energy-threshold chain, and the
per-FFT-line audibility threshold, collapsed onto the 32 coder
partitions (Table D.5) to per-subband signal-to-mask ratios `SMR_n`.
`allocate_bits_psy` then runs the §C.1.5.1.6 iterative allocator on the
SMR (`MNR = SNR(nb) − SMR`): masked subbands draw no bits and the most
perceptually exposed subbands are served first. The model is honoured at
32 / 44,1 / 48 kHz (the rates with Annex D tables); the MPEG-2 LSF
half-rates fall back to the signal-energy proxy. The default allocator
(`psychoacoustic = false`) remains the non-psychoacoustic energy proxy
for byte-for-byte compatibility.

**Every** numeric table in Annex D — the Layer I and Layer II
threshold-in-quiet families (D.1a–c / D.1d–f), the critical-band tables
(D.2a–f), the Model 2 calculation-partition tables (D.3a–c), the Model 2
per-line absolute-threshold tables (D.4a–c) and the coder-partition table
(D.5) — is now transcribed in the `psy` module from the staged ISO text
extractions / 400-DPI renders. The Layer II LTq tables D.1d–f sit on the
finer 1024-point FFT-line grid (132/130/126 rows vs the Layer I
108/106/102) and feed the Layer II Model 2 path.

## API

The crate exposes both the registry path
(`oxideav_core::register!("mp1", register)`, installed under WAVE format
tag `0x0050` and Matroska codec id `A_MPEG/L1`) and direct factory
endpoints: `decoder::make_decoder`, `decoder::make_decoder_with_concealment`,
`encoder::make_encoder`, `encoder::make_encoder_with_crc`, and
`encoder::make_encoder_layer2`. The Layer I vs Layer II encode branch is
also selectable via `EncodeParams::with_layer(LayerSelect)`.

The Annex D Model 2 psychoacoustic allocator is wired into **both**
encode layers. For Layer II, `encode_layer2_frame_psy` drives the
§C.1.5.2.7 allocator from per-subband SMR (`MNR = SNR(nlevels) − SMR`),
and `Mp1Layer2FrameEncoder::with_psychoacoustic(true)` maintains a
per-channel 1024-sample sliding FFT window, runs the per-channel
`Model2State` each 1152-sample frame, and allocates against the resulting
SMR. As with Layer I, the model is honoured at 32 / 44.1 / 48 kHz and the
MPEG-2 LSF half-rates fall back to the signal-energy proxy.

**Layer II intensity_stereo (joint_stereo)** codes one shared sample
stream in the upper band `[bound, sblimit)` that the decoder rescales
into both channels with each channel's own Table 3-B.1 scalefactor
(§2.4.1.6 / Annex B). The encoder forms that stream from the per-slot
channel average `(L+R)/2`, so both channels' content survives into the
coded samplecode while the per-channel scalefactors restore each
channel's loudness; channel 0's upper-band scalefactor is selected from
the combined peak (`|(L+R)/2| ≤ max(|L|,|R|)`) so the shared samplecode
never overflows quantization.

**Per-frame variable bitrate (VBR)** is available for Layer II via
`Mp1Layer2FrameEncoder::with_vbr(target_mnr_db)` (alongside
`with_psychoacoustic(true)`): each frame's `bitrate_index` is chosen as
the smallest §2.4.2.3 ladder rung whose Model 2 allocation clears the
target mask-to-noise margin, falling back to the top rung when none does.
`select_layer2_vbr_bitrate`, `layer2_min_mnr_db` and
`layer2_bitrate_ladder` expose the selector's building blocks.

## Not yet supported

- **Annex D Model 1.** Only Model 2 is assembled into a per-frame
  driver; the Model 1 masking-index / masking-function closed forms are
  staged in `psy` but there is no Model 1 frame driver (Model 2 is the
  more capable of the two example models).
- **Model 2 pre-echo control** (clause D.2.4 m) is Layer III-only and
  intentionally omitted for Layers I / II.

## Robustness

A `cargo-fuzz` harness under `fuzz/` (a self-contained sub-crate) drives
attacker-controlled bytes through the registered decoder trait object
across both the Layer I and Layer II decode chains and the concealment
paths; the contract under test is panic-freedom on arbitrary input.

## License

MIT — see [LICENSE](./LICENSE).
