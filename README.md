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

The encoder uses a non-psychoacoustic signal-energy SMR proxy; the
Annex D psychoacoustic models (Model 1 / Model 2) are partially staged
in the `psy` module but not yet wired into bit allocation. Staged so
far: the §D.2 critical-band tables (D.2a–f), the Step 6 masking-index /
spreading-function closed forms, the Step 7 global-threshold sum, the
Model 2 partition-domain spreading operator at 32 kHz (Table D.3a), the
complete 108-row Table D.1a threshold-in-quiet table for Layer I at
32 kHz, and — this round — the **complete 106-row Table D.1b
threshold-in-quiet table** for Layer I at 44,1 kHz.

## API

The crate exposes both the registry path
(`oxideav_core::register!("mp1", register)`, installed under WAVE format
tag `0x0050` and Matroska codec id `A_MPEG/L1`) and direct factory
endpoints: `decoder::make_decoder`, `decoder::make_decoder_with_concealment`,
`encoder::make_encoder`, `encoder::make_encoder_with_crc`, and
`encoder::make_encoder_layer2`. The Layer I vs Layer II encode branch is
also selectable via `EncodeParams::with_layer(LayerSelect)`.

## Not yet supported

- Psychoacoustic-model-driven encode quality. The §C.1.5.2.7 allocator
  uses a signal-energy SMR proxy; the Annex D psychoacoustic models
  (Model 1 / Model 2) are partially staged in `psy` but not yet wired
  into bit allocation. Remaining table gaps before the model can be
  wired: the 48 kHz Layer I threshold-in-quiet table (D.1c) and the
  Layer II variants (D.1d–f), the Model 2 partition tables at 44,1 /
  48 kHz (D.3b/c), and the per-line absolute-threshold tables D.4a–c —
  all now staged as CSVs under `docs/audio/mp3/` awaiting transcription.

## Robustness

A `cargo-fuzz` harness under `fuzz/` (a self-contained sub-crate) drives
attacker-controlled bytes through the registered decoder trait object
across both the Layer I and Layer II decode chains and the concealment
paths; the contract under test is panic-freedom on arbitrary input.

## License

MIT — see [LICENSE](./LICENSE).
