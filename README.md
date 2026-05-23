# oxideav-mp1

Pure-Rust **MPEG-1 Audio Layer I** codec — decoder + encoder. Zero C
dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-codec = "0.1"
oxideav-mp1 = "0.0"
```

## Quick use

MP1 frames are self-contained — each input packet is one MPEG-1 Layer I
frame (384 samples per channel). Decoder output is interleaved S16 PCM.

```rust
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, RuntimeContext, TimeBase};

let mut ctx = RuntimeContext::new();
oxideav_mp1::register(&mut ctx);

// The decoder reads sample_rate / channels from the frame header, so
// params can be minimal.
let params = CodecParameters::audio(CodecId::new("mp1"));

let mut dec = ctx.codecs.make_decoder(&params)?;
dec.send_packet(&Packet::new(0, TimeBase::new(1, 44_100), mp1_frame))?;
let Frame::Audio(a) = dec.receive_frame()? else { unreachable!() };
// `a.data[0]` is 384 * a.channels S16 samples at a.sample_rate Hz.
# Ok::<(), oxideav_core::Error>(())
```

Encoder (CBR, takes interleaved S16 PCM, emits one Layer I frame per
384 samples per channel):

```rust
let mut params = CodecParameters::audio(CodecId::new("mp1"));
params.sample_rate = Some(44_100);
params.channels = Some(2);
params.bit_rate = Some(192_000);            // bits per second
let mut enc = reg.make_encoder(&params)?;
enc.send_frame(&Frame::Audio(pcm_frame))?;  // 384 * 2 S16 samples
let pkt = enc.receive_packet()?;            // one Layer-I frame
```

VBR mode is selected by setting either `vbr_target_kbps` (the average
target bitrate, in kbps) or `vbr_quality` (0..=9, lower = stricter
masking). The per-frame `bitrate_index` then floats over the standard
Layer I ladder (32..=448 kbps); the long-term average converges on
the target via a rolling-average controller.

```rust
use oxideav_core::options::CodecOptions;

let mut params = CodecParameters::audio(CodecId::new("mp1"));
params.sample_rate = Some(44_100);
params.channels = Some(2);
params.options = CodecOptions::new()
    .set("vbr_target_kbps", "192")          // average target
    .set("vbr_quality", "3");               // mask strictness 0..=9
let mut enc = reg.make_encoder(&params)?;
```

CRC-16 protection (§2.4.3.1) is opt-in. Setting `crc_check` clears the
`protection_bit` and inserts the 16-bit CRC word after each header; the
decoder then verifies it and rejects corrupted frames:

```rust
params.options = CodecOptions::new().set("crc_check", "true");
```

Joint stereo (§2.4.2.3 / §2.4.1.5) is opt-in for stereo input. Setting
`joint_stereo` emits `mode = 01` and shares the upper subbands
`[bound..32)` between the two channels — one allocation + one quantised
sample stream (the per-sample mid `M = (L + R) / 2`), but a scalefactor
per channel, so the decoder reconstructs `L = R = M` above the bound.
Layer I has no intensity scaling, so this M/S-style mid sharing is the
full extent of Layer I joint stereo. `js_bound` selects the bound
(`4`/`8`/`12`/`16`, mapping to `mode_extension` 0/1/2/3; default 8). The
freed upper-band bits are redistributed to the loudest below-bound bands:

```rust
params.options = CodecOptions::new()
    .set("joint_stereo", "true")
    .set("js_bound", "8");                   // bound 8 → mode_extension 1
```

### Supported features

Decoder:

- MPEG-1 only (MPEG-2 LSF / 2.5 rejected).
- Sample rates: 32 000, 44 100, 48 000 Hz.
- Bitrates: 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384,
  416, 448 kbit/s (every Layer I index 1..=14). Free format (index 0)
  is not supported.
- Channel modes: single-channel (mono), stereo, dual-channel,
  joint-stereo (all four `mode_extension` bounds: 4 / 8 / 12 / 16).
  Layer I joint stereo is M/S-style sample sharing above the bound;
  there is no intensity-stereo scaling in Layer I.
- Header fields parsed: `protection_bit`, `padding`, `private`,
  `mode`, `mode_extension`, `copyright`, `original`, `emphasis`. When
  `protection_bit = 0`, the CRC-16 word is verified against the
  header tail + bit-allocation field (§2.4.3.1); a mismatch rejects
  the frame.

Encoder (CBR + VBR):

- MPEG-1 only.
- Sample rates: 32 000, 44 100, 48 000 Hz.
- Bitrates: every Layer I rate from 32 to 448 kbit/s (the 14 values
  above).
- Channel modes: mono (single-channel), plain stereo, or joint stereo
  (`joint_stereo` option). Joint stereo shares the upper subbands
  `[bound..32)` as one allocation + one quantised sample stream (the
  per-sample mid `M = (L + R) / 2`) with a per-channel scalefactor, so
  the decoder reconstructs `L = R = M` above the bound; `js_bound`
  picks 4/8/12/16 (`mode_extension` 0/1/2/3, §2.4.2.3). Layer I has no
  intensity scaling, so this M/S-style mid sharing is the full extent
  of Layer I joint stereo. ffmpeg's MPEG-audio decoder accepts our
  joint-stereo streams and recovers the tone cleanly. No dual-channel
  output.
- Optional CRC-16 protection (§2.4.3.1). Off by default
  (`protection_bit = 1`); set the `crc_check` option to clear the
  protection bit and insert the 16-bit CRC word after the header
  (polynomial `x^16 + x^15 + x^2 + 1`, init `0xFFFF`, protecting the
  header tail + the bit-allocation field per Table 3-B.5). The decoder
  verifies the word and rejects frames whose protected field is
  corrupted. ffmpeg's MPEG-audio decoder accepts our CRC streams and
  flags only deliberate corruptions, confirming wire-compatibility.
- CBR allocator: greedy energy-per-bit heuristic — no psymodel.
  Adequate for high bitrates / test signals; not competitive with a
  proper masked-noise allocator at low bitrates.
- VBR allocator: per-subband masking model (energy-based threshold
  with one-tap inter-band spreading + ATH boost above 12 kHz, see
  `psy.rs`) drives a two-phase iteration — Phase 1 fills until every
  band's quantisation noise sits below its mask threshold, Phase 2
  spends any remaining headroom up to a quality-scaled fraction of
  the rolling per-frame target. Per-frame `bitrate_index` is picked
  from the standard ladder; the long-term average converges on
  `vbr_target_kbps`.
- `private`, `copyright`, `original`, `emphasis` are all emitted as 0.

### Codec IDs

- `"mp1"` (registered under this ID; aliases are handled by the
  aggregator crate).

## License

MIT — see [LICENSE](LICENSE).
