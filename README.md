# oxideav-mp1

Pure-Rust **MPEG-1 Audio Layer I** codec — decoder + encoder. Covers
every bitrate (32–448 kbit/s) and every sample rate (32 / 44.1 / 48
kHz) the Layer-I spec defines. Zero C dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Installation

```toml
[dependencies]
oxideav-core = "0.0"
oxideav-codec = "0.0"
oxideav-mp1 = "0.0"
```

## Quick use

MP1 frames are self-contained — each input packet is one MPEG-1 Layer I
frame (384 samples × channels). Output is interleaved S16 PCM.

```rust
use oxideav_codec::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

let mut reg = CodecRegistry::new();
oxideav_mp1::register(&mut reg);

// The decoder reads sample_rate / channels from the frame header, so
// params can be minimal.
let params = CodecParameters::audio(CodecId::new("mp1"));

let mut dec = reg.make_decoder(&params)?;
dec.send_packet(&Packet::new(0, TimeBase::new(1, 44_100), mp1_frame))?;
let Frame::Audio(a) = dec.receive_frame()? else { unreachable!() };
// `a.data[0]` is 384 * a.channels S16 samples at a.sample_rate Hz.
# Ok::<(), oxideav_core::Error>(())
```

Encoder:

```rust
let mut params = CodecParameters::audio(CodecId::new("mp1"));
params.sample_rate = Some(44_100);
params.channels = Some(2);
params.bit_rate = Some(192_000);            // bitrate-to-emit
let mut enc = reg.make_encoder(&params)?;
enc.send_frame(&Frame::Audio(pcm_frame))?;  // 384 × 2 S16 samples
let pkt = enc.receive_packet()?;            // one Layer-I frame
```

### Codec IDs

- `"mp1"` (alias: `"mpa1"`)

## License

MIT — see [LICENSE](LICENSE).
