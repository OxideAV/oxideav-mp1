# oxideav-mp1

Pure-Rust **MPEG-1 Audio Layer I** codec — decoder + encoder.

Covers every bitrate (32–448 kbit/s) and every sample rate (32 / 44.1
/ 48 kHz) the Layer-I spec defines. Zero C dependencies, no FFI, no
`*-sys` crates.

Originally part of the [oxideav](https://github.com/KarpelesLab/oxideav)
framework; extracted to its own crate for independent publication.

## Usage

```toml
[dependencies]
oxideav-mp1 = "0.0.3"
```

Plugs into [`oxideav-codec`](https://crates.io/crates/oxideav-codec):

```rust
let mut reg = oxideav_codec::CodecRegistry::new();
oxideav_mp1::register(&mut reg);
```

Decoder id: `"mp1"` — input packets are MPEG-1 Layer I frames; output
is interleaved 16-bit PCM. Encoder takes mono or stereo PCM and emits
CBR Layer-I frames with a greedy bit allocator.

## License

MIT — see [LICENSE](LICENSE).
