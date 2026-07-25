#![no_main]

//! §2.4.1.4 / §2.4.3.1 `error_check()` CRC-16 oracle target, Layer I
//! and Layer II.
//!
//! Property set (all guaranteed by the CRC-16 definition — generator
//! `G(X) = X^16 + X^15 + X^2 + 1`, init `0xFFFF`, over the Annex B
//! Table 3-B.5 protected fields):
//!
//! 1. **Compute/verify round-trip** — planting the word produced by
//!    `compute_crc` (Layer I) / `compute_layer2_crc` (Layer II) into
//!    the `error_check()` slot must verify as [`CrcStatus::Ok`].
//! 2. **Single-bit-error detection** — any CRC with a degree-16
//!    generator detects every single-bit error inside the covered
//!    span. Flipping exactly one of the 16 protected header bits
//!    (header bits 16…31 = `header_bytes[2..4]`, covered for both
//!    layers with a covered-length that does not depend on those
//!    bits) must verify as [`CrcStatus::Mismatch`]. For Layer I the
//!    allocation-field length depends only on the (unchanged, already
//!    parsed) header, so a single-bit flip inside the covered
//!    allocation bits must also be detected.
//! 3. Unprotected frames (`protection_bit == 1`) always report
//!    [`CrcStatus::Absent`] and consume nothing.
//! 4. `verify_layer2_crc` / `verify_crc` never panic on truncated or
//!    arbitrary `after_header` slices (the `None` too-short contract).
//!
//! Spec basis: ISO/IEC 11172-3 §2.4.1.4, §2.4.3.1, Annex B Table
//! 3-B.5; ISO/IEC 13818-3 §2.4.2.3 (LSF header ladders reuse the same
//! error_check() definition).

use libfuzzer_sys::fuzz_target;
use oxideav_mp1::{compute_layer2_crc, verify_layer2_crc, CrcStatus, FrameHeader};

/// Build a syntactically valid, CRC-protected (or not) header.
fn build_header(id: u8, layer: u8, sr: u8, br: u8, mode: u8, ext: u8, protected: bool) -> [u8; 4] {
    let mut raw: u32 = 0xFFF << 20;
    raw |= u32::from(id & 1) << 19;
    let layer_bits = if layer & 1 == 0 { 0b11u32 } else { 0b10u32 };
    raw |= layer_bits << 17;
    raw |= u32::from(!protected) << 16; // '0' = CRC present
    raw |= u32::from((br % 14) + 1) << 12;
    raw |= u32::from(sr % 3) << 10;
    raw |= u32::from(mode & 0b11) << 6;
    raw |= u32::from(ext & 0b11) << 4;
    raw.to_be_bytes()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let (ctl, body) = (data[..6].to_vec(), &data[6..]);
    let layer1 = ctl[0] & 1 == 0;
    let hdr = build_header(ctl[1], ctl[0], ctl[2], ctl[3], ctl[4], ctl[5], true);
    let h = FrameHeader::parse(&hdr).expect("crafted header must parse");
    assert!(h.has_crc());

    // ---- 4. No-panic / too-short probing on arbitrary slices. ----
    if layer1 {
        let _ = h.verify_crc(&hdr, body);
        let _ = h.verify_crc(&hdr, &[]);
    } else {
        let _ = verify_layer2_crc(&h, &hdr, body);
        let _ = verify_layer2_crc(&h, &hdr, &[]);
    }

    // ---- 1. Compute → plant → verify == Ok. ----
    // `body` supplies the allocation(+scfsi) field bytes; computing may
    // legitimately return None (slice too short for the protected span,
    // or a Layer II allocation index with no table entry).
    let computed = if layer1 {
        h.compute_crc(&hdr, body)
    } else {
        compute_layer2_crc(&h, &hdr, body)
    };
    let Some(crc) = computed else { return };

    let mut after = Vec::with_capacity(2 + body.len());
    after.extend_from_slice(&crc.to_be_bytes());
    after.extend_from_slice(body);
    let status = if layer1 {
        h.verify_crc(&hdr, &after)
    } else {
        verify_layer2_crc(&h, &hdr, &after)
    };
    assert!(
        matches!(status, Some(CrcStatus::Ok(v)) if v == crc),
        "planted CRC did not verify: {status:?}"
    );

    // ---- 2a. Single-bit flip in the protected header bits. ----
    // The covered length derives from `h` (already parsed), never from
    // the flipped bytes, so a one-bit change inside header bits 16…31
    // is a single-bit error in the covered span for both layers.
    let flip = body[0] % 16;
    let mut bad_hdr = hdr;
    bad_hdr[2 + (flip / 8) as usize] ^= 1 << (flip % 8);
    let status = if layer1 {
        h.verify_crc(&bad_hdr, &after)
    } else {
        verify_layer2_crc(&h, &bad_hdr, &after)
    };
    assert!(
        matches!(status, Some(CrcStatus::Mismatch { .. })),
        "single-bit header error not detected: {status:?}"
    );

    // ---- 2b. Layer I: single-bit flip inside the covered allocation
    // bits (their count depends only on `h`). Layer II is excluded:
    // there the covered *scfsi* length depends on the allocation
    // content itself, so a flipped allocation bit changes the covered
    // span and the guarantee no longer follows from CRC linearity.
    if layer1 {
        // 4 allocation bits per subband per (non-joint) channel — at
        // least 32 subbands * 4 bits mono; flip within the first 32
        // bits which are always covered.
        let bit = body[1] % 32;
        let mut bad_after = after.clone();
        bad_after[2 + (bit / 8) as usize] ^= 0x80 >> (bit % 8);
        let status = h.verify_crc(&hdr, &bad_after);
        assert!(
            matches!(status, Some(CrcStatus::Mismatch { .. })),
            "single-bit allocation error not detected: {status:?}"
        );
    }

    // ---- 3. Unprotected header reports Absent. ----
    let open_hdr = build_header(ctl[1], ctl[0], ctl[2], ctl[3], ctl[4], ctl[5], false);
    let oh = FrameHeader::parse(&open_hdr).expect("unprotected header must parse");
    let status = if layer1 {
        oh.verify_crc(&open_hdr, body)
    } else {
        verify_layer2_crc(&oh, &open_hdr, body)
    };
    assert!(matches!(status, Some(CrcStatus::Absent)));
});
