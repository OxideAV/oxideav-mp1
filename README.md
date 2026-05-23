# oxideav-mp1

A pure-Rust **MPEG-1 Audio Layer I** (MP1) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Orphan-rebuild scaffold (2026-05-24).** The prior implementation was
retired under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/IMPLEMENTOR_ROUND.md):
the provenance of its 512-tap synthesis-window data table could not be
defended as clean-room — the module doc-comment recorded that the
values had been transcribed from an external library's source file
rather than read solely from the ISO/IEC specification, which violates
the clean-room provenance requirement. Master history was fully erased
per the Hat-3 cold-enforcement procedure.

The implementation will be re-built from scratch against the staged
ISO/IEC 11172-3 Layer I specification (numeric tables read only from
the standard) in a future clean-room round.

## License

MIT — see [LICENSE](./LICENSE).
