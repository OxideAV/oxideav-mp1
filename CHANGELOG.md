# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.3](https://github.com/OxideAV/oxideav-mp1/releases/tag/v0.0.3) - 2026-04-17

### Other

- add GitHub Actions (tests + release-plz auto-publish)
- make crate standalone (pin deps to crates.io, update README + LICENSE)
- add publish metadata (readme/homepage/keywords/categories)
- cargo fmt across the workspace
- add Layer I encoder, completing the MPEG-1 Audio encode trio
- codec todos: mp3↔mp1 table dedup + speex postfilter + mpeg4video polish
- complete decoder with bit allocation + scalefactors + polyphase synthesis
- harvest 5 more agent scaffolds (g7231, g728, g729, gsm, speex)
- harvest 5 rate-limited agent worktrees: mp1/mp2/mp3/aac/celt scaffolds
