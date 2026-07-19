# Changelog

All notable changes to Velox are documented here. The project follows
[Semantic Versioning](https://semver.org/) while the major version is zero:
breaking changes are allowed, but must be recorded in this file and follow the
deprecation policy where practical.

## [0.1.0] - 2026-07-19

### Added

- Public version constants and `velox::versionString()`.
- `VELOX_DEPRECATED(message)` for source-compatible API transitions.
- Opt-in strict CPU determinism mode and cross-platform trace CI.

### Changed

- The public version header is the source of the CMake package version.
