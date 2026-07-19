# Deprecation Policy

Velox is pre-1.0 software. Source compatibility is a goal, but the ABI is not
stable until a 1.0 release explicitly says otherwise.

Public APIs scheduled for replacement are marked with
`VELOX_DEPRECATED("replacement guidance")`. Deprecated behavior remains
functional for at least two minor releases while the major version is zero;
removal is recorded in `CHANGELOG.md`. New public struct fields are appended
only, and virtual interface methods are appended only, during a minor series.

Consumers should avoid relying on the binary layout of public structs, private
headers, or `Backend` implementation details across releases. Version checks
should use `velox::kVersionMajor`, `kVersionMinor`, `kVersionPatch`, or
`versionString()`.
