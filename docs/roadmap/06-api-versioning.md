# 06 — API Versioning

## Goal

Establish a semver-based versioning scheme, a CHANGELOG format, and a deprecation policy so that Velox can evolve its public API without breaking existing users. Every public symbol gets a version tag; deprecated symbols are marked with a macro that emits compiler warnings and tracks usage for eventual removal.

## Public API

```cpp
namespace velox {

// Library version constants. Increment major on ABI-incompatible changes,
// minor on new features, patch on bug fixes.
inline constexpr uint32_t kVersionMajor = 0;
inline constexpr uint32_t kVersionMinor = 9;
inline constexpr uint32_t kVersionPatch = 0;

// Human-readable version string: "MAJOR.MINOR.PATCH".
inline const char* versionString() { return "0.9.0"; }

// Deprecation macro: emits a compiler warning when the annotated symbol is used.
// Usage: [VELOX_DEPRECATED("Use World::addSphereV2 instead")] BodyId addSphere(...);
#if defined(_MSC_VER)
#define VELOX_DEPRECATED(msg) __declspec(deprecated(msg))
#elif defined(__GNUC__) || defined(__clang__)
#define VELOX_DEPRECATED(msg) __attribute__((deprecated(msg)))
#else
#define VELOX_DEPRECATED(msg)
#endif

// Minimum compiler version required. Enforced at compile time.
inline constexpr uint32_t kMinCxxStandard = 17;

} // namespace velox
```

## Data structures

- Version constants — live in `include/velox/version.h` (new file).
- `VELOX_DEPRECATED` macro — lives in `include/velox/version.h`, guards with compiler detection.
- CHANGELOG.md — new file at project root, follows Keep a Changelog format.
- DEPRECATION_POLICY.md — new file in docs/, documents the deprecation timeline.

## Algorithm

**Semver rules for Velox:**

1. **Major version (X.y.z):** Increment when removing public symbols, changing struct layouts that affect ABI, or modifying the binary interface of Backend virtual methods. Major = 0 means the API is still evolving; breaking changes are allowed but documented.
2. **Minor version (x.Y.z):** Increment for new public methods, new enum values, new structs. Backward compatible at the source level; existing code compiles and links without changes.
3. **Patch version (x.y.Z):** Increment for bug fixes that don't change behavior observable through the public API. Binary-compatible.

**Deprecation timeline:**

1. A symbol marked `VELOX_DEPRECATED` in minor version N is eligible for removal in major version N+1 (if major > 0) or minor version N+2 (if major = 0).
2. Deprecated symbols remain functional for the entire deprecation window; they emit compiler warnings but do not change behavior.
3. Internal symbols (not in the public header set) are exempt from deprecation markers but follow the same removal timeline in practice.

**CHANGELOG format (Keep a Changelog):**

```markdown
# Changelog

All notable changes to Velox will be documented in this file.

## [0.9.0] — 2026-07-14
### Added
- Persistent contact manifolds with Sutherland-Hodgman clipping
- Multi-TOI CCD for sequential impact processing
- Per-body motion quality flags

### Changed
- Body::shape enum extended with RoundedBox, Ellipsoid (forward compat)

### Deprecated
- World::addSphere(float radius) — use addSphere(Vec3 position, float radius, float mass)

### Removed
- (none yet)
```

**ABI stability checks:**

1. Every struct in `include/velox/*.h` is frozen once shipped in a minor version; new fields are appended at the end with default values.
2. Virtual method signatures on `Backend` cannot change between minor versions; new methods are added to the end of the vtable.
3. `BodyId` and `JointId` layout (uint64_t with slot/generation split) is frozen; internal dense index representation may change.

## Files

**New files:**
- `include/velox/version.h` — version constants, deprecation macro
- `CHANGELOG.md` — Keep a Changelog format history
- `docs/DEPRECATION_POLICY.md` — full deprecation timeline and rules
- `cmake/version.cmake` — CMake snippet to set project version from version.h (new file, not modifying CMakeLists.txt)

**Modified files:**
- None (version.h is a new header; existing public headers don't need changes yet)

## Tests

1. **Deprecation warning emits:** Compile a translation unit that calls a `VELOX_DEPRECATED` symbol; verify the compiler emits exactly one warning with the deprecation message. Test on MSVC, GCC, Clang.
2. **Version string matches version.h:** Build the library and run a test that asserts `velox::versionString()` equals the string literal in version.h. Catches drift between header and actual release tag.
3. **ABI stability across minor versions:** Compile against version 0.9.0 headers, link against a binary built from 0.8.0 sources (simulated via header swap). All struct sizes must match; all vtable offsets must match.

## Acceptance

- [ ] `include/velox/version.h` exists with major/minor/patch constants and `versionString()`
- [ ] `VELOX_DEPRECATED(msg)` macro works on MSVC, GCC, and Clang
- [ ] CHANGELOG.md follows Keep a Changelog format with at least one entry
- [ ] Deprecation policy document exists in docs/
- [ ] No public symbol is removed without being deprecated for ≥ 2 minor versions first

## Size: S

## Risks

- Semver is source-level, not binary-level. A struct layout change that doesn't affect the ABI (adding a field at the end with a default) is technically a minor version bump but can break users who memcpy'd the struct. Must document this explicitly.
- The deprecation macro emits warnings that some users disable globally (`-Wno-deprecated`), rendering the signal useless. Consider a separate `VELOX_DEPRECATED_HARD` that errors instead of warns for symbols past their removal date.
- Version.h must be included by every translation unit that uses Velox types; adding it as a dependency increases compile times marginally. Acceptable tradeoff for the clarity it provides.
