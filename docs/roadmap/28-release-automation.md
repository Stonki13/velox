# 28 — Release Automation

## Goal

Automate the creation of prebuilt binaries and package them for distribution via vcpkg and Conan. This eliminates manual build steps for users and ensures that every release tag produces consistent, reproducible artifacts.

## Public API

```cpp
// No new public symbols. This item adds build system automation, packaging
// scripts, and version management infrastructure.

// Version extraction macro (used in CMake to read version from version.h):
// VELOX_VERSION_MAJOR, VELOX_VERSION_MINOR, VELOX_VERSION_PATCH
// These are set by cmake/version.cmake at configure time.

```

## Data structures

- No new library data structures. All changes are in build system files (cmake/, scripts/).

## Algorithm

**Prebuilt binary generation:**

1. **Build matrix:** For each release tag, build Velox on:
   - Windows x86_64 (MSVC 2022, Release + Debug)
   - Linux x86_64 (GCC 12, Clang 17, Release + Debug)
   - macOS ARM (Clang from Xcode 15, Release)
   - Each with and without CUDA support (if an NVIDIA GPU is available in the runner)
2. **Artifact packaging:** For each platform/build combination, produce:
   - `velox-{version}-{platform}-{buildtype}.zip` containing:
     - `include/velox/*.h` — public headers
     - `lib/libvelox.a` or `lib/velox.lib` — static library
     - `bin/velox.dll` or `libvelox.so` — shared library (if built)
     - `cmake/velox-config.cmake` — CMake package config for `find_package(velox)`
3. **Upload to GitHub Releases:** Attach the zip files as release assets. Tag each build with the version number (e.g., `v0.9.0-windows-x64-release`).

**vcpkg integration:**

1. Create a vcpkg portfile (`ports/velox/portfile.cmake`) that:
   - Downloads the prebuilt binary from GitHub Releases.
   - Extracts it to the vcpkg install directory.
   - Installs headers, libraries, and CMake config files.
2. Submit the port to the upstream vcpkg repository for inclusion.

**Conan integration:**

1. Create a Conan recipe (`conanfile.py`) that:
   - Defines the package name (`velox`), version, and description.
   - Downloads prebuilt binaries from GitHub Releases based on the target platform.
   - Generates a Conan package with headers, libraries, and CMake targets.
2. Publish to Conan Center Index.

**Reproducibility:**

1. Pin all dependency versions in `cmake/dependencies.cmake` (LZ4, ZSTD, etc.).
2. Use deterministic build flags (`-DNDEBUG`, `-O3`, specific SIMD flags) across all platforms.
3. Store build logs and artifact hashes in a release manifest for auditability.

## Files

- `cmake/version.cmake` — read version from `include/velox/version.h` and set CMake variables
- `cmake/velox-config.cmake.in` — CMake package config template
- `scripts/build_release.py` — automation script that builds all platforms and uploads artifacts
- `ports/velox/portfile.cmake` — vcpkg portfile (for upstream submission)
- `conanfile.py` — Conan recipe (for upstream submission)
- `.github/workflows/release.yml` — release automation workflow

## Tests

1. **Release build:** Run `scripts/build_release.py` for a test version (e.g., `v0.9.1-test`). Verify that zip files are produced for all platforms and contain the expected files.
2. **vcpkg install:** Install Velox via vcpkg from the test release; verify that `find_package(velox)` works in a consumer project.
3. **Conan install:** Install Velox via Conan from the test release; verify that the package is available and links correctly.
4. **Version consistency:** The version in `include/velox/version.h` matches the GitHub release tag and the version reported by `velox::versionString()`.

## Acceptance

- [ ] Prebuilt binaries are produced for Windows, Linux, and macOS on every release tag
- [ ] vcpkg portfile installs Velox correctly via `vcpkg install velox`
- [ ] Conan recipe publishes a working package via `conan install velox/0.9.1@`
- [ ] CMake `find_package(velox)` works with the prebuilt binaries
- [ ] Release manifest includes artifact hashes for auditability

## Size: M

## Risks

- Prebuilt binaries increase the attack surface; users must trust the build process. Sign artifacts with GPG keys and publish signatures alongside the releases.
- vcpkg and Conan have different package naming conventions and versioning schemes. The portfile and recipe must be maintained separately; a change in one does not automatically update the other.
- Building on macOS ARM requires access to Apple Silicon CI runners, which may have limited availability or higher costs than x86_64 runners. Consider using GitHub's hosted macOS runners (limited minutes per month) or self-hosted Mac minis.
