# Packaging And Releases

Velox installs a standard CMake package named `Velox`:

```powershell
cmake -S . -B build -DVELOX_ENABLE_CUDA=OFF -DVELOX_BUILD_EXAMPLES=OFF
cmake --build build --config Release
cmake --install build --config Release --prefix install
```

A consumer then uses:

```cmake
find_package(Velox CONFIG REQUIRED)
target_link_libraries(my_game PRIVATE velox::velox)
```

The CI package-consumer smoke test configures and links this exact path after
every CPU platform build. `tests/package_consumer` is kept intentionally small
so it verifies package metadata, public includes, static linking, and a basic
simulation without depending on the source tree's include paths.

## Conan

`conanfile.py` is a Conan 2 source-build recipe. It derives the package version
from `include/velox/version.h`; it does not download opaque prebuilt binaries.

```powershell
conan create . --build=missing -o velox/*:with_cuda=False
```

The `with_cuda` option defaults to off, making the generated package portable
across Intel and AMD CPUs and systems without an NVIDIA toolkit. CUDA packages
require a compatible toolkit on both build and consumer machines.

## vcpkg

The repository root doubles as a vcpkg port: `vcpkg.json` is the manifest and
`portfile.cmake` builds and installs the library, the public headers, and the
CMake package config consumed through `find_package(Velox)`. The port derives
its version from the `vcpkg.json` `version` field, which must be kept in sync
with `kVersionMajor/Minor/Patch` in `include/velox/version.h` (vcpkg cannot read
the header at manifest-parse time).

Velox is currently distributed as an overlay/registry port from this repository;
it is not claimed to be in the public vcpkg registry. Install it from a checkout:

```powershell
vcpkg install velox --overlay-ports=C:\path\to\velox
```

The portfile pins the signed `v1.0.0` source archive with its SHA-512. Update
that hash only after creating and verifying the corresponding signed release
tag. Link it from CMake exactly as with the Conan package:

```cmake
find_package(Velox CONFIG REQUIRED)
target_link_libraries(my_game PRIVATE velox::velox)
```

Notes:

- Static vs shared is selected by the active triplet's `VCPKG_LIBRARY_LINKAGE`;
  `vcpkg_cmake_configure` forwards `BUILD_SHARED_LIBS` and the build defines
  `VELOX_BUILDING_SHARED` / `VELOX_USING_SHARED` from the resolved target type,
  so both linkages export and import symbols correctly.
- The optional CUDA backend is opt-in via the `cuda` feature:
  `vcpkg install velox[cuda]`. As with Conan, the default is CPU-only so the
  package stays portable across Intel and AMD machines without an NVIDIA toolkit.

## Tagged Releases

The `Release` workflow runs for `v*` tags or manual dispatch. It creates
CPU-only Windows x64, Linux x64, macOS x64, and macOS arm64 install trees,
archives each one, produces SHA-256 manifests, and attaches the artifacts to
the GitHub release. Publishing to Conan Center or the public vcpkg registry
remains a separate maintainer action after a tagged artifact passes this
pipeline.
