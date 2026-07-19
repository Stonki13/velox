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

## Tagged Releases

The `Release` workflow runs only for `v*` tags or manual dispatch. It creates
CPU-only Windows, Linux, and macOS install trees, archives each one, produces
SHA-256 manifests, and attaches both to the GitHub release. Publishing to
Conan Center or the vcpkg registry remains a maintainer action after a tagged
artifact has passed this pipeline.
