# Release Automation Notes

## Design

Release packaging builds the portable CPU configuration on Windows, Linux, and
Apple Silicon macOS. Each platform installs Velox to a staging prefix, verifies
that an external CMake project can `find_package(Velox CONFIG REQUIRED)`, then
archives the prefix and writes a SHA-256 sidecar. The publish job attaches the
archives and combined manifest only after every platform package succeeds.

The new Conan 2 recipe builds from the checked-out source and derives its
version from `include/velox/version.h`. It deliberately does not download
unverified prebuilt binaries. CUDA is an explicit, off-by-default option, so
the default package works on Intel and AMD CPU systems without an NVIDIA
toolkit.

## Verification

- CPU-only Release configure, build, install, and external CMake consumer
  configure/build/CTest passed locally.
- The consumer linked `velox::velox`, included only installed headers, created
  a CPU world, stepped it, and validated a body handle.
- Archive smoke test verified the intended top-level `include/` and `lib/`
  layout plus the installed `VeloxConfig.cmake` files.
- `python -m py_compile conanfile.py` passed.

Conan itself is not installed in this workspace, so `conan create` remains a
CI or maintainer-machine verification step. No GitHub tag was created or
published; the release workflow has not run in hosted CI yet.

## Merge Recommendation

Ready for review. This establishes reproducible source and CMake package
verification now; publishing to Conan Center or the vcpkg registry requires
the normal upstream review process after the first tagged artifacts exist.
