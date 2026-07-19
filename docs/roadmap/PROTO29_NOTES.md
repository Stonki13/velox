# Roadmap 29: Community Infrastructure

## Delivered

- Added GitHub issue templates for bug reports, performance reports, and
  feature requests. The bug template explicitly asks for an optional
  serialized repro scene.
- Added `docs/CONTRIBUTING.md` with project code style, CPU/CUDA test, benchmark,
  and review expectations.
- Added five self-contained, CPU-backed examples and registered them as CTest
  smoke tests: `hello_velox`, `contact_events`, `joints_demo`, `queries_demo`,
  and `multi_threaded`.
- The joint example creates one isolated assembly for every public joint type
  (ball, distance, spring, hinge, cone-twist, fixed, prismatic, and six-DoF),
  steps the world, and checks handles and query results for validity.

## Design Decisions

- Examples intentionally use the public umbrella include only and do not add
  framework or rendering dependencies.
- Every example returns a non-zero status when its small behavioral assertion
  fails, allowing CTest and the existing examples CI build to catch API drift.
- `queries_demo.cpp` uses the current public `sphereCast` API. The roadmap's
  intended `queries.cpp` name is represented by this established target and
  CTest entry rather than a duplicate source file.

## Verification

- CUDA-enabled Release build: passed.
- CUDA-enabled Release CTest: 21/21 passed, including the sandbox and
  differential-test configurations plus all five example smoke tests.
- CPU-only Release build: passed.
- CPU-only Release CTest: 19/19 passed, including all five example smoke tests.

The examples are compiled as ordinary CMake targets and are directly
registered in `examples/CMakeLists.txt` when `BUILD_TESTING` is enabled.

## Merge Recommendation

Ready to merge.
