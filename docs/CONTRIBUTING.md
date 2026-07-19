# Contributing To Velox

Use a focused branch and keep commits narrowly scoped. Do not weaken or remove
an existing test to make a change pass.

## Development Gate

Configure a CPU build with `cmake -S . -B build -DVELOX_ENABLE_CUDA=OFF`, then
build with `cmake --build build --config Release` and run
`ctest --test-dir build -C Release --output-on-failure`. Changes touching
shared geometry, narrow phase, or solver code also require a CUDA build when a
toolkit is available, plus fuzzing and benchmark comparison.

Use four-space indentation, CamelCase types, and the surrounding naming style.
Code compiled by CUDA must preserve `VELOX_HD` compatibility and avoid host-only
library facilities. New behavior needs a focused regression executable or test.

Benchmark changes against the canonical scenes and explain CPU regressions over
five percent. Include reproduction steps, test results, and affected backend
paths in pull requests. One maintainer review is required before merge.
