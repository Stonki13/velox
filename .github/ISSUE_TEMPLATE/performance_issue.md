---
name: Performance regression
about: Report a measurable simulation, query, or build-performance regression
labels: performance
---

## Environment

- Velox version / commit:
- Platform, compiler, CPU, RAM:
- Backend and GPU driver:

## Workload

Describe body count, shape mix, contacts, joints, timestep, substeps, and
whether the scene is deterministic. Attach a serialized scene or source repro.

## Measurements

Paste `benchmark.exe` output and relevant `StepStats` phase timings. Include
the prior baseline, current result, sample count, and profiler evidence.

## Expected And Actual Performance
