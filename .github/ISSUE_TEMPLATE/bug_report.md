---
name: Physics bug report
about: Report an incorrect collision, constraint, query, or determinism result
labels: bug
---

## Environment

- Velox version / commit:
- Platform and compiler:
- CPU architecture:
- Backend (`Cpu`, `Cuda`, or `Auto`):
- GPU and driver, if applicable:

## Reproduction

1.
2.
3.

Attach a minimal serialized scene (`serializeWorld()`), a short source repro,
or both whenever possible. Include the timestep, substeps, and any nondefault
CCD, solver, or thread-safety settings.

## Expected Behavior

## Actual Behavior

## Evidence

Include `StepStats`, debug lines, logs, a screenshot/video, and a strict trace
when the issue may be deterministic or cross-platform.
