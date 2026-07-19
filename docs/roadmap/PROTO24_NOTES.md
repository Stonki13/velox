# PROTO24 - Cross-Platform Determinism: Foundation Status

## Status

The strict CPU-reference foundation is implemented. This is not yet a claim of
bit-identical CPU/CUDA or Windows/Linux/macOS parity: those require execution
on the relevant runners and an ordered CUDA solver, which remains follow-up
work.

## Design

- `VELOX_STRICT_FLOATING_POINT=ON` selects reproducible CPU compiler settings:
  `/fp:precise` on MSVC and `-fno-fast-math -ffp-contract=off` on
  GCC/Clang. The CUDA compile option is also prepared with `--fmad=false`.
- `World::setDeterminismMode(Strict)` is accepted only by a strict build. It
  recreates the backend as the portable CPU reference implementation and
  forces sequential island solving.
- CUDA strict mode is deliberately not advertised. Its graph-colored solver
  changes impulse order, so accepting it would create an unsupported
  bit-identical guarantee. Switching back to `Relaxed` restores the backend
  originally requested by the `World` constructor.
- A normal build rejects strict mode with a clear `logic_error`, preventing a
  runtime API choice from silently bypassing the required compiler flags.

## Verification

Two independent Release configurations passed `velox.determinism`:

| Configuration | Result |
| --- | --- |
| Default floating point, CUDA enabled | Strict mode correctly rejected |
| `VELOX_STRICT_FLOATING_POINT=ON`, CPU only | Auto backend became CPU; parallel-island selection was rejected; 360-frame one-worker and auto-worker replays were bitwise identical |

The regression scene contains a resting 12-box stack with angular motion, so
it covers deterministic broad-phase ordering, contact generation, warm
starting, solving, and sleeping rather than a contact-free trajectory.

`determinism_demo --trace` also records all tracked float fields after every
frame into an explicitly serialized 1000-frame FNV trace. The local MSVC
strict trace is `b7a7ef9fb436ad19`. CI uploads and compares this trace across
Windows, Linux, macOS Intel, and macOS ARM once the workflow runs.

## Remaining Acceptance Work

- Run and compare strict trace artifacts on Windows/MSVC, Linux/GCC,
  Linux/Clang, and macOS/Clang.
- Add an ordered CUDA strict backend, then compare its trace against the CPU
  reference. Until then strict mode intentionally selects CPU.
- Add an assembly-level FMA inspection gate for each supported compiler.
- Audit third-party math functions used by strict scenarios for last-ULP
  platform variation and replace any unstable operation with a controlled
  implementation if the cross-runner artifacts expose one.

## Merge Recommendation

The foundation is ready to merge as an opt-in, accurately scoped API. Do not
claim full cross-platform or CPU/CUDA lockstep support until the remaining
acceptance work passes on CI.
