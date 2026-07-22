# Real-Workload Release Gate

`release_gate` is a deterministic 3,600-frame CPU reference workload used to
catch failures that isolated unit tests do not expose. It is registered as
`velox.release_gate` and runs with the normal CTest suite.

## Scene

- Static plane and 33 by 33 procedural heightfield.
- Raised triangle-mesh bridge and a five-story box tower.
- Fifty dynamic spheres, boxes, and capsules with masses from 0.5 to 49.5 kg.
  Several begin at 35 m/s to exercise the CCD safety path.
- A six-capsule ball-joint chain, a hinged pendulum, and a spring-mounted
  platform.
- Ray, overlap, and sphere-cast queries at frame 1,800.

## Gate Conditions

The executable fails when a tracked body becomes non-finite or escapes below
the level, when any mid-run query has no hit, or when mean CPU step time across
the complete workload exceeds 10 ms. The 10 ms budget is intentionally an
average rather than a single-frame threshold because it is meaningful on
shared CI runners while still detecting order-of-magnitude regressions.

Run it directly with:

```powershell
build\examples\Release\release_gate.exe
```

The gate complements, rather than replaces, stress, differential, geometry
fuzz, manifold, serialization, character, vehicle, and nightly soak tests.
