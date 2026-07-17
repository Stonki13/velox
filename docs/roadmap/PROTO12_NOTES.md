# PROTO12 - Raycast Vehicle: Review Status

## Status

Implemented on `proto/vehicle`: `include/velox/vehicle.h` + `src/vehicle.cpp`
in the library, verified by `examples/vehicle_demo.cpp` (ctest
`velox.vehicle`).

## Design

- The chassis is a normal dynamic box Body created by the `Vehicle`
  constructor; wheels are virtual (no wheel bodies). Each frame,
  `Vehicle::Step(dt)` — called BEFORE `World::step` so the forces enter that
  step — raycasts each wheel from the hub along the suspension axis
  (ignoring the chassis), then applies:
  - **Suspension**: spring + damper on the clamped compression, force at the
    hub. Compression rate is `dot(hubVelocity, down)` (hub closing on the
    ground compresses).
  - **Tires**: a simplified Pacejka-style slip curve (rise to peak at a
    configured slip, slight fall-off beyond) for longitudinal slip ratio and
    lateral slip angle, scaled by the live suspension load, clamped by a
    combined friction circle, applied at the contact point.
  - **Wheel spin**: per-wheel angular state integrates drive torque, brake
    torque, ground reaction, plus a grounded relaxation toward rolling.
  - **Drivetrain**: engine torque tapering toward redline, through the
    current gear and final drive, split across driven wheels (RWD/FWD/AWD).
    RPM follows the driven wheels; automatic up/down shifts at configured
    RPM with a 0.5 s cooldown against hunting.
  - **Anti-roll bars**: per axle pair, adds suspension force on the more
    compressed side and unloads the other, righting the body.
- `WheelState` exposes per-wheel telemetry (grounded, compression, load,
  spin, contact) for rendering and tuning.

## Spec deviations

- `Vehicle(world, config, position)` creates the chassis itself and
  `AddWheel(config)` attaches to it (the spec's `AddWheel(config, chassisBody)`
  contradicted its own constructor). `AddDefaultWheels()` builds the standard
  4-wheel arrangement with steerable fronts and drivetrain-driven flags.
- `Step` must run BEFORE `World::step` (forces are cleared at the end of a
  step, so applying them after would drop them) — the spec said after.
- The friction curve is the documented game-tunable simplification rather
  than full Pacejka coefficients, per the spec's own risk note.

## Bugs caught by the demo during bring-up

- Damper sign was inverted (pumped energy; the car shook itself over).
- Anti-roll force sign was inverted (amplified roll instead of righting).
Both are the kind of sign error the acceptance scenarios exist to catch.

## Verification (`vehicle_demo`)

- Static hang: all four wheels grounded, suspension carries the full
  1500 kg within 15%, each wheel within 30% of a quarter load, no drift.
- Acceleration: 12.1 m/s at 2 s, 23.8 m/s at 5 s, third gear, RPM tracking.
- Braking: 30 m/s to rest in 38.2 m (< 50 m spec bound).
- Cornering at ~18 m/s, 0.30 rad steering: heading changes 2.68 rad; body
  roll 0.186 rad with anti-roll vs 0.391 rad without (52% reduction, spec
  requires >= 30%).
- Full suite green alongside: stress, fuzz, soak, geometry_fuzz, character,
  sandbox, difftest, islands, vehicle; `fuzz_demo 80`; `proto_manifold`.

## Follow-ups

- Sandbox F4 still spawns the jointed "contraption"; wiring the real
  Vehicle with keyboard driving into the sandbox is a natural next step.
- Heightfield/mesh terrain driving works through the generic raycast but has
  no dedicated test yet.

## Merge recommendation

Ready to merge after normal review.
