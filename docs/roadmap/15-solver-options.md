# 15 — Solver Options

## Goal

Expose per-world and per-contact solver tuning knobs: friction-cone block solving mode, iteration count policies (fixed vs adaptive), and stack-stabilization presets. These let users trade simulation quality for performance on a per-scene basis without modifying the core solver.

## Public API

```cpp
namespace velox {

enum class FrictionModel : uint8_t {
    TwoAxisCoulomb = 0,   // current: two tangent impulses with elliptic clamp
    ConeBlockSolver = 1   // block solve normal + friction as a single cone
};

enum class IterationPolicy : uint8_t {
    Fixed = 0,            // fixed iteration count per substep
    Adaptive = 1          // stop when impulse change < threshold
};

struct SolverOptions {
    FrictionModel frictionModel = FrictionModel::TwoAxisCoulomb;
    IterationPolicy iterationPolicy = IterationPolicy::Fixed;
    int velocityIterations = 8;           // per substep (current default)
    int positionIterations = 4;           // Baumgarte correction passes
    float impulseThreshold = 1e-5f;       // adaptive: stop when max impulse < this
    float stackStiffness = 1000.0f;       // extra stiffness for resting contacts
    float stackDamping = 10.0f;           // damping for stack stabilization
    bool enableStackStabilization = false;
};

struct SolverPreset {
    std::string name;
    SolverOptions options;
};

// Predefined presets:
inline SolverPreset PresetHighQuality() { /* high iterations, cone solver */ }
inline SolverPreset PresetFast()        { /* low iterations, two-axis */ }
inline SolverPreset PresetStacking()    { /* stack stabilization on */ }

} // namespace velox
```

## Data structures

- `FrictionModel`, `IterationPolicy` — new enums in `include/velox/solver.h`.
- `SolverOptions` struct — new file `include/velox/solver.h`.
- `SolverPreset` struct — new file `include/velox/solver.h`.
- `World::solverOptions_` member — added to `include/velox/world.h`.

## Algorithm

**Friction-cone block solving:**

1. Instead of solving normal and two tangents independently, form a 3×3 effective mass matrix for the contact.
2. Solve the full cone constraint: find impulses `(jn, jt1, jt2)` that satisfy `vn_target` and lie within the friction cone `sqrt(jt1² + jt2²) ≤ μ * jn`.
3. Use a projected Gauss-Seidel inner loop: solve the unconstrained system, then project onto the cone if violated.

**Adaptive iteration policy:**

1. After each iteration, compute the maximum absolute impulse applied across all contacts.
2. If `maxImpulse < impulseThreshold`, stop iterating early — further iterations won't change the result meaningfully.
3. This saves work on resting contacts that have already converged while keeping accuracy for dynamic impacts.

**Stack stabilization:**

1. For contacts where both bodies are nearly at rest (velocity < 0.01 m/s), apply extra spring-damper forces proportional to penetration depth.
2. This counteracts the small velocity biases that accumulate from split-impulse positional correction and cause slow drift in tall stacks.

## Files

- `include/velox/solver.h` — new header with SolverOptions, presets
- `src/solver.cpp` — implement friction cone solver path, adaptive iteration loop
- `tests/solver_options.cpp` — test file

## Tests

1. **Cone vs two-axis friction:** Box on inclined plane (30°). Cone solver: box stays at rest up to 45°. Two-axis: slips at ~38° due to tangent decoupling error.
2. **Adaptive iteration savings:** 1000 resting contacts, fixed 8 iterations vs adaptive. Adaptive stops at iteration 2 on average; same final state.
3. **Stack stabilization:** 20-box tower with default solver drifts 5 mm over 10 seconds. With stackStiffness=1000, drift < 0.5 mm.

## Acceptance

- [ ] Friction cone block solver produces more accurate friction response than two-axis
- [ ] Adaptive iteration policy converges to same result as fixed with fewer iterations on average
- [ ] Stack stabilization preset reduces resting drift by ≥ 80%
- [ ] Solver options can be changed at runtime via `World::setSolverOptions()`

## Size: M

## Risks

- Cone block solver is more expensive per iteration (3×3 matrix solve vs scalar). Must benchmark to ensure the accuracy gain justifies the cost.
- Stack stabilization adds artificial forces that may feel "sticky" for sliding objects. Must be opt-in and clearly documented as a stability aid, not a physical model.
