# 07 — Documentation

## Goal

Produce a complete, buildable Doxygen API reference from the existing public headers, plus a getting-started guide and a concepts document that explain Velox's design philosophy (Predictive Contact Sweeping, TGS solver, conservative advancement) to new users. The documentation must be source-linked so that every public symbol traces back to its header definition.

## Public API

```cpp
// Doxygen configuration is in docs/Doxyfile (new file). No new C++ symbols;
// this item adds documentation comments to existing headers.

/// Advances the simulation by one fixed timestep.
///
/// @param dt Time step in seconds. Must be > 0 and ≤ 1/30 for stability.
/// @note Thread-safe from a single thread only. See ThreadSafetyPolicy.
/// @see World::saveSnapshot, World::restoreSnapshot
void step(float dt);

/// Cast a ray and return the closest hit.
///
/// @param origin Ray origin in world space.
/// @param dir    Ray direction (must be normalized).
/// @param maxDist Maximum distance along the ray to test.
/// @param filter Collision filtering applied to candidate bodies.
/// @return RayHit with hit=true if a collider was intersected within maxDist.
RayHit rayCast(Vec3 origin, Vec3 dir, float maxDist,
               const QueryFilter& filter = {}) const;

/// Add a sphere body at the given position with the given radius and mass.
///
/// @param position World-space center of the sphere.
/// @param radius   Sphere radius in meters. Must be > 0.
/// @param mass     Mass in kilograms. Use 0 for static bodies.
/// @return Stable BodyId handle. Valid until removeBody() is called.
/// @deprecated Use addSphereV2 for future-proofing (see versioning policy).
VELOX_DEPRECATED("Use addSphereV2")
BodyId addSphere(Vec3 position, float radius, float mass);

```

## Data structures

- Doxygen configuration — `docs/Doxyfile` (new file), standard Doxyfile with PROJECT_NAME = "Velox", EXTRACT_ALL = YES, SOURCE_BROWSER = YES, INLINE_SOURCES = YES.
- Getting started guide — `docs/getting-started.md` (new file).
- Concepts document — `docs/concepts.md` (new file).
- Doc comments added to existing headers — `include/velox/world.h`, `include/velox/body.h`, `include/velox/joint.h`, `include/velox/backend.h` get Doxygen-style `///` comments on every public symbol.

## Algorithm

**Documentation structure:**

1. **Doxyfile configuration.** Standard Doxygen setup: extract all symbols, generate HTML + LaTeX output, link to external references (Bullet, Box2D docs for algorithm comparisons), enable source code browsing with line numbers.
2. **Getting-started guide (`docs/getting-started.md`).** Covers: build instructions (cmake flow), first simulation (add a sphere, step the world, read positions), adding colliders (all shape types), applying forces/impulses, querying (raycast, overlap), handling contact events, basic debugging with debug lines.
3. **Concepts document (`docs/concepts.md`).** Explains: Predictive Contact Sweeping (the three-layer pipeline), TGS solver substeps and live gaps, conservative advancement as a safety net, sleeping/islands, broad-phase AABB tree, joint types and their use cases, CCD quality levels, multi-TOI processing.
4. **API reference.** Doxygen extracts from headers; every public method gets a `@brief`, `@param` tags, `@return`, and `@see` cross-references where applicable.

**Doc comment additions to existing headers:**

For each public symbol in `include/velox/world.h`:
- Add `///` brief description before the declaration
- Add `@param` for each parameter with units and constraints
- Add `@return` describing the return value semantics
- Add `@note` for thread-safety, stability constraints, or known limitations
- Add `@see` cross-references to related methods

## Files

**New files:**
- `docs/Doxyfile` — Doxygen configuration
- `docs/getting-started.md` — step-by-step tutorial
- `docs/concepts.md` — design philosophy and algorithm explanations
- `docs/api-reference/` — generated output directory (not committed; documented in .gitignore)

**Modified files:**
- `include/velox/world.h` — add Doxygen comments to all public methods
- `include/velox/body.h` — add Doxygen comments to Body struct fields and methods
- `include/velox/joint.h` — add Doxygen comments to Joint struct fields
- `include/velox/backend.h` — add Doxygen comments to Backend interface

## Tests

1. **Doxygen builds without errors:** Run `doxygen docs/Doxyfile`; exit code 0; no warnings about undocumented symbols in the public API.
2. **Getting-started compiles:** Copy-paste every code example from `docs/getting-started.md` into a fresh project; all must compile and link against Velox.
3. **Concepts document references are accurate:** Every algorithm name mentioned (Sutherland-Hodgman, GJK, EPA, TGS) matches the actual implementation in src/. Verified by cross-referencing with source code.

## Acceptance

- [ ] Doxygen builds successfully from existing headers with no undocumented public symbols
- [ ] `docs/getting-started.md` contains a working "hello world" example that compiles
- [ ] `docs/concepts.md` explains PCS, TGS solver, conservative advancement, and sleeping/islands
- [ ] Every public method in world.h has a Doxygen brief comment
- [ ] Getting-started examples compile against the current API without modification

## Size: S

## Risks

- Doxygen comments on existing headers are low-risk but high-effort; easy to do incompletely. Must prioritize: every public method gets at least a one-line brief before the full @param/@return treatment.
- The getting-started guide must be kept in sync with API changes. If the API evolves faster than the docs, the guide becomes misleading. Consider linking to the Doxygen reference as the source of truth and keeping the guide minimal.
- Concepts document risks becoming outdated as algorithms are refined (e.g., if Sutherland-Hodgman is replaced by a different clipping method in item 01). Must note when the doc describes current implementation vs. design intent.
