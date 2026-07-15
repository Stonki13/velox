# Roadmap 02 implementation notes

## Status

Implemented and CPU-verified on `proto/manifolds`.

## Changes

- Added the CUDA-safe `GeometryQuality` hint to `Body`. `Normal` keeps the
  production GJK path unchanged; `Robust` and `Paranoid` use additional
  deterministic seed directions and iteration budget.
- Added `World::queryGeometryDiagnostics(BodyId)`, reporting primitive and
  hull edge ranges, aspect ratio, divergence-theorem volume, degeneracy, and
  near-coplanar triangulated hull faces. It is computed on demand so edits
  through `World::body()` cannot leave a stale cache.
- Hull and compound-hull creation now reject points within `1e-8` units of an
  existing point before QuickHull/mass-property construction.
- Replaced fixed GJK/EPA duplicate, near-origin, tetrahedron, face-area, and
  convergence thresholds with pair-scale-aware values. Degenerate GJK
  tetrahedra are reduced to their nearest triangle instead of being reported
  as overlapping.
- EPA filters newly-created faces below `1e-8` of the largest seed-face area.
- Added `geometry_fuzz`: deterministic needle, coplanar, tiny-shape,
  diagnostics, and duplicate-hull regressions plus random metamorphic runs.
  It checks finite outputs, separated-distance/normal symmetry, translation
  invariance at the smallest representable feature scale, and support-plane
  witness residuals.

## Validation limits

The fuzzer spans `1e-4` to `1e3` shape scales. Witness checks use a 0.5%
scale-relative tolerance: a fixed `1e-6` threshold is not meaningful for
single-precision support points at the largest generated scales. EPA
penetration depth is checked for finite stable output rather than exact swap
symmetry because different valid EPA seed polytopes can converge to slightly
different depths on overlapping, high-aspect pairs.

## Remaining follow-up

- A full 30-minute/one-million-pair CI soak should be wired into the project
  CI once CI configuration is added; the local deterministic 100,000-pair
  gate passes.
- Compound diagnostics currently report the conservative bounding-sphere
  volume because an exact compound volume requires boolean-union handling of
  overlapping children.
