# Runtime Collider Mutation Notes

## Primitive Transaction

The first implementation supports sphere, box, capsule, cylinder, and cone
replacement plus valid primitive scaling. It keeps the body's stable handle,
transform, motion type, and joints. Unless requested otherwise it retains the
body mass and recomputes analytical principal inertia for the replacement
geometry. Mutation removes contacts involving the body, wakes it, marks the
broad phase touched, and invalidates backend caches.

`setCollisionMargin()` updates the existing per-body CCD margin through the
same mutation boundary.

## Remaining Geometry

Hull and compound payloads remain explicitly rejected. Their implementation
must append and rebase new hull/child ranges in `MeshSoup` as one strong
exception-safe transaction; it cannot mutate old ranges because serialized
snapshots and active GPU uploads may still reference them. This roadmap item
is therefore in progress, not ready to merge as complete.

## Verification

`runtime_mutation_demo` covers sphere-to-box replacement, anisotropic box
scale with inertia refresh, hinge preservation, collision-margin update, and
a next-frame overlap query proving the broad-phase proxy was refreshed.
CPU-only Release CTest passed 14/14 and the CUDA-enabled Release target built
and passed the mutation demo.
