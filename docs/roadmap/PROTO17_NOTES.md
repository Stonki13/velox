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

## Hull And Compound Transaction

Hull and compound payloads are constructed in an isolated CPU World first.
Once validation and mass generation succeed, Velox copies the generated hull
points, face indices, and compound children into a copied `MeshSoup`, rebases
the appended ranges, then commits the soup and body together. Old ranges are
left intact for snapshots and active GPU uploads. The uncommon mutation path
therefore favors strong exception safety over avoiding a soup copy.

## Verification

`runtime_mutation_demo` covers sphere-to-box replacement, anisotropic box
scale with inertia refresh, hinge preservation, collision-margin update, and
a next-frame overlap query proving the broad-phase proxy was refreshed.
CPU-only Release CTest passed 14/14 and the CUDA-enabled Release target built,
ran the mutation demo, and passed the full stress suite.

## Merge Recommendation

Ready for the full CUDA regression gate. The descriptor retains body mass on
inertia refresh because Velox does not persist authoring density separately.
