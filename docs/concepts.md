# Velox Concepts

## Predictive Contact Sweeping

Velox combines speculative contacts with conservative advancement. Broad phase
and narrow phase create contacts before surfaces meet when their relative
motion can close the gap during the step. The iterative solver removes only
the approach velocity that would exceed that gap. If an object still ends a
step interpenetrating, conservative advancement rewinds the pair to a safe
time of impact and resolves it there. This design avoids a global time-of-
impact event queue while protecting high-speed objects from tunneling.

## Contact Manifolds and TGS

Convex face contacts are clipped into persistent manifolds with feature keys.
Those keys reconnect impulses from the previous frame, known as warm starting.
The solver uses temporal Gauss-Seidel substeps: collision detection runs once
per outer step, while each substep recomputes a contact's live separation from
local anchors. Resting stacks converge faster and speculative contacts do not
need to be recreated every substep.

## Broad Phase and Narrow Phase

The CPU backend stores finite shapes in an incremental dynamic AABB tree with
fat proxies. Sleeping or stationary content avoids unnecessary tree updates.
Candidate pairs are sorted into a deterministic order before narrow phase.
GJK calculates distance and witnesses for convex shapes; EPA supplies a
penetration direction for suitable overlapping convex cores. Static meshes use
a triangle BVH and convex-vs-triangle GJK tests.

## Islands, Sleeping, and Joints

Contacts and joints form islands. An island below motion thresholds can sleep,
removing its simulation cost until a new interaction wakes it. Independent
CPU islands may solve concurrently in relaxed mode. Joints include ball,
distance/spring, hinge, cone/twist, fixed, prismatic, and six-degree-of-
freedom constraints with limits and motors.

## CPU, CUDA, and Determinism

The normal CPU backend uses worker threads for eligible work. CUDA uses
graph-colored contacts because independent colors can solve in parallel. That
is fast but has a different constraint order from sequential CPU solving.

Strict determinism is an opt-in CPU reference mode. It disables floating-point
contraction at build time, forces ordered sequential island solving, and has a
trace regression gate. It is not yet a CPU/CUDA lockstep mode.
