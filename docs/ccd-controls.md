# CCD Quality Controls

Every body has `BodyCcdTuning`, configured with `World::setCcdTuning()`.
World defaults apply when a body is created and can be changed with
`World::setCcdDefaults()`.

`MotionQuality::Medium` is the default Predictive Contact Sweeping behavior.
`Low` disables speculative sweep expansion and conservative recovery for that
body, trading tunneling resistance for lower broad-phase work. `Locked` turns
a dynamic body into an immovable collider until it is assigned another quality;
forces, impulses, joints, gravity, and transform advancement cannot accumulate
hidden motion while it is locked.

`collisionMargin` expands only swept broad-phase bounds. It never changes GJK
or contact geometry, so a positive margin cannot create a narrow-phase contact
by itself. `speculativeDistance` adds to the contact-generation reach, and
`minVelocityForCCD` avoids conservative recovery for slow bodies.

`World::queryMultiToi()` uses conservative advancement followed by bisection to
return a deterministic list of candidate impacts sorted by time, then body
handle. High-quality `step()` processing replays static-geometry impacts in
chronological order, with per-body and per-step caps reported as
`StepStats::multiToiEvents`. Dynamic-dynamic chronological rescheduling is
still under development; do not use High mode as a lockstep guarantee for
multiple moving bodies yet.
