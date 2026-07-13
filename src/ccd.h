#pragma once
#include "velox/body.h"

namespace velox {

// Exact swept-collision primitives over a displacement (the safety net of
// Predictive Contact Sweeping). Each returns the time of impact in [0, 1] of
// the displacement, or a negative value if there is no impact.
float sweepSpherePlane(const Vec3& p0, const Vec3& disp, float radius,
                       const Vec3& planeNormal, float planeOffset);
float sweepSphereSphere(const Vec3& pa, const Vec3& dispA, float ra,
                        const Vec3& pb, const Vec3& dispB, float rb);

} // namespace velox
