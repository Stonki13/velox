#pragma once
#include "velox/body.h"

namespace velox {

// Continuous collision detection primitives.
// Each returns time of impact in [0, 1] of the step, or a negative value if
// there is no impact within the step.
float sweepSpherePlane(const Body& sphere, const Body& plane, float dt);
float sweepSphereSphere(const Body& a, const Body& b, float dt);

} // namespace velox
