#include "velox/character.h"

#include <cmath>
#include "velox/error.h"

namespace velox {

namespace {

constexpr Vec3 kUp{0, 1, 0};
constexpr float kMoveEpsilon = 1e-6f;

bool finiteVec3(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

Vec3 rejectNormal(Vec3 value, Vec3 normal) {
    float intoSurface = dot(value, normal);
    return intoSurface < 0.0f ? value - normal * intoSurface : value;
}

} // namespace

CharacterController::CharacterController(World& world, const CharacterControllerDesc& desc)
    : world_(world), desc_(desc) {
    if (!std::isfinite(desc_.capsuleRadius) || desc_.capsuleRadius <= 0.0f ||
        !std::isfinite(desc_.capsuleHalfHeight) || desc_.capsuleHalfHeight < 0.0f ||
        !std::isfinite(desc_.stepMaxHeight) || desc_.stepMaxHeight < 0.0f ||
        !std::isfinite(desc_.slopeLimitCosine) || desc_.slopeLimitCosine < 0.0f ||
        desc_.slopeLimitCosine > 1.0f || !std::isfinite(desc_.movementSpeed) ||
        desc_.movementSpeed < 0.0f || !std::isfinite(desc_.ghostPadding) ||
        desc_.ghostPadding < 0.0f)
        VELOX_THROW(VeloxInvalidArgument, ErrorCode::CharacterInvalidDesc, "invalid character controller description");
}

void CharacterController::SetPosition(Vec3 position) {
    if (!finiteVec3(position))
        VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "character position must be finite");
    position_ = position;
}

bool CharacterController::probeGround(Vec3 position, ShapeCastHit& hit) const {
    const float skin = desc_.ghostPadding;
    const float probeDistance = skin * 2.0f + 1e-4f;
    hit = world_.capsuleCast(position + kUp * skin, desc_.capsuleRadius,
                             desc_.capsuleHalfHeight, {}, -kUp, probeDistance);
    return hit.hit && hit.normal.y >= desc_.slopeLimitCosine;
}

bool CharacterController::tryStep(Vec3& position, Vec3 remaining,
                                  CharacterControllerResult& result) const {
    if (desc_.stepMaxHeight <= kMoveEpsilon) return false;

    Vec3 horizontal{remaining.x, 0.0f, remaining.z};
    float horizontalDistance = length(horizontal);
    if (horizontalDistance <= kMoveEpsilon) return false;

    const float skin = desc_.ghostPadding;
    const float lift = desc_.stepMaxHeight + skin;
    ShapeCastHit ceiling = world_.capsuleCast(position + kUp * skin, desc_.capsuleRadius,
                                               desc_.capsuleHalfHeight, {}, kUp, lift);
    if (ceiling.hit) return false;

    Vec3 raised = position + kUp * desc_.stepMaxHeight;
    Vec3 direction = horizontal * (1.0f / horizontalDistance);
    ShapeCastHit forward = world_.capsuleCast(raised, desc_.capsuleRadius,
                                               desc_.capsuleHalfHeight, {}, direction,
                                               horizontalDistance + skin);
    if (forward.hit && forward.distance < horizontalDistance - skin) return false;

    Vec3 moved = raised + horizontal;
    ShapeCastHit landing = world_.capsuleCast(moved, desc_.capsuleRadius,
                                               desc_.capsuleHalfHeight, {}, -kUp,
                                               desc_.stepMaxHeight + skin * 2.0f + 1e-4f);
    if (!landing.hit || landing.normal.y < desc_.slopeLimitCosine) return false;

    float drop = landing.distance - skin;
    if (drop < 0.0f) drop = 0.0f;
    position = moved - kUp * drop;
    result.stepped = true;
    ++result.contactCount;
    return true;
}

CharacterControllerResult CharacterController::Move(Vec3 displacement) {
    if (!finiteVec3(displacement))
        VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonFiniteValue, "character displacement must be finite");

    CharacterControllerResult result;
    const Vec3 start = position_;
    ShapeCastHit ground;
    const bool initiallyGrounded = probeGround(position_, ground);

    bool jumped = jumpVel_ > 0.0f && initiallyGrounded;
    if (jumped) {
        displacement += kUp * jumpVel_;
        jumpVel_ = 0.0f;
    }
    if (initiallyGrounded && !jumped)
        displacement -= ground.normal * dot(displacement, ground.normal);

    Vec3 remaining = displacement;
    for (int iteration = 0; iteration < 4 && lengthSq(remaining) > kMoveEpsilon * kMoveEpsilon;
         ++iteration) {
        float distance = length(remaining);
        Vec3 direction = remaining * (1.0f / distance);
        // A capsule exactly on a walkable floor is already touching that
        // floor at t=0. Forward queries use the ghost skin so tangential
        // movement does not turn that resting contact into a wall hit.
        Vec3 castPosition = position_;
        if (direction.y >= -kMoveEpsilon) castPosition += kUp * desc_.ghostPadding;
        ShapeCastHit hit = world_.capsuleCast(castPosition, desc_.capsuleRadius,
                                               desc_.capsuleHalfHeight, {}, direction,
                                               distance + desc_.ghostPadding);
        if (!hit.hit || hit.distance >= distance + desc_.ghostPadding) {
            position_ += remaining;
            remaining = {};
            break;
        }

        float travel = hit.distance - desc_.ghostPadding;
        if (travel > 0.0f) position_ += direction * travel;
        remaining -= direction * (travel > 0.0f ? travel : 0.0f);
        ++result.contactCount;

        if (hit.normal.y < desc_.slopeLimitCosine && tryStep(position_, remaining, result)) {
            remaining = {};
            break;
        }

        if (hit.normal.y < desc_.slopeLimitCosine) result.hitWall = true;
        remaining = rejectNormal(remaining, hit.normal);
    }

    // The bounded sweep count is deliberate: do not leave a fifth residual
    // movement that could create corner jitter or inject energy next frame.
    ShapeCastHit finalGround;
    result.grounded = !jumped && probeGround(position_, finalGround);
    if (result.grounded) {
        result.groundNormal = finalGround.normal;
        ++result.contactCount;
    }
    result.finalPosition = position_;
    result.slideVelocity = position_ - start;
    return result;
}

} // namespace velox
