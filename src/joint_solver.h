#pragma once
#include "velox/joint.h"

namespace velox::joint_solver {

constexpr int kIterations = 8;
constexpr float kBeta = 0.2f;
constexpr float kMaxPositionBias = 10.0f;

VELOX_HD inline Vec3 clampPositionBias(const Vec3& bias) {
    float magnitudeSq = lengthSq(bias);
    if (magnitudeSq <= kMaxPositionBias * kMaxPositionBias) return bias;
    return bias * (kMaxPositionBias / sqrtf(magnitudeSq));
}

struct Mat3 {
    Vec3 c0, c1, c2;
};

VELOX_HD inline Vec3 mul(const Mat3& m, const Vec3& v) {
    return m.c0 * v.x + m.c1 * v.y + m.c2 * v.z;
}

VELOX_HD inline Mat3 inverse(const Mat3& m) {
    Vec3 r0 = cross(m.c1, m.c2);
    Vec3 r1 = cross(m.c2, m.c0);
    Vec3 r2 = cross(m.c0, m.c1);
    float det = dot(m.c0, r0);
    float inv = fabsf(det) > 1e-12f ? 1.0f / det : 0.0f;
    return {{r0.x * inv, r1.x * inv, r2.x * inv},
            {r0.y * inv, r1.y * inv, r2.y * inv},
            {r0.z * inv, r1.z * inv, r2.z * inv}};
}

VELOX_HD inline Mat3 pointMass(const Body& a, const Body& b,
                               const Vec3& ra, const Vec3& rb) {
    Vec3 axes[3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    Mat3 result;
    Vec3* columns[3] = {&result.c0, &result.c1, &result.c2};
    for (int i = 0; i < 3; ++i) {
        Vec3 e = axes[i];
        Vec3 k = e * (a.solverInvMass() + b.solverInvMass());
        k += cross(a.invInertiaMul(cross(ra, e)), ra);
        k += cross(b.invInertiaMul(cross(rb, e)), rb);
        *columns[i] = k;
    }
    return result;
}

VELOX_HD inline Mat3 angularMass(const Body& a, const Body& b) {
    return {a.invInertiaMul({1,0,0}) + b.invInertiaMul({1,0,0}),
            a.invInertiaMul({0,1,0}) + b.invInertiaMul({0,1,0}),
            a.invInertiaMul({0,0,1}) + b.invInertiaMul({0,0,1})};
}

VELOX_HD inline Vec3 anchorVelocity(const Body& body, const Vec3& r) {
    return body.velocity + cross(body.angularVelocity, r);
}

VELOX_HD inline void applyImpulse(Body& body, const Vec3& r,
                                  const Vec3& impulse, float sign) {
    if (!body.isDynamic() || body.isLocked()) return;
    body.velocity += impulse * (sign * body.solverInvMass());
    body.angularVelocity += body.invInertiaMul(cross(r, impulse)) * sign;
}

VELOX_HD inline void applyLinear(Joint& joint, Body& a, Body& b,
                                 const Vec3& ra, const Vec3& rb,
                                 const Vec3& impulse, float signA) {
    applyImpulse(a, ra, impulse, signA);
    applyImpulse(b, rb, impulse, -signA);
    joint.reactionLinearImpulse += impulse * signA;
}

VELOX_HD inline void applyAngular(Joint& joint, Body& a, Body& b,
                                  const Vec3& impulse, float signA = 1.0f) {
    if (a.isDynamic() && !a.isLocked()) a.angularVelocity += a.invInertiaMul(impulse) * signA;
    if (b.isDynamic() && !b.isLocked()) b.angularVelocity -= b.invInertiaMul(impulse) * signA;
    joint.reactionAngularImpulse += impulse * signA;
}

VELOX_HD inline void frame(const Joint& joint, const Body& a, const Body& b,
                           Vec3& xA, Vec3& yA, Vec3& zA,
                           Vec3& xB, Vec3& yB, Vec3& zB) {
    xA = normalize(rotate(a.orientation, joint.localAxisA));
    yA = normalize(rotate(a.orientation, joint.localRefA));
    zA = normalize(cross(xA, yA));
    yA = cross(zA, xA);
    xB = normalize(rotate(b.orientation, joint.localAxisB));
    yB = normalize(rotate(b.orientation, joint.localRefB));
    zB = normalize(cross(xB, yB));
    yB = cross(zB, xB);
}

VELOX_HD inline Quat matrixQuaternion(float m00, float m01, float m02,
                                      float m10, float m11, float m12,
                                      float m20, float m21, float m22) {
    Quat q;
    float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        q = {(m21 - m12) / s, (m02 - m20) / s,
             (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s,
             (m21 - m12) / s};
    } else if (m11 > m22) {
        float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s,
             (m02 - m20) / s};
    } else {
        float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s,
             (m10 - m01) / s};
    }
    return normalize(q);
}

VELOX_HD inline Vec3 rotationVector(const Joint& joint,
                                    const Body& a, const Body& b) {
    Vec3 xA, yA, zA, xB, yB, zB;
    frame(joint, a, b, xA, yA, zA, xB, yB, zB);
    Quat q = matrixQuaternion(
        dot(xA, xB), dot(xA, yB), dot(xA, zB),
        dot(yA, xB), dot(yA, yB), dot(yA, zB),
        dot(zA, xB), dot(zA, yB), dot(zA, zB));
    if (q.w < 0.0f) q = {-q.x, -q.y, -q.z, -q.w};
    Vec3 v{q.x, q.y, q.z};
    float sinHalf = length(v);
    if (sinHalf < 1e-7f) return v * 2.0f;
    float angle = 2.0f * atan2f(sinHalf, vclamp(q.w, 0.0f, 1.0f));
    return v * (angle / sinHalf);
}

VELOX_HD inline float component(const Vec3& value, int axis) {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

VELOX_HD inline float& component(Vec3& value, int axis) {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

VELOX_HD inline void reset(Joint& joint) {
    joint.motorImpulse = 0.0f;
    joint.limitImpulse = 0.0f;
    joint.swingImpulse = 0.0f;
    joint.twistImpulse = 0.0f;
    joint.springImpulse = 0.0f;
    joint.linearMotorImpulse = {};
    joint.angularMotorImpulse = {};
    joint.linearLimitImpulse = {};
    joint.angularLimitImpulse = {};
    joint.reactionLinearImpulse = {};
    joint.reactionAngularImpulse = {};
    joint.broken = false;
}

VELOX_HD inline void solveAngularFrame(Joint& joint, Body& a, Body& b,
                                       float biasScale) {
    Vec3 xA, yA, zA, xB, yB, zB;
    frame(joint, a, b, xA, yA, zA, xB, yB, zB);
    Vec3 error = (cross(xB, xA) + cross(yB, yA) +
                  cross(zB, zA)) * 0.5f;
    Vec3 relativeW = a.angularVelocity - b.angularVelocity;
    Vec3 impulse = mul(inverse(angularMass(a, b)),
                       -(relativeW + error * biasScale));
    applyAngular(joint, a, b, impulse);
}

VELOX_HD inline void solveSixDof(Joint& joint, Body& a, Body& b,
                                 const Vec3& ra, const Vec3& rb,
                                 const Vec3& pa, const Vec3& pb, float dt) {
    Vec3 xA, yA, zA, xB, yB, zB;
    frame(joint, a, b, xA, yA, zA, xB, yB, zB);
    Vec3 axes[3] = {xA, yA, zA};
    Vec3 d = pb - pa;
    Mat3 pointK = pointMass(a, b, ra, rb);
    for (int i = 0; i < 3; ++i) {
        uint8_t bit = uint8_t(1u << i);
        Vec3 axis = axes[i];
        float k = dot(axis, mul(pointK, axis));
        if (k <= 0.0f) continue;
        float speed = dot(anchorVelocity(b, rb) - anchorVelocity(a, ra), axis);
        if (joint.linearMotorMask & bit) {
            float lambda = (component(joint.linearMotorSpeed, i) - speed) / k;
            float maxImpulse = component(joint.maxLinearMotorForce, i) * dt;
            float& sum = component(joint.linearMotorImpulse, i);
            float old = sum;
            sum = vclamp(old + lambda, -maxImpulse, maxImpulse);
            lambda = sum - old;
            applyLinear(joint, a, b, ra, rb, axis * lambda, -1.0f);
            speed = dot(anchorVelocity(b, rb) - anchorVelocity(a, ra), axis);
        }
        if (!(joint.linearLimitMask & bit)) continue;
        float position = dot(d, axis);
        float lower = component(joint.lowerLinearLimit, i);
        float upper = component(joint.upperLinearLimit, i);
        float lambda = 0.0f;
        float& sum = component(joint.linearLimitImpulse, i);
        float old = sum;
        if (lower == upper) {
            lambda = -(speed + (position - lower) * (kBeta / dt)) / k;
            sum += lambda;
        } else if (position < lower) {
            lambda = -(speed + (position - lower) * (kBeta / dt)) / k;
            sum = vmax(0.0f, old + lambda);
            lambda = sum - old;
        } else if (position > upper) {
            lambda = -(speed + (position - upper) * (kBeta / dt)) / k;
            sum = vmin(0.0f, old + lambda);
            lambda = sum - old;
        }
        if (lambda != 0.0f)
            applyLinear(joint, a, b, ra, rb, axis * lambda, -1.0f);
    }

    Vec3 rotation = rotationVector(joint, a, b);
    for (int i = 0; i < 3; ++i) {
        uint8_t bit = uint8_t(1u << i);
        Vec3 axis = axes[i];
        float k = dot(axis, a.invInertiaMul(axis)) +
                  dot(axis, b.invInertiaMul(axis));
        if (k <= 0.0f) continue;
        float speed = dot(b.angularVelocity - a.angularVelocity, axis);
        if (joint.angularMotorMask & bit) {
            float lambda = (speed - component(joint.angularMotorSpeed, i)) / k;
            float maxImpulse = component(joint.maxAngularMotorTorque, i) * dt;
            float& sum = component(joint.angularMotorImpulse, i);
            float old = sum;
            sum = vclamp(old + lambda, -maxImpulse, maxImpulse);
            lambda = sum - old;
            applyAngular(joint, a, b, axis * lambda);
            speed = dot(b.angularVelocity - a.angularVelocity, axis);
        }
        if (!(joint.angularLimitMask & bit)) continue;
        float angle = component(rotation, i);
        float lower = component(joint.lowerAngularLimit, i);
        float upper = component(joint.upperAngularLimit, i);
        float lambda = 0.0f;
        float& sum = component(joint.angularLimitImpulse, i);
        float old = sum;
        if (lower == upper) {
            lambda = (speed + (angle - lower) * (kBeta / dt)) / k;
            sum += lambda;
        } else if (angle < lower) {
            lambda = (speed + (angle - lower) * (kBeta / dt)) / k;
            sum = vmin(0.0f, old + lambda);
            lambda = sum - old;
        } else if (angle > upper) {
            lambda = (speed + (angle - upper) * (kBeta / dt)) / k;
            sum = vmax(0.0f, old + lambda);
            lambda = sum - old;
        }
        if (lambda != 0.0f) applyAngular(joint, a, b, axis * lambda);
    }
}

VELOX_HD inline void solvePrismatic(Joint& joint, Body& a, Body& b,
                                    const Vec3& ra, const Vec3& rb,
                                    const Vec3& pa, const Vec3& pb, float dt) {
    Vec3 axis = normalize(rotate(a.orientation, joint.localAxisA));
    Vec3 ref = rotate(a.orientation, joint.localRefA);
    Vec3 t1 = normalize(ref - axis * dot(ref, axis));
    if (lengthSq(t1) < 1e-12f) {
        ref = fabsf(axis.x) < 0.9f ? Vec3{1,0,0} : Vec3{0,1,0};
        t1 = normalize(cross(axis, ref));
    }
    Vec3 tangents[2] = {t1, cross(axis, t1)};
    Mat3 pointK = pointMass(a, b, ra, rb);
    Vec3 error = pa - pb;
    Vec3 velocity = anchorVelocity(a, ra) - anchorVelocity(b, rb);
    for (int i = 0; i < 2; ++i) {
        Vec3 tangent = tangents[i];
        float k = dot(tangent, mul(pointK, tangent));
        if (k <= 0.0f) continue;
        float lambda = -(dot(velocity, tangent) +
                         dot(error, tangent) * (kBeta / dt)) / k;
        applyLinear(joint, a, b, ra, rb, tangent * lambda, 1.0f);
        velocity = anchorVelocity(a, ra) - anchorVelocity(b, rb);
    }
    solveAngularFrame(joint, a, b, kBeta / dt);

    float kAxis = dot(axis, mul(pointK, axis));
    if (kAxis <= 0.0f) return;
    float axisSpeed = dot(anchorVelocity(b, rb) -
                          anchorVelocity(a, ra), axis);
    if (joint.enableMotor) {
        float lambda = (joint.motorSpeed - axisSpeed) / kAxis;
        float maxImpulse = joint.maxMotorForce * dt;
        float old = joint.motorImpulse;
        joint.motorImpulse = vclamp(old + lambda, -maxImpulse, maxImpulse);
        lambda = joint.motorImpulse - old;
        applyLinear(joint, a, b, ra, rb, axis * lambda, -1.0f);
        axisSpeed = dot(anchorVelocity(b, rb) -
                        anchorVelocity(a, ra), axis);
    }
    if (!joint.enableLimit) return;
    float translation = dot(pb - pa, axis);
    float lambda = 0.0f;
    float old = joint.limitImpulse;
    if (translation < joint.lowerLimit) {
        float bias = (translation - joint.lowerLimit) * (kBeta / dt);
        lambda = -(axisSpeed + bias) / kAxis;
        joint.limitImpulse = vmax(0.0f, old + lambda);
        lambda = joint.limitImpulse - old;
    } else if (translation > joint.upperLimit) {
        float bias = (translation - joint.upperLimit) * (kBeta / dt);
        lambda = -(axisSpeed + bias) / kAxis;
        joint.limitImpulse = vmin(0.0f, old + lambda);
        lambda = joint.limitImpulse - old;
    }
    if (lambda != 0.0f)
        applyLinear(joint, a, b, ra, rb, axis * lambda, -1.0f);
}

VELOX_HD inline void solveHinge(Joint& joint, Body& a, Body& b,
                                const Vec3& ra, const Vec3& rb,
                                const Vec3& pa, const Vec3& pb, float dt) {
    Vec3 velocity = anchorVelocity(a, ra) - anchorVelocity(b, rb);
    Vec3 bias = clampPositionBias((pa - pb) * (kBeta / dt));
    Vec3 impulse = mul(inverse(pointMass(a, b, ra, rb)),
                       -(velocity + bias));
    applyLinear(joint, a, b, ra, rb, impulse, 1.0f);

    Vec3 axisA = rotate(a.orientation, joint.localAxisA);
    Vec3 axisB = rotate(b.orientation, joint.localAxisB);
    Vec3 error = cross(axisB, axisA);
    Vec3 ref = fabsf(axisA.x) < 0.9f ? Vec3{1,0,0} : Vec3{0,1,0};
    Vec3 tangents[2] = {normalize(cross(axisA, ref)), {}};
    tangents[1] = cross(axisA, tangents[0]);
    Vec3 relativeW = a.angularVelocity - b.angularVelocity;
    for (int i = 0; i < 2; ++i) {
        Vec3 tangent = tangents[i];
        float k = dot(tangent, a.invInertiaMul(tangent)) +
                  dot(tangent, b.invInertiaMul(tangent));
        if (k <= 0.0f) continue;
        float lambda = -(dot(relativeW, tangent) +
                         dot(error, tangent) * (kBeta / dt)) / k;
        applyAngular(joint, a, b, tangent * lambda);
        relativeW = a.angularVelocity - b.angularVelocity;
    }

    float kAxis = dot(axisA, a.invInertiaMul(axisA)) +
                  dot(axisA, b.invInertiaMul(axisA));
    if (kAxis <= 0.0f) return;
    if (joint.enableMotor) {
        float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
        float lambda = (joint.motorSpeed - wAxis) / kAxis;
        float maxImpulse = joint.maxMotorTorque * dt;
        float old = joint.motorImpulse;
        joint.motorImpulse = vclamp(old + lambda, -maxImpulse, maxImpulse);
        lambda = joint.motorImpulse - old;
        applyAngular(joint, a, b, axisA * lambda);
    }
    if (!joint.enableLimit) return;
    Vec3 refA = rotate(a.orientation, joint.localRefA);
    Vec3 refB = rotate(b.orientation, joint.localRefB);
    float angle = atan2f(dot(cross(refA, refB), axisA), dot(refA, refB));
    float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
    float lambda = 0.0f;
    float old = joint.limitImpulse;
    if (angle < joint.lowerLimit) {
        float target = (angle - joint.lowerLimit) * (kBeta / dt);
        lambda = (target - wAxis) / kAxis;
        joint.limitImpulse = vmin(0.0f, old + lambda);
        lambda = joint.limitImpulse - old;
    } else if (angle > joint.upperLimit) {
        float target = (angle - joint.upperLimit) * (kBeta / dt);
        lambda = (target - wAxis) / kAxis;
        joint.limitImpulse = vmax(0.0f, old + lambda);
        lambda = joint.limitImpulse - old;
    }
    if (lambda != 0.0f) applyAngular(joint, a, b, axisA * lambda);
}

VELOX_HD inline void solveConeTwist(Joint& joint, Body& a, Body& b,
                                    const Vec3& ra, const Vec3& rb,
                                    const Vec3& pa, const Vec3& pb, float dt) {
    Vec3 velocity = anchorVelocity(a, ra) - anchorVelocity(b, rb);
    Vec3 bias = (pa - pb) * (kBeta / dt);
    Vec3 impulse = mul(inverse(pointMass(a, b, ra, rb)),
                       -(velocity + bias));
    applyLinear(joint, a, b, ra, rb, impulse, 1.0f);

    Vec3 axisA = normalize(rotate(a.orientation, joint.localAxisA));
    Vec3 axisB = normalize(rotate(b.orientation, joint.localAxisB));
    Vec3 relativeW = a.angularVelocity - b.angularVelocity;
    if (joint.enableSwingLimit) {
        float angle = acosf(vclamp(dot(axisA, axisB), -1.0f, 1.0f));
        if (angle > joint.swingLimit) {
            Vec3 n = cross(axisB, axisA);
            if (lengthSq(n) < 1e-10f) {
                Vec3 refA = rotate(a.orientation, joint.localRefA);
                n = cross(axisA, refA);
            }
            n = normalize(n);
            float k = dot(n, a.invInertiaMul(n)) +
                      dot(n, b.invInertiaMul(n));
            if (k > 0.0f) {
                float error = angle - joint.swingLimit;
                float lambda = -(dot(relativeW, n) +
                                 error * (kBeta / dt)) / k;
                float old = joint.swingImpulse;
                joint.swingImpulse = vmin(0.0f, old + lambda);
                lambda = joint.swingImpulse - old;
                applyAngular(joint, a, b, n * lambda);
                relativeW = a.angularVelocity - b.angularVelocity;
            }
        }
    }
    if (!joint.enableTwistLimit) return;
    Vec3 refA = rotate(a.orientation, joint.localRefA);
    Vec3 refB = rotate(b.orientation, joint.localRefB);
    refA = normalize(refA - axisA * dot(refA, axisA));
    refB = normalize(refB - axisA * dot(refB, axisA));
    if (lengthSq(refA) < 1e-10f || lengthSq(refB) < 1e-10f) return;
    float angle = atan2f(dot(cross(refA, refB), axisA), dot(refA, refB));
    float k = dot(axisA, a.invInertiaMul(axisA)) +
              dot(axisA, b.invInertiaMul(axisA));
    if (k <= 0.0f) return;
    float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
    float lambda = 0.0f;
    float old = joint.twistImpulse;
    if (angle < joint.lowerTwistLimit) {
        float target = (angle - joint.lowerTwistLimit) * (kBeta / dt);
        lambda = (target - wAxis) / k;
        joint.twistImpulse = vmin(0.0f, old + lambda);
        lambda = joint.twistImpulse - old;
    } else if (angle > joint.upperTwistLimit) {
        float target = (angle - joint.upperTwistLimit) * (kBeta / dt);
        lambda = (target - wAxis) / k;
        joint.twistImpulse = vmax(0.0f, old + lambda);
        lambda = joint.twistImpulse - old;
    }
    if (lambda != 0.0f) applyAngular(joint, a, b, axisA * lambda);
}

VELOX_HD inline void solve(Joint& joint, Body& a, Body& b, float dt) {
    Vec3 ra = rotate(a.orientation, joint.localAnchorA);
    Vec3 rb = rotate(b.orientation, joint.localAnchorB);
    Vec3 pa = a.position + ra;
    Vec3 pb = b.position + rb;
    Vec3 velocity = anchorVelocity(a, ra) - anchorVelocity(b, rb);
    switch (joint.type) {
    case JointType::Ball: {
        Vec3 bias = (pa - pb) * (kBeta / dt);
        Vec3 impulse = mul(inverse(pointMass(a, b, ra, rb)),
                           -(velocity + bias));
        applyLinear(joint, a, b, ra, rb, impulse, 1.0f);
        break;
    }
    case JointType::Distance: {
        Vec3 d = pa - pb;
        float len = length(d);
        if (len < 1e-8f) break;
        Vec3 n = d * (1.0f / len);
        Vec3 raxn = cross(ra, n), rbxn = cross(rb, n);
        float k = a.solverInvMass() + b.solverInvMass() +
                  dot(raxn, a.invInertiaMul(raxn)) +
                  dot(rbxn, b.invInertiaMul(rbxn));
        if (k <= 0.0f) break;
        float lambda;
        if (joint.enableSpring) {
            constexpr float tau = 6.28318530718f;
            float mass = 1.0f / k;
            float omega = tau * joint.springFrequencyHz;
            float stiffness = mass * omega * omega;
            float damping = 2.0f * mass * joint.springDampingRatio * omega;
            float softness = 1.0f / (dt * (damping + dt * stiffness));
            float bias = (len - joint.restLength) * dt * stiffness * softness;
            lambda = -(dot(velocity, n) + bias +
                       softness * joint.springImpulse) / (k + softness);
            joint.springImpulse += lambda;
        } else {
            float bias = (len - joint.restLength) * (kBeta / dt);
            lambda = -(dot(velocity, n) + bias) / k;
        }
        applyLinear(joint, a, b, ra, rb, n * lambda, 1.0f);
        break;
    }
    case JointType::Fixed: {
        Vec3 bias = (pa - pb) * (kBeta / dt);
        Vec3 impulse = mul(inverse(pointMass(a, b, ra, rb)),
                           -(velocity + bias));
        applyLinear(joint, a, b, ra, rb, impulse, 1.0f);
        solveAngularFrame(joint, a, b, kBeta / dt);
        break;
    }
    case JointType::SixDof:
        solveSixDof(joint, a, b, ra, rb, pa, pb, dt);
        break;
    case JointType::Prismatic:
        solvePrismatic(joint, a, b, ra, rb, pa, pb, dt);
        break;
    case JointType::Hinge:
        solveHinge(joint, a, b, ra, rb, pa, pb, dt);
        break;
    case JointType::ConeTwist:
        solveConeTwist(joint, a, b, ra, rb, pa, pb, dt);
        break;
    default:
        break;
    }
}

inline bool supported(const Joint& joint) {
    return (uint8_t)joint.type <= (uint8_t)JointType::SixDof;
}

} // namespace velox::joint_solver
