#include "diff_test.h"

#include <algorithm>
#include <cmath>

namespace difftest {
namespace {

float pearson(const std::vector<float>& a, const std::vector<float>& b) {
    const size_t n = std::min(a.size(), b.size());
    if (n < 2) return 1.0f;
    double meanA = 0.0, meanB = 0.0;
    for (size_t i = 0; i < n; ++i) { meanA += a[i]; meanB += b[i]; }
    meanA /= static_cast<double>(n);
    meanB /= static_cast<double>(n);
    double covariance = 0.0, varA = 0.0, varB = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double da = a[i] - meanA, db = b[i] - meanB;
        covariance += da * db;
        varA += da * da;
        varB += db * db;
    }
    // A near-constant series (a resting body's solver jitter) carries no
    // correlation information; treat it as perfectly correlated rather than
    // correlating sub-millimeter noise. Threshold: stddev < 2 mm.
    const double floor = 4e-6 * static_cast<double>(n);
    if (varA < floor || varB < floor) return 1.0f;
    return static_cast<float>(covariance / std::sqrt(varA * varB));
}

} // namespace

float totalEnergy(const SceneDesc& scene, const FrameState& frame) {
    float energy = 0.0f;
    const float g = length(scene.gravity);
    for (size_t i = 0; i < frame.bodies.size() && i < scene.tracked.size(); ++i) {
        const BodyDesc& desc = scene.bodies[static_cast<size_t>(scene.tracked[i])];
        if (desc.mass <= 0.0f) continue;
        const BodySample& sample = frame.bodies[i];
        const float speedSq = sample.velocity.x * sample.velocity.x +
                              sample.velocity.y * sample.velocity.y +
                              sample.velocity.z * sample.velocity.z;
        // Rough rotational term: solid-sphere-like inertia bound. The metric
        // tracks an engine's own drift over time, so a coarse tensor is fine.
        const float radius = desc.shape == BodyDesc::Shape::Box
            ? length(desc.halfExtents) : desc.radius;
        const float angularSq = sample.angularVelocity.x * sample.angularVelocity.x +
                                sample.angularVelocity.y * sample.angularVelocity.y +
                                sample.angularVelocity.z * sample.angularVelocity.z;
        energy += 0.5f * desc.mass * speedSq;
        energy += 0.5f * (0.4f * desc.mass * radius * radius) * angularSq;
        energy += desc.mass * g * sample.position.y;
    }
    return energy;
}

DiffResult compare(const SceneDesc& scene, const Trajectory& a, const Trajectory& b,
                   const Tolerances& tolerances) {
    DiffResult result;
    const size_t frames = std::min(a.size(), b.size());
    const size_t bodies = scene.tracked.size();

    for (size_t frame = 0; frame < frames; ++frame) {
        for (size_t body = 0; body < bodies; ++body) {
            const BodySample& sa = a[frame].bodies[body];
            const BodySample& sb = b[frame].bodies[body];
            const Vec3f dp{sa.position.x - sb.position.x, sa.position.y - sb.position.y,
                           sa.position.z - sb.position.z};
            const Vec3f dv{sa.velocity.x - sb.velocity.x, sa.velocity.y - sb.velocity.y,
                           sa.velocity.z - sb.velocity.z};
            result.maxPositionDelta = std::max(result.maxPositionDelta, length(dp));
            result.maxVelocityDelta = std::max(result.maxVelocityDelta, length(dv));
        }
    }

    // Per-engine energy drift relative to the first frame (plus a small floor
    // so scenes that start at rest do not divide by ~zero).
    auto drift = [&](const Trajectory& trajectory) {
        if (trajectory.empty()) return 0.0f;
        const float initial = totalEnergy(scene, trajectory.front());
        const float floor = std::max(std::fabs(initial), 1.0f);
        float worst = 0.0f;
        for (const FrameState& frame : trajectory) {
            const float now = totalEnergy(scene, frame);
            // Dissipation (friction, restitution < 1) is expected; only GAINED
            // energy indicates an unstable solver.
            worst = std::max(worst, (now - initial) / floor);
        }
        return worst;
    };
    result.energyDriftA = drift(a);
    result.energyDriftB = drift(b);

    // Pearson correlation per body per axis; keep the minimum.
    result.trajectoryCorrelation = 1.0f;
    for (size_t body = 0; body < bodies; ++body) {
        for (int axis = 0; axis < 3; ++axis) {
            std::vector<float> seriesA(frames), seriesB(frames);
            for (size_t frame = 0; frame < frames; ++frame) {
                const Vec3f& pa = a[frame].bodies[body].position;
                const Vec3f& pb = b[frame].bodies[body].position;
                seriesA[frame] = axis == 0 ? pa.x : axis == 1 ? pa.y : pa.z;
                seriesB[frame] = axis == 0 ? pb.x : axis == 1 ? pb.y : pb.z;
            }
            result.trajectoryCorrelation =
                std::min(result.trajectoryCorrelation, pearson(seriesA, seriesB));
        }
    }

    result.passed = result.maxPositionDelta <= tolerances.maxPositionDelta &&
                    result.maxVelocityDelta <= tolerances.maxVelocityDelta &&
                    result.energyDriftA <= tolerances.maxEnergyDrift &&
                    result.energyDriftB <= tolerances.maxEnergyDrift &&
                    result.trajectoryCorrelation >= tolerances.minCorrelation;
    return result;
}

} // namespace difftest
