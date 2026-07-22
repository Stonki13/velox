#pragma once

#include "body.h"
#include "math.h"
#include <cstdint>
#include <string>
#include <vector>

namespace velox {

/**
 * @file queries.h
 * @brief Scene-query types: raycasts, overlaps, shape casts, and the
 *        value-owned request/result pair used by batched and async queries.
 *
 * All queries are read-only against the World and never mutate simulation
 * state. Synchronous queries (`World::rayCast`, `World::overlapSphere`, ...)
 * resolve immediately against the current World state. Batched and async
 * queries copy a value-owned QueryDesc so the request can be safely
 * transferred across worker threads.
 */

/**
 * @brief Result of a single raycast.
 *
 * Returned by `World::rayCast` and as one element of `World::rayCastAll`.
 * Inspect @ref hit before reading the remaining fields; when @ref hit is
 * `false` the other members are unspecified.
 */
struct RayHit {
    bool hit = false;   ///< True when the ray intersected a body.
    BodyId body;        ///< Handle of the body that was hit.
    float t = 0.0f;     ///< Parametric distance along the ray (world units).
    Vec3 point, normal; ///< World-space hit point and outward surface normal.
};

/**
 * @brief Category/mask filter shared by every scene query.
 *
 * A body is considered by a query only when its collision bits satisfy the
 * filter and the sensor policy matches. The default filter accepts every
 * body, including sensors.
 *
 * Matching rule (mirrors `Body::canCollideWith`):
 * @code
 * (body.categoryBits & filter.maskBits) != 0 &&
 * (filter.categoryBits & body.maskBits) != 0
 * @endcode
 */
struct QueryFilter {
    uint32_t categoryBits = UINT32_MAX; ///< Categories this query belongs to.
    uint32_t maskBits = UINT32_MAX;     ///< Body categories the query accepts.
    bool includeSensors = true;         ///< When false, sensor bodies are skipped.
    BodyId ignoredBody;                 ///< Body excluded from the results, if any.
};

/**
 * @brief Result of a shape cast (sweep) query.
 *
 * Returned by `World::sphereCast`, `World::boxCast`, `World::capsuleCast`,
 * and `World::convexHullCast`. @ref fraction is the fraction of the swept
 * distance at which the first contact occurred (`0` = starting in contact).
 */
struct ShapeCastHit {
    bool hit = false;        ///< True when the swept shape hit a body.
    BodyId body;             ///< Handle of the body that was hit.
    float distance = 0.0f;   ///< World-space distance travelled before contact.
    float fraction = 0.0f;   ///< `distance / maxDist`, in the range `[0, 1]`.
    Vec3 point, normal;      ///< Contact point and outward surface normal.
};

/**
 * @brief A value-type request for one scene query.
 *
 * Batch and async queries accept only primitives so the request can be copied
 * safely across worker threads. Populate the fields relevant to @ref type and
 * leave the rest at their defaults.
 *
 * @see World::batchQueries, World::submitAsyncQuery, QueryResult
 */
struct QueryDesc {
    /** @brief The kind of scene query this request describes. */
    enum class Type : uint8_t {
        Raycast,        ///< Nearest ray intersection (`origin` + `direction`).
        RaycastAll,     ///< All ray intersections, nearest first.
        OverlapSphere,  ///< Bodies overlapping a sphere (`center`, `radius`).
        OverlapBox,     ///< Bodies overlapping an oriented box.
        OverlapCapsule, ///< Bodies overlapping an oriented capsule.
        SphereCast,     ///< Swept sphere (`center`, `radius`, `direction`).
        BoxCast,        ///< Swept oriented box.
        CapsuleCast,    ///< Swept oriented capsule.
    };

    Type type = Type::Raycast;      ///< Selects which fields are interpreted.
    QueryFilter filter{};           ///< Category/mask and sensor filtering.
    Vec3 origin{}, direction{};     ///< Ray origin and direction (raycasts/casts).
    float maxDist = 1e10f;          ///< Maximum ray/cast distance.
    Vec3 center{};                  ///< Overlap/cast shape center.
    Vec3 halfExtents{};             ///< Box half extents (box overlap/cast).
    Quat orientation{};             ///< Orientation for box/capsule shapes.
    float radius = 0.0f;            ///< Sphere/capsule radius.
    float capsuleHalfHeight = 0.0f; ///< Capsule half height along its axis.
    void* userData = nullptr;       ///< Opaque value echoed back in the result.
};

/**
 * @brief Value-owned result of a batched or async query.
 *
 * Every field is value-owned so a result remains valid after the World has
 * advanced. Only the field matching @ref type is populated on a successful
 * query; on failure @ref success is `false` and @ref error describes why.
 *
 * | `type`            | Populated field   |
 * | ----------------- | ----------------- |
 * | `Raycast`         | @ref rayHit       |
 * | `RaycastAll`      | @ref rayHits      |
 * | `Overlap*`        | @ref overlaps     |
 * | `*Cast`           | @ref shapeCastHit |
 */
struct QueryResult {
    bool success = false;               ///< True when the query executed cleanly.
    QueryDesc::Type type = QueryDesc::Type::Raycast; ///< Echo of the request type.
    void* userData = nullptr;           ///< Opaque value copied from the request.
    RayHit rayHit{};                    ///< Result for `Raycast`.
    std::vector<RayHit> rayHits;        ///< Result for `RaycastAll`.
    ShapeCastHit shapeCastHit{};        ///< Result for `SphereCast`/`BoxCast`/`CapsuleCast`.
    std::vector<BodyId> overlaps;       ///< Result for the `Overlap*` queries.
    std::string error;                  ///< Diagnostic message when `success` is false.
};

/**
 * @brief Opaque handle to an in-flight asynchronous query.
 *
 * Returned by `World::submitAsyncQuery`. Pass it to `World::getAsyncResult`
 * to block until the query has been resolved. A handle may be consumed once.
 */
struct AsyncQueryHandle {
    uint64_t id = 0; ///< Internal identifier; `0` is never a valid handle.
};

} // namespace velox
