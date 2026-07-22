#pragma once
#include "body.h"
#include <cstdint>
#include <functional>

namespace velox {

/**
 * @file collision_filter.h
 * @brief Flexible collision filtering: 32 named layers, positive/negative
 *        group indices, custom filter callbacks, and combination rules.
 *
 * Velox collision filtering evaluates three independent criteria in a fixed
 * priority order:
 *
 * 1. **Group index** (`Body::groupIndex`): when two bodies share a non-zero
 *    group index the sign decides immediately — positive groups always
 *    collide, negative groups never collide. Group evaluation short-circuits
 *    the remaining tests.
 *
 * 2. **Layer bits** (`Body::categoryBits` / `Body::maskBits`): the classic
 *    bidirectional category/mask test. Each of the 32 bits is an independent
 *    collision layer; @ref CollisionLayers provides named constants for the
 *    first twelve.
 *
 * 3. **Custom callback** (set via `World::setCollisionFilterCallback`): an
 *    optional user-supplied function that can override the built-in result.
 *    The callback receives both bodies and returns @ref FilterResult::Accept,
 *    @ref FilterResult::Reject, or @ref FilterResult::Default (defer to the
 *    built-in group+layer evaluation). The callback is invoked only on the
 *    CPU broadphase path; GPU-resident stepping uses the built-in rules only.
 *
 * @code
 * velox::World world(velox::BackendType::Cpu);
 *
 * // Layer-based: floor on Static layer, ball on Dynamic layer
 * velox::BodyId floor = world.addStaticPlane({0,1,0}, 0.0f);
 * world.setCollisionFilter(floor,
 *     velox::CollisionLayers::Static,                    // category
 *     velox::CollisionLayers::Dynamic | velox::CollisionLayers::Character);
 *
 * velox::BodyId ball = world.addSphere({0,5,0}, 0.5f, 1.0f);
 * world.setCollisionFilter(ball,
 *     velox::CollisionLayers::Dynamic,
 *     velox::CollisionLayers::Static | velox::CollisionLayers::Dynamic);
 *
 * // Group-based: two debris pieces in the same negative group never collide
 * velox::BodyId d1 = world.addSphere({0,2,0}, 0.2f, 0.5f);
 * velox::BodyId d2 = world.addSphere({1,2,0}, 0.2f, 0.5f);
 * world.setCollisionFilter(d1, velox::CollisionFilterData{1, UINT32_MAX, -1});
 * world.setCollisionFilter(d2, velox::CollisionFilterData{1, UINT32_MAX, -1});
 *
 * // Custom callback: reject all contacts involving a specific body
 * world.setCollisionFilterCallback([](const velox::Body& a, const velox::Body& b) {
 *     if (a.categoryBits & velox::CollisionLayers::Trigger ||
 *         b.categoryBits & velox::CollisionLayers::Trigger)
 *         return velox::FilterResult::Reject;
 *     return velox::FilterResult::Default;
 * });
 * @endcode
 */

// ---------------------------------------------------------------------------
// Named collision layers
// ---------------------------------------------------------------------------

/**
 * @brief Predefined collision-layer bit constants.
 *
 * Layers 0–10 carry conventional names; layers 11–31 are reserved for
 * user-defined purposes (`User0` … `User20`). Every constant is a single
 * bit in the 32-bit `categoryBits` / `maskBits` fields on @ref Body.
 */
namespace CollisionLayers {
    constexpr uint32_t None       = 0;           ///< No layer — collides with nothing.
    constexpr uint32_t All        = UINT32_MAX;  ///< Every layer.
    constexpr uint32_t Default    = 1u << 0;     ///< General-purpose default layer.
    constexpr uint32_t Static     = 1u << 1;     ///< Non-moving level geometry.
    constexpr uint32_t Dynamic    = 1u << 2;     ///< Fully simulated rigid bodies.
    constexpr uint32_t Kinematic  = 1u << 3;     ///< Scripted / user-driven bodies.
    constexpr uint32_t Debris     = 1u << 4;     ///< Small cosmetic fragments.
    constexpr uint32_t Projectile = 1u << 5;     ///< Bullets, arrows, thrown objects.
    constexpr uint32_t Character  = 1u << 6;     ///< Player / NPC character controllers.
    constexpr uint32_t Vehicle    = 1u << 7;     ///< Wheeled / tracked vehicles.
    constexpr uint32_t Trigger    = 1u << 8;     ///< Sensor-only trigger volumes.
    constexpr uint32_t Ragdoll    = 1u << 9;     ///< Ragdoll articulation segments.
    constexpr uint32_t Water      = 1u << 10;    ///< Fluid / buoyancy volumes.
    // User-defined layers 11–31.
    constexpr uint32_t User0      = 1u << 11;
    constexpr uint32_t User1      = 1u << 12;
    constexpr uint32_t User2      = 1u << 13;
    constexpr uint32_t User3      = 1u << 14;
    constexpr uint32_t User4      = 1u << 15;
    constexpr uint32_t User5      = 1u << 16;
    constexpr uint32_t User6      = 1u << 17;
    constexpr uint32_t User7      = 1u << 18;
    constexpr uint32_t User8      = 1u << 19;
    constexpr uint32_t User9      = 1u << 20;
    constexpr uint32_t User10     = 1u << 21;
    constexpr uint32_t User11     = 1u << 22;
    constexpr uint32_t User12     = 1u << 23;
    constexpr uint32_t User13     = 1u << 24;
    constexpr uint32_t User14     = 1u << 25;
    constexpr uint32_t User15     = 1u << 26;
    constexpr uint32_t User16     = 1u << 27;
    constexpr uint32_t User17     = 1u << 28;
    constexpr uint32_t User18     = 1u << 29;
    constexpr uint32_t User19     = 1u << 30;
    constexpr uint32_t User20     = 1u << 31;
} // namespace CollisionLayers

// ---------------------------------------------------------------------------
// Group index semantics
// ---------------------------------------------------------------------------

/**
 * @brief Group-index collision rules.
 *
 * The group index (`Body::groupIndex`) provides a fast same-group override
 * that takes priority over layer bits:
 *
 * | groupIndex A | groupIndex B | Same? | Result                        |
 * |-------------:|-------------:|:-----:|-------------------------------|
 * |           +N |           +N |  yes  | **Always collide**            |
 * |           -N |           -N |  yes  | **Never collide**             |
 * |            0 |          any |   —   | Fall through to layer bits    |
 * |           +N |           -M |   no  | Fall through to layer bits    |
 * |           +N |            0 |   no  | Fall through to layer bits    |
 *
 * Positive groups are useful for ragdoll segments that must always interact;
 * negative groups prevent self-collision among debris spawned from the same
 * object.
 */
namespace CollisionGroups {
    constexpr int32_t None = 0; ///< No group — use layer filtering only.
}

// ---------------------------------------------------------------------------
// Filter data
// ---------------------------------------------------------------------------

/**
 * @brief Complete per-body collision filter configuration.
 *
 * Bundles the three filter dimensions (layer bits + group index) into a
 * single value type for convenient assignment via
 * `World::setCollisionFilter(BodyId, CollisionFilterData)`.
 */
struct CollisionFilterData {
    uint32_t categoryBits = 1u;         ///< Layers this body belongs to.
    uint32_t maskBits     = UINT32_MAX; ///< Layers this body collides with.
    int32_t  groupIndex   = 0;          ///< Same-group override (see @ref CollisionGroups).

    /// Construct from layer bits only (group = 0).
    constexpr CollisionFilterData() = default;
    constexpr CollisionFilterData(uint32_t category, uint32_t mask)
        : categoryBits(category), maskBits(mask), groupIndex(0) {}
    constexpr CollisionFilterData(uint32_t category, uint32_t mask, int32_t group)
        : categoryBits(category), maskBits(mask), groupIndex(group) {}
};

// ---------------------------------------------------------------------------
// Custom filter callback
// ---------------------------------------------------------------------------

/**
 * @brief Result returned by a custom collision filter callback.
 *
 * The callback is invoked once per candidate pair during the CPU broadphase,
 * *after* the built-in group + layer test has already run. Returning
 * @ref Default keeps the built-in result; @ref Accept or @ref Reject
 * overrides it unconditionally.
 */
enum class FilterResult : uint8_t {
    Default, ///< Keep the built-in group + layer result.
    Accept,  ///< Force the pair to collide regardless of layers/groups.
    Reject,  ///< Force the pair to be ignored regardless of layers/groups.
};

/**
 * @brief User-supplied collision filter callback.
 *
 * @param a  The first body (lower dense index in the candidate pair).
 * @param b  The second body.
 * @return   @ref FilterResult::Accept, @ref FilterResult::Reject, or
 *           @ref FilterResult::Default to defer to the built-in rules.
 *
 * The callback must be lightweight — it is invoked for every broadphase
 * candidate pair, potentially thousands of times per step. Avoid heap
 * allocations, locks, or I/O inside the callback.
 *
 * @note The callback is CPU-only. GPU-resident stepping
 *       (`GPUResidentMode::Resident`) uses the built-in group + layer
 *       rules and does not invoke the callback.
 */
using CollisionFilterCallback = std::function<FilterResult(const Body& a, const Body& b)>;

// ---------------------------------------------------------------------------
// Built-in filter evaluation (GPU-safe)
// ---------------------------------------------------------------------------

/**
 * @brief Evaluate the built-in group + layer collision filter.
 *
 * This is the core filter test used by the broadphase, narrowphase fallback,
 * CCD, and scene queries. It is `VELOX_HD` so the same code runs on CPU and
 * CUDA.
 *
 * Priority:
 * 1. If both bodies share a non-zero group index, the sign decides.
 * 2. Otherwise, the bidirectional category/mask layer test applies.
 *
 * @param a  First body.
 * @param b  Second body.
 * @return   True when the two bodies may collide.
 */
VELOX_HD inline bool evaluateCollisionFilter(const Body& a, const Body& b) {
    // --- Group override (highest priority) ---
    if (a.groupIndex != 0 && a.groupIndex == b.groupIndex)
        return a.groupIndex > 0;

    // --- Layer bits (bidirectional category/mask) ---
    return (a.maskBits & b.categoryBits) != 0 &&
           (b.maskBits & a.categoryBits) != 0;
}

/**
 * @brief Evaluate the collision filter including an optional custom callback.
 *
 * CPU-only wrapper around @ref evaluateCollisionFilter that additionally
 * invokes the user callback when one is set. Used by the World's broadphase
 * and CCD paths.
 *
 * @param a         First body.
 * @param b         Second body.
 * @param callback  Optional user callback (may be null / empty).
 * @return True when the two bodies may collide.
 */
inline bool evaluateCollisionFilterWithCallback(
        const Body& a, const Body& b,
        const CollisionFilterCallback& callback) {
    bool builtIn = evaluateCollisionFilter(a, b);
    if (!callback) return builtIn;
    FilterResult r = callback(a, b);
    if (r == FilterResult::Accept) return true;
    if (r == FilterResult::Reject) return false;
    return builtIn;
}

// ---------------------------------------------------------------------------
// Layer helpers
// ---------------------------------------------------------------------------

/**
 * @brief Build a category bitmask from individual layer bits.
 *
 * Convenience for combining layers without raw bitwise OR at the call site:
 * @code
 * uint32_t cat = velox::makeLayerMask(velox::CollisionLayers::Dynamic,
 *                                     velox::CollisionLayers::Projectile);
 * @endcode
 */
constexpr uint32_t makeLayerMask(uint32_t layer) { return layer; }

/// @overload
constexpr uint32_t makeLayerMask(uint32_t a, uint32_t b) { return a | b; }

/// @overload
constexpr uint32_t makeLayerMask(uint32_t a, uint32_t b, uint32_t c) {
    return a | b | c;
}

/// @overload
constexpr uint32_t makeLayerMask(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    return a | b | c | d;
}

/**
 * @brief Test whether a body's category includes a specific layer.
 * @param body  The body to inspect.
 * @param layer A single layer bit (e.g. `CollisionLayers::Dynamic`).
 * @return True when the body belongs to the given layer.
 */
VELOX_HD inline bool bodyHasLayer(const Body& body, uint32_t layer) {
    return (body.categoryBits & layer) != 0;
}

/**
 * @brief Test whether a body's mask accepts a specific layer.
 * @param body  The body to inspect.
 * @param layer A single layer bit.
 * @return True when the body collides with the given layer.
 */
VELOX_HD inline bool bodyAcceptsLayer(const Body& body, uint32_t layer) {
    return (body.maskBits & layer) != 0;
}

} // namespace velox
