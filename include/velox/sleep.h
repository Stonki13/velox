#pragma once
#include "body.h"
#include "math.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace velox {

/**
 * @file sleep.h
 * @brief Island-based sleeping with configurable thresholds, gradual sleep,
 *        contact stability tracking, sleep/wake callbacks, and statistics.
 *
 * The @ref SleepManager encapsulates Velox's improved sleep algorithm. Bodies
 * whose island stays below configurable motion thresholds for long enough are
 * put to sleep (zero simulation cost until something touches them). An optional
 * *drowsy* intermediate state reduces the simulation rate before full sleep,
 * preventing the visual "pop" of bodies freezing mid-motion.
 *
 * @code
 * // Through the World API (recommended):
 * velox::World world(velox::BackendType::Cpu);
 * world.sleepConfig().timeToSleep = 1.0f;
 * world.sleepConfig().enableGradualSleep = true;
 * world.setSleepCallbacks({
 *     .onSleep = [](velox::BodyId id) { /* body fell asleep *\/ },
 *     .onWake  = [](velox::BodyId id) { /* body woke up *\/ },
 * });
 * @endcode
 *
 * @see World::sleepConfig, World::sleepStats, World::setSleepCallbacks
 */

// Forward declarations for types defined in other headers.
struct Contact;
struct Joint;

/**
 * @brief Lifecycle state of a body in the sleep system.
 *
 * The @ref Body::asleep field encodes this as a `uint8_t`:
 * - `0` = Awake (fully simulated)
 * - `1` = Asleep (skips integration and solving entirely)
 * - `2` = Drowsy (simulated at a reduced rate before full sleep)
 *
 * The encoding is backward compatible: existing code that checks
 * `body.asleep != 0` to mean "not fully simulated" continues to work.
 */
enum class SleepState : uint8_t {
    Awake = 0,   ///< Fully simulated at normal rate.
    Asleep = 1,  ///< Skips integration and solving; zero cost.
    Drowsy = 2   ///< Simulated at a reduced rate (gradual sleep).
};

/// @brief Encode a @ref SleepState into the `Body::asleep` field.
VELOX_HD inline uint8_t sleepStateToByte(SleepState s) { return uint8_t(s); }

/// @brief Decode the `Body::asleep` field into a @ref SleepState.
VELOX_HD inline SleepState sleepStateFromByte(uint8_t v) {
    return v == 0 ? SleepState::Awake
         : v == 1 ? SleepState::Asleep
         : SleepState::Drowsy;
}

/// @brief True when the body is fully asleep (not simulated at all).
VELOX_HD inline bool isFullyAsleep(uint8_t asleepByte) { return asleepByte == 1; }

/// @brief True when the body is drowsy (reduced simulation rate).
VELOX_HD inline bool isDrowsy(uint8_t asleepByte) { return asleepByte == 2; }

/// @brief True when the body is not fully simulated (asleep or drowsy).
VELOX_HD inline bool isInactive(uint8_t asleepByte) { return asleepByte != 0; }

/**
 * @brief Configurable thresholds and tuning for the sleep system.
 *
 * All thresholds are in SI-like units (meters, seconds, radians). The default
 * values are tuned for typical game scenarios; adjust for your scale.
 *
 * A body is considered "calm" when its linear speed is below
 * @ref linearVelocityThreshold AND its angular speed is below
 * @ref angularVelocityThreshold AND its acceleration magnitude is below
 * @ref accelerationThreshold. Once every dynamic body in an island has been
 * calm for @ref timeToSleep seconds, the whole island falls asleep.
 *
 * When @ref enableGradualSleep is true, bodies enter the @ref SleepState::Drowsy
 * state after @ref timeToDrowsy seconds of calmness. Drowsy bodies are
 * simulated at @ref drowsySimulationRate (a fraction of the normal substep
 * count), giving a smooth visual transition before full sleep.
 *
 * When @ref enableContactStability is true, a body must also have stable
 * contacts (low relative velocity at contact points for
 * @ref contactStabilityFrames consecutive frames) before it can sleep. This
 * prevents bodies resting on vibrating or jittering surfaces from sleeping
 * prematurely.
 */
struct SleepConfig {
    /// @brief Linear speed threshold (m/s). Below this, the body is "calm".
    float linearVelocityThreshold = 0.05f;

    /// @brief Angular speed threshold (rad/s). Below this, the body is "calm".
    float angularVelocityThreshold = 0.05f;

    /// @brief Acceleration magnitude threshold (m/s²).
    ///
    /// Computed as `|force * invMass|` each step. Bodies with significant
    /// applied forces are kept awake even if their velocity is low.
    float accelerationThreshold = 0.01f;

    /// @brief Seconds every island member must stay calm before the island sleeps.
    float timeToSleep = 0.5f;

    /// @brief Seconds of calmness before entering the drowsy state.
    ///
    /// Only used when @ref enableGradualSleep is true. Must be less than
    /// @ref timeToSleep. Clamped to `[0, timeToSleep)` at validation time.
    float timeToDrowsy = 0.25f;

    /// @brief Fraction of substeps a drowsy body participates in (0, 1].
    ///
    /// Lower values save more CPU but make the transition more visible.
    /// `0.25` means the body is simulated every 4th substep.
    float drowsySimulationRate = 0.25f;

    /// @brief Enable the drowsy intermediate state (gradual sleep).
    bool enableGradualSleep = true;

    /// @brief Require stable contacts before allowing sleep.
    bool enableContactStability = true;

    /// @brief Maximum relative velocity at a contact for it to count as "stable".
    float contactStabilityThreshold = 0.02f;

    /// @brief Consecutive frames of stable contacts required before sleep.
    int contactStabilityFrames = 10;

    /// @brief Validate and clamp configuration values.
    ///
    /// Ensures `timeToDrowsy < timeToSleep`, rates are in valid ranges, and
    /// thresholds are non-negative. Call after modifying fields directly.
    void validate() {
        if (linearVelocityThreshold < 0.0f) linearVelocityThreshold = 0.0f;
        if (angularVelocityThreshold < 0.0f) angularVelocityThreshold = 0.0f;
        if (accelerationThreshold < 0.0f) accelerationThreshold = 0.0f;
        if (timeToSleep < 0.0f) timeToSleep = 0.0f;
        if (timeToDrowsy < 0.0f) timeToDrowsy = 0.0f;
        if (timeToDrowsy >= timeToSleep && timeToSleep > 0.0f)
            timeToDrowsy = timeToSleep * 0.5f;
        if (drowsySimulationRate <= 0.0f) drowsySimulationRate = 0.01f;
        if (drowsySimulationRate > 1.0f) drowsySimulationRate = 1.0f;
        if (contactStabilityThreshold < 0.0f) contactStabilityThreshold = 0.0f;
        if (contactStabilityFrames < 0) contactStabilityFrames = 0;
    }
};

/**
 * @brief Aggregate statistics from the sleep system.
 *
 * Updated once per `World::step()`. Query via `World::sleepStats()`.
 */
struct SleepStats {
    size_t totalDynamicBodies = 0;   ///< Dynamic bodies in the world.
    size_t awakeBodies = 0;          ///< Dynamic bodies fully simulated.
    size_t drowsyBodies = 0;         ///< Dynamic bodies in reduced-rate mode.
    size_t sleepingBodies = 0;       ///< Dynamic bodies fully asleep.
    size_t islandCount = 0;          ///< Total islands formed this step.
    size_t sleepingIslandCount = 0;  ///< Islands fully asleep.
    size_t drowsyIslandCount = 0;    ///< Islands in drowsy transition.
    size_t awakeIslandCount = 0;     ///< Islands fully awake.
    uint64_t totalSleepTransitions = 0; ///< Cumulative awake→asleep transitions.
    uint64_t totalWakeTransitions = 0;  ///< Cumulative asleep→awake transitions.
    uint64_t totalDrowsyTransitions = 0; ///< Cumulative awake→drowsy transitions.
    double lastUpdateMs = 0.0;       ///< Wall time of the last sleep update (ms).
};

/**
 * @brief Snapshot of one island's sleep state.
 *
 * Populated by `SleepManager::update()` and queryable via
 * `World::sleepIslands()` for debug visualization and diagnostics.
 */
struct SleepIsland {
    uint32_t root = 0;          ///< Union-find root (dense body index).
    SleepState state = SleepState::Awake; ///< Aggregate island state.
    size_t bodyCount = 0;       ///< Number of dynamic bodies in the island.
    float minSleepTimer = 0.0f; ///< Shortest calm timer among members.
    float maxMotion = 0.0f;     ///< Highest motion metric among members.
};

/**
 * @brief Callbacks invoked on sleep state transitions.
 *
 * Install via `World::setSleepCallbacks()`. All callbacks run synchronously
 * inside `World::step()` after the sleep update. The `BodyId` passed is valid
 * at the time of the call but may become stale after body removal.
 *
 * @warning Do not call `World::step()` or mutate the World from inside a
 * callback; doing so throws or deadlocks.
 */
struct SleepCallbacks {
    /// @brief Called when a body transitions to @ref SleepState::Asleep.
    std::function<void(BodyId)> onSleep;

    /// @brief Called when a body transitions from asleep/drowsy to @ref SleepState::Awake.
    std::function<void(BodyId)> onWake;

    /// @brief Called when a body transitions to @ref SleepState::Drowsy.
    std::function<void(BodyId)> onDrowsy;
};

/**
 * @brief Island-based sleep manager with configurable thresholds and gradual sleep.
 *
 * The SleepManager encapsulates the improved Velox sleep algorithm:
 *
 * 1. **Island formation**: Union-find over dynamic-dynamic contacts and joints
 *    groups bodies into independent islands.
 * 2. **Motion evaluation**: Each dynamic body's linear velocity, angular
 *    velocity, and acceleration are tested against configurable thresholds.
 * 3. **Contact stability**: When enabled, bodies must also have stable
 *    contacts (low relative velocity) for a configurable number of frames.
 * 4. **Gradual sleep**: Bodies enter a drowsy state (reduced simulation rate)
 *    before falling fully asleep, preventing visual popping.
 * 5. **Island sleep**: An island sleeps only when *every* member has been calm
 *    for at least `timeToSleep` seconds.
 * 6. **Callbacks**: Sleep/wake/drowsy transitions fire user callbacks.
 *
 * The manager is owned by @ref World and updated once per `step()`. Application
 * code configures it through `World::sleepConfig()` and reads diagnostics
 * through `World::sleepStats()` and `World::sleepIslands()`.
 */
class SleepManager {
public:
    SleepManager() = default;

    /// @name Configuration
    /// @{

    /// @brief Mutable access to the sleep configuration.
    ///
    /// Call `config().validate()` after modifying fields directly.
    SleepConfig& config() { return config_; }

    /// @brief Read-only access to the sleep configuration.
    const SleepConfig& config() const { return config_; }

    /// @brief Replace the entire configuration (validated).
    void setConfig(SleepConfig config) {
        config.validate();
        config_ = config;
    }

    /// @}

    /// @name Callbacks
    /// @{

    /// @brief Install sleep/wake/drowsy transition callbacks.
    void setCallbacks(SleepCallbacks callbacks) { callbacks_ = std::move(callbacks); }

    /// @brief Current callbacks (may be empty).
    const SleepCallbacks& callbacks() const { return callbacks_; }

    /// @}

    /// @name Statistics & diagnostics
    /// @{

    /// @brief Statistics from the most recent update.
    const SleepStats& stats() const { return stats_; }

    /// @brief Island snapshots from the most recent update.
    const std::vector<SleepIsland>& islands() const { return islands_; }

    /// @}

    /// @name State queries
    /// @{

    /**
     * @brief Decode a body's sleep state from its `asleep` field.
     * @param body The body to query.
     * @return The decoded @ref SleepState.
     */
    static SleepState bodySleepState(const Body& body) {
        return sleepStateFromByte(body.asleep);
    }

    /**
     * @brief True when the body should skip integration and solving entirely.
     * @param body The body to query.
     */
    static bool isFullyAsleep(const Body& body) {
        return velox::isFullyAsleep(body.asleep);
    }

    /**
     * @brief True when the body is in the drowsy reduced-rate state.
     * @param body The body to query.
     */
    static bool isDrowsy(const Body& body) {
        return velox::isDrowsy(body.asleep);
    }

    /// @}

    /// @name Forced state transitions
    /// @{

    /**
     * @brief Force a body awake, firing the onWake callback if it was inactive.
     * @param body The body to wake.
     * @param id   The body's handle (for callbacks).
     */
    void wakeBody(Body& body, BodyId id);

    /**
     * @brief Force a body to sleep immediately, firing the onSleep callback.
     * @param body The body to sleep.
     * @param id   The body's handle (for callbacks).
     */
    void sleepBody(Body& body, BodyId id);

    /// @}

    /**
     * @brief Run the sleep update for one timestep.
     *
     * Called once per `World::step()` after collision detection and solving.
     * Forms islands, evaluates motion and contact stability, advances sleep
     * timers, transitions drowsy/asleep states, and fires callbacks.
     *
     * @param bodies         Mutable body array (dense indices).
     * @param contacts       Active contacts from the current step.
     * @param joints         Active joints.
     * @param unionParent    Union-find parent array (resized internally).
     * @param islandTimer    Per-body island timer array (resized internally).
     * @param dt             Timestep in seconds.
     * @param handleResolver Maps a dense `BodyIndex` to its `BodyId` handle.
     */
    void update(std::vector<Body>& bodies,
                const std::vector<Contact>& contacts,
                const std::vector<Joint>& joints,
                std::vector<uint32_t>& unionParent,
                std::vector<float>& islandTimer,
                float dt,
                const std::function<BodyId(BodyIndex)>& handleResolver);

    /**
     * @brief Reset all internal state (timers, stability, islands).
     *
     * Called when the World is reset or bodies are bulk-removed. Does not
     * fire callbacks.
     */
    void reset();

private:
    SleepConfig config_;
    SleepStats stats_;
    SleepCallbacks callbacks_;
    std::vector<SleepIsland> islands_;
    /// Per-body count of consecutive frames with stable contacts.
    std::vector<uint16_t> contactStability_;
    /// Per-body drowsy substep phase counter for rate reduction.
    std::vector<uint8_t> drowsyPhase_;
    /// Monotonic counter for drowsy substep scheduling.
    uint32_t stepCounter_ = 0;
};

} // namespace velox
