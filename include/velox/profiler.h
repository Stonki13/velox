#pragma once
// Velox — hierarchical performance profiler.
//
// A self-contained, header-only instrumentation toolkit whose job is to make
// the engine's performance bottlenecks *visible and actionable*. It is built
// around five ideas:
//
//   1. Scoped timers      — RAII zones that time a block of code.
//   2. Hierarchy          — zones nest into a call tree, so you can see not
//                           just *what* is slow but *where* the time goes
//                           (total vs. self time, parent vs. child).
//   3. Frame breakdown    — every zone carries a Category; the profiler
//                           accumulates per-category time for the last frame
//                           so a single glance answers "is this frame bound by
//                           broadphase, narrowphase, or the solver?".
//   4. Memory tracking    — allocation/deallocation hooks report current,
//                           peak, and total bytes plus object counts.
//   5. GPU/CPU sync points— named instant markers flag the expensive
//                           host<->device synchronization boundaries.
//
// Everything can be dumped to the Chrome Trace Event Format (chrome://tracing
// or https://ui.perfetto.dev) for interactive visualization, and to a plain
// text report for terminal/CI use.
//
// The profiler is OFF by default. While disabled every macro collapses to a
// single relaxed atomic load, so leaving instrumentation in hot paths costs
// essentially nothing in shipping builds. Enable it with:
//
//     velox::profile::Profiler::instance().setEnabled(true);
//
// Typical use:
//
//     auto& prof = velox::profile::Profiler::instance();
//     prof.setEnabled(true);
//     prof.beginFrame();
//     {
//         VELOX_PROFILE_CATEGORY_SCOPE("Broadphase", velox::profile::Category::Broadphase);
//         doBroadphase();
//     }
//     prof.endFrame();
//     prof.printReport();
//     prof.exportChromeTrace("velox_trace.json");

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace velox {
namespace profile {

// Coarse buckets used for the per-frame time breakdown. The hot paths of the
// engine each map onto one of these so a frame can be decomposed at a glance.
enum class Category : uint8_t {
    None = 0,
    Setup,
    Broadphase,
    Narrowphase,
    ConstraintSolve,
    Island,
    Ccd,
    Finalize,
    Render,
    Custom,
    Count
};

inline const char* categoryName(Category category) {
    switch (category) {
    case Category::None:          return "None";
    case Category::Setup:         return "Setup";
    case Category::Broadphase:    return "Broadphase";
    case Category::Narrowphase:   return "Narrowphase";
    case Category::ConstraintSolve: return "ConstraintSolve";
    case Category::Island:        return "Island";
    case Category::Ccd:           return "Ccd";
    case Category::Finalize:      return "Finalize";
    case Category::Render:        return "Render";
    case Category::Custom:        return "Custom";
    default:                      return "Unknown";
    }
}

// Aggregated statistics for one node of the zone hierarchy tree.
struct ZoneStats {
    const char* name = "";
    Category category = Category::None;
    uint64_t count = 0;
    double totalMs = 0.0;   // wall time including children
    double selfMs = 0.0;    // wall time excluding children
    double minMs = 0.0;
    double maxMs = 0.0;
    uint32_t maxDepth = 0;
};

// Per-frame category breakdown produced by endFrame().
struct FrameBreakdown {
    uint64_t frameIndex = 0;
    double frameMs = 0.0;
    double categoryMs[static_cast<size_t>(Category::Count)] = {};
};

// Memory tracking snapshot.
struct MemoryStats {
    uint64_t allocationCount = 0;
    uint64_t deallocationCount = 0;
    uint64_t currentBytes = 0;
    uint64_t peakBytes = 0;
    uint64_t totalAllocatedBytes = 0;
};

namespace detail {

inline uint64_t nowNs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

// One completed zone, retained for the Chrome trace export.
struct RawZone {
    const char* name;
    Category category;
    uint64_t startNs;
    uint64_t endNs;
    uint32_t threadId;
    uint32_t depth;
};

// An instant marker (GPU/CPU sync point, frame boundary, ...).
struct RawInstant {
    const char* name;
    uint64_t timeNs;
    uint32_t threadId;
    char scope; // 'g' global, 'p' process, 't' thread
};

// A counter sample, rendered as a Chrome "C" event (e.g. memory, frame time).
struct RawCounter {
    const char* name;
    uint64_t timeNs;
    double value;
};

// A node in the aggregated hierarchy tree. Node 0 is the synthetic root.
struct AggNode {
    const char* name = "<root>";
    Category category = Category::None;
    size_t parent = SIZE_MAX;
    std::vector<size_t> children;
    ZoneStats stats;
};

// Per-thread active-zone frame used to compute hierarchy and self time.
struct StackFrame {
    size_t node;        // index into Profiler::nodes_
    uint64_t startNs;
    double childMs;     // time already attributed to children
};

inline std::string jsonEscape(const char* text) {
    std::string out;
    for (const char* p = text; *p; ++p) {
        switch (*p) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:   out += *p; break;
        }
    }
    return out;
}

} // namespace detail

class Profiler {
public:
    // Meyers singleton: thread-safe construction, no static-init-order issues,
    // and a single shared instance across all translation units (the inline
    // function-local static is the same object everywhere in C++17).
    static Profiler& instance() {
        static Profiler profiler;
        return profiler;
    }

    // --- enable / configure ---------------------------------------------------
    void setEnabled(bool enabled) { enabled_.store(enabled, std::memory_order_relaxed); }
    bool enabled() const { return enabled_.load(std::memory_order_relaxed); }

    // Cap on retained raw trace events so a long capture cannot grow unbounded.
    void setMaxTraceEvents(size_t maxEvents) {
        std::lock_guard<std::mutex> lock(mutex_);
        maxTraceEvents_ = maxEvents;
    }
    size_t droppedEvents() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return droppedEvents_;
    }

    // --- thread naming --------------------------------------------------------
    // Returns a stable small integer id for the calling thread, registering it
    // on first use. The id is what the Chrome trace groups events under.
    uint32_t registerThread(const char* name = nullptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t id = static_cast<uint32_t>(threadNames_.size());
        threadNames_.push_back(name ? name : ("Thread-" + std::to_string(id)));
        return id;
    }
    void nameCurrentThread(const char* name) {
        uint32_t id = currentThreadId();
        std::lock_guard<std::mutex> lock(mutex_);
        if (id < threadNames_.size()) threadNames_[id] = name;
    }

    // --- frame lifecycle ------------------------------------------------------
    void beginFrame() {
        if (!enabled()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        ++frameCounter_;
        frameStartNs_ = detail::nowNs();
        for (double& value : frameCategoryMs_) value = 0.0;
    }

    FrameBreakdown endFrame() {
        FrameBreakdown breakdown;
        if (!enabled()) return breakdown;
        const uint64_t endNs = detail::nowNs();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            breakdown.frameIndex = frameCounter_;
            breakdown.frameMs = (endNs - frameStartNs_) * 1e-6;
            for (size_t i = 0; i < frameCategoryMs_.size(); ++i)
                breakdown.categoryMs[i] = frameCategoryMs_[i];
            lastBreakdown_ = breakdown;
            // Counter event so the frame time shows as a graph in the trace.
            pushCounterLocked("FrameTimeMs", endNs, breakdown.frameMs);
        }
        return breakdown;
    }

    FrameBreakdown lastBreakdown() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastBreakdown_;
    }

    // --- zone recording (called by ScopedZone) --------------------------------
    void beginZone(const char* name, Category category, detail::StackFrame& frame) {
        frame.node = SIZE_MAX;
        if (!enabled()) return;
        const uint64_t startNs = detail::nowNs();
        const uint32_t threadId = currentThreadId();
        std::lock_guard<std::mutex> lock(mutex_);
        size_t parent = stackFor(threadId).empty() ? 0 : stackFor(threadId).back().node;
        size_t node = findOrCreateChild(parent, name, category);
        const uint32_t depth = static_cast<uint32_t>(stackFor(threadId).size());
        nodes_[node].stats.maxDepth = std::max(nodes_[node].stats.maxDepth, depth);
        stackFor(threadId).push_back({node, startNs, 0.0});
        frame.node = node;
        frame.startNs = startNs;
        frame.childMs = 0.0;
    }

    void endZone(const char* name, Category category, const detail::StackFrame& frame) {
        if (!enabled() || frame.node == SIZE_MAX) return;
        const uint64_t endNs = detail::nowNs();
        const uint32_t threadId = currentThreadId();
        std::lock_guard<std::mutex> lock(mutex_);
        auto& stack = stackFor(threadId);
        if (stack.empty()) return;
        detail::StackFrame top = stack.back();
        stack.pop_back();
        const double durationMs = (endNs - top.startNs) * 1e-6;
        const double selfMs = durationMs - top.childMs;

        ZoneStats& stats = nodes_[top.node].stats;
        stats.name = name;
        stats.category = category;
        stats.count += 1;
        stats.totalMs += durationMs;
        stats.selfMs += selfMs > 0.0 ? selfMs : 0.0;
        stats.minMs = stats.count == 1 ? durationMs : std::min(stats.minMs, durationMs);
        stats.maxMs = std::max(stats.maxMs, durationMs);

        // Attribute this zone's SELF time to its category for the frame
        // breakdown (self time, not wall time, so nested zones do not double
        // count and the per-category percentages sum to ~100%). Then roll the
        // full duration up into the parent's child time so the parent's own
        // self time excludes it.
        if (selfMs > 0.0)
            frameCategoryMs_[static_cast<size_t>(category)] += selfMs;
        if (!stack.empty()) stack.back().childMs += durationMs;

        if (zones_.size() < maxTraceEvents_)
            zones_.push_back({name, category, top.startNs, endNs, threadId,
                              static_cast<uint32_t>(stack.size())});
        else
            ++droppedEvents_;
    }

    // --- instant markers / GPU-CPU sync points --------------------------------
    void syncPoint(const char* name, char scope = 'g') {
        if (!enabled()) return;
        const uint64_t timeNs = detail::nowNs();
        const uint32_t threadId = currentThreadId();
        std::lock_guard<std::mutex> lock(mutex_);
        if (instants_.size() < maxTraceEvents_)
            instants_.push_back({name, timeNs, threadId, scope});
        else
            ++droppedEvents_;
        ++syncPointCount_;
    }

    uint64_t syncPointCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return syncPointCount_;
    }

    // --- memory tracking ------------------------------------------------------
    void trackAllocation(uint64_t bytes) {
        if (!enabled()) return;
        const uint64_t timeNs = detail::nowNs();
        std::lock_guard<std::mutex> lock(mutex_);
        memory_.allocationCount += 1;
        memory_.currentBytes += bytes;
        memory_.totalAllocatedBytes += bytes;
        memory_.peakBytes = std::max(memory_.peakBytes, memory_.currentBytes);
        pushCounterLocked("MemoryBytes", timeNs, static_cast<double>(memory_.currentBytes));
    }

    void trackDeallocation(uint64_t bytes) {
        if (!enabled()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        memory_.deallocationCount += 1;
        if (bytes <= memory_.currentBytes) memory_.currentBytes -= bytes;
        else memory_.currentBytes = 0;
    }

    MemoryStats memoryStats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return memory_;
    }

    // --- aggregation access ---------------------------------------------------
    // Returns the aggregated zone stats, deepest/most-expensive first. The
    // synthetic root (node 0) is excluded.
    std::vector<ZoneStats> report() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<ZoneStats> out;
        out.reserve(nodes_.size());
        for (size_t i = 1; i < nodes_.size(); ++i)
            if (nodes_[i].stats.count > 0) out.push_back(nodes_[i].stats);
        std::sort(out.begin(), out.end(), [](const ZoneStats& a, const ZoneStats& b) {
            return a.totalMs != b.totalMs ? a.totalMs > b.totalMs : a.count > b.count;
        });
        return out;
    }

    // Prints a flat, self-time-sorted table plus the per-frame category
    // breakdown and memory summary to stdout.
    void printReport() const {
        std::vector<ZoneStats> stats = report();
        FrameBreakdown frame = lastBreakdown();
        MemoryStats memory = memoryStats();

        std::printf("\n==== Velox Profiler Report ====\n");
        std::printf("Frame #%llu: %.3f ms\n",
                    static_cast<unsigned long long>(frame.frameIndex), frame.frameMs);

        std::printf("\n-- Frame time breakdown --\n");
        for (size_t i = 0; i < static_cast<size_t>(Category::Count); ++i) {
            if (frame.categoryMs[i] <= 0.0) continue;
            const double pct = frame.frameMs > 0.0
                ? 100.0 * frame.categoryMs[i] / frame.frameMs : 0.0;
            std::printf("  %-16s %9.3f ms  (%5.1f%%)\n",
                        categoryName(static_cast<Category>(i)), frame.categoryMs[i], pct);
        }

        std::printf("\n-- Zones (sorted by total time) --\n");
        std::printf("  %-28s %10s %10s %10s %10s %8s %5s\n",
                    "name", "calls", "total_ms", "self_ms", "avg_ms", "max_ms", "depth");
        for (const ZoneStats& zone : stats) {
            const double avgMs = zone.count ? zone.totalMs / static_cast<double>(zone.count) : 0.0;
            std::printf("  %-28s %10llu %10.3f %10.3f %10.3f %8.3f %5u\n",
                        zone.name, static_cast<unsigned long long>(zone.count),
                        zone.totalMs, zone.selfMs, avgMs, zone.maxMs, zone.maxDepth);
        }

        std::printf("\n-- Memory --\n");
        std::printf("  allocations:   %llu\n",
                    static_cast<unsigned long long>(memory.allocationCount));
        std::printf("  deallocations: %llu\n",
                    static_cast<unsigned long long>(memory.deallocationCount));
        std::printf("  current:       %llu bytes\n",
                    static_cast<unsigned long long>(memory.currentBytes));
        std::printf("  peak:          %llu bytes\n",
                    static_cast<unsigned long long>(memory.peakBytes));
        std::printf("  total:         %llu bytes\n",
                    static_cast<unsigned long long>(memory.totalAllocatedBytes));
        std::printf("  sync points:   %llu\n",
                    static_cast<unsigned long long>(syncPointCount()));
        std::printf("===============================\n\n");
    }

    // --- Chrome Trace Event Format export -------------------------------------
    // Writes a JSON array of trace events loadable in chrome://tracing or
    // https://ui.perfetto.dev. Zones become complete ("X") events, sync points
    // and frame boundaries become instant ("i") events, and memory/frame-time
    // become counter ("C") events.
    bool exportChromeTrace(const char* path) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ofstream file(path);
        if (!file) return false;
        const uint64_t origin = zones_.empty()
            ? (instants_.empty() ? 0 : instants_.front().timeNs)
            : zones_.front().startNs;
        auto tsUs = [origin](uint64_t ns) {
            return (static_cast<double>(ns) - static_cast<double>(origin)) / 1000.0;
        };

        file << "[\n";
        bool first = true;
        auto comma = [&]() { if (!first) file << ",\n"; first = false; };

        // Thread name metadata so the trace shows readable lane labels.
        for (size_t i = 0; i < threadNames_.size(); ++i) {
            comma();
            file << "{\"name\":\"thread_name\",\"ph\":\"M\",\"pid\":0,\"tid\":" << i
                 << ",\"args\":{\"name\":\"" << detail::jsonEscape(threadNames_[i].c_str())
                 << "\"}}";
        }
        // Process label.
        comma();
        file << "{\"name\":\"process_name\",\"ph\":\"M\",\"pid\":0,\"tid\":0,"
             << "\"args\":{\"name\":\"Velox\"}}";

        for (const detail::RawZone& zone : zones_) {
            comma();
            file << "{\"name\":\"" << detail::jsonEscape(zone.name)
                 << "\",\"cat\":\"" << categoryName(zone.category)
                 << "\",\"ph\":\"X\",\"pid\":0,\"tid\":" << zone.threadId
                 << ",\"ts\":" << tsUs(zone.startNs)
                 << ",\"dur\":" << ((zone.endNs - zone.startNs) / 1000.0)
                 << ",\"args\":{\"depth\":" << zone.depth << "}}";
        }
        for (const detail::RawInstant& instant : instants_) {
            comma();
            file << "{\"name\":\"" << detail::jsonEscape(instant.name)
                 << "\",\"cat\":\"sync\",\"ph\":\"i\",\"s\":\"" << instant.scope
                 << "\",\"pid\":0,\"tid\":" << instant.threadId
                 << ",\"ts\":" << tsUs(instant.timeNs) << "}";
        }
        for (const detail::RawCounter& counter : counters_) {
            comma();
            file << "{\"name\":\"" << detail::jsonEscape(counter.name)
                 << "\",\"ph\":\"C\",\"pid\":0,\"tid\":0"
                 << ",\"ts\":" << tsUs(counter.timeNs)
                 << ",\"args\":{\"value\":" << counter.value << "}}";
        }
        file << "\n]\n";
        return static_cast<bool>(file);
    }

    // --- reset ----------------------------------------------------------------
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        zones_.clear();
        instants_.clear();
        counters_.clear();
        nodes_.clear();
        nodes_.push_back(detail::AggNode{}); // re-create the synthetic root
        stacks_.clear();
        memory_ = MemoryStats{};
        lastBreakdown_ = FrameBreakdown{};
        frameCounter_ = 0;
        frameStartNs_ = detail::nowNs();
        for (double& value : frameCategoryMs_) value = 0.0;
        droppedEvents_ = 0;
        syncPointCount_ = 0;
    }

private:
    Profiler() {
        nodes_.push_back(detail::AggNode{}); // node 0: synthetic root
        frameStartNs_ = detail::nowNs();
    }
    Profiler(const Profiler&) = delete;
    Profiler& operator=(const Profiler&) = delete;

    uint32_t currentThreadId() {
        static thread_local uint32_t id = registerThread();
        return id;
    }

    std::vector<detail::StackFrame>& stackFor(uint32_t threadId) {
        if (threadId >= stacks_.size()) stacks_.resize(threadId + 1);
        return stacks_[threadId];
    }

    size_t findOrCreateChild(size_t parent, const char* name, Category category) {
        for (size_t child : nodes_[parent].children)
            if (nodes_[child].stats.category == category &&
                std::strcmp(nodes_[child].name, name) == 0)
                return child;
        detail::AggNode node;
        node.name = name;
        node.category = category;
        node.parent = parent;
        node.stats.name = name;
        node.stats.category = category;
        const size_t index = nodes_.size();
        nodes_.push_back(node);
        nodes_[parent].children.push_back(index);
        return index;
    }

    void pushCounterLocked(const char* name, uint64_t timeNs, double value) {
        if (counters_.size() < maxTraceEvents_)
            counters_.push_back({name, timeNs, value});
        else
            ++droppedEvents_;
    }

    mutable std::mutex mutex_;
    std::atomic<bool> enabled_{false};

    std::vector<detail::RawZone> zones_;
    std::vector<detail::RawInstant> instants_;
    std::vector<detail::RawCounter> counters_;
    std::vector<detail::AggNode> nodes_;
    std::vector<std::vector<detail::StackFrame>> stacks_; // indexed by thread id
    std::vector<std::string> threadNames_;

    uint64_t frameCounter_ = 0;
    uint64_t frameStartNs_ = 0;
    std::array<double, static_cast<size_t>(Category::Count)> frameCategoryMs_ = {};
    FrameBreakdown lastBreakdown_;

    MemoryStats memory_;
    uint64_t syncPointCount_ = 0;
    size_t maxTraceEvents_ = 2000000;
    size_t droppedEvents_ = 0;
};

// RAII zone. Times the enclosing scope and records it into the profiler's
// hierarchy. Inactive (a single relaxed atomic load) while profiling is off.
class ScopedZone {
public:
    ScopedZone(const char* name, Category category)
        : name_(name), category_(category) {
        Profiler::instance().beginZone(name_, category_, frame_);
    }
    ~ScopedZone() {
        Profiler::instance().endZone(name_, category_, frame_);
    }
    ScopedZone(const ScopedZone&) = delete;
    ScopedZone& operator=(const ScopedZone&) = delete;

private:
    const char* name_;
    Category category_;
    detail::StackFrame frame_;
};

// RAII frame guard: beginFrame() on construction, endFrame() on destruction.
class FrameGuard {
public:
    FrameGuard() { Profiler::instance().beginFrame(); }
    ~FrameGuard() { Profiler::instance().endFrame(); }
    FrameGuard(const FrameGuard&) = delete;
    FrameGuard& operator=(const FrameGuard&) = delete;
};

} // namespace profile
} // namespace velox

// --- macros -------------------------------------------------------------------
// Token-paste helpers so __LINE__ expands before concatenation.
#define VELOX_PROFILER_CONCAT_(a, b) a##b
#define VELOX_PROFILER_CONCAT(a, b) VELOX_PROFILER_CONCAT_(a, b)

// Time the current scope as a Custom-category zone.
#define VELOX_PROFILE_SCOPE(name) \
    ::velox::profile::ScopedZone VELOX_PROFILER_CONCAT(velox_zone_, __LINE__)( \
        (name), ::velox::profile::Category::Custom)

// Time the current scope under an explicit category (drives the frame
// breakdown).
#define VELOX_PROFILE_CATEGORY_SCOPE(name, category) \
    ::velox::profile::ScopedZone VELOX_PROFILER_CONCAT(velox_zone_, __LINE__)( \
        (name), (category))

// Time the current scope, named after the enclosing function.
#define VELOX_PROFILE_FUNCTION() \
    ::velox::profile::ScopedZone VELOX_PROFILER_CONCAT(velox_zone_, __LINE__)( \
        __func__, ::velox::profile::Category::Custom)

// Mark the start of a frame; the matching end happens when the guard dies.
#define VELOX_PROFILE_FRAME() \
    ::velox::profile::FrameGuard VELOX_PROFILER_CONCAT(velox_frame_, __LINE__)

// Record a GPU/CPU synchronization point (an instant marker in the trace).
#define VELOX_PROFILE_GPU_SYNC(name) \
    ::velox::profile::Profiler::instance().syncPoint((name), 'g')

// Memory tracking hooks.
#define VELOX_PROFILE_ALLOC(bytes) \
    ::velox::profile::Profiler::instance().trackAllocation((bytes))
#define VELOX_PROFILE_FREE(bytes) \
    ::velox::profile::Profiler::instance().trackDeallocation((bytes))
