#pragma once
#include <cstddef>
#include <functional>

namespace velox {

// External task system interface for game engine integration.
// Implement this to use your engine's job scheduler instead of Velox's
// internal worker pool. This is critical for engines like Unreal or Godot
// that have their own task graphs and want to avoid thread oversubscription.
class TaskSystem {
public:
    virtual ~TaskSystem() = default;

    // Execute a function that processes items in [begin, end).
    // The implementation may parallelize this across its own thread pool.
    // chunkSize is a hint for work distribution granularity.
    virtual void parallelFor(size_t begin, size_t end, size_t chunkSize,
                             const std::function<void(size_t, size_t)>& fn) = 0;

    // Return the number of worker threads available.
    virtual uint32_t workerCount() const = 0;
};

} // namespace velox
