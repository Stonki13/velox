# Velox benchmark suite.
#
# Each benchmark is a standalone executable sharing the header-only framework
# (benchmark_framework.h), the scene builders (benchmark_scenes.h), and the
# CLI/output helper (benchmark_cli.h). They are driven uniformly by
# scripts/run_benchmarks.py, which collects their --json output.

set(VELOX_BENCHMARKS
    benchmark_broadphase
    benchmark_narrowphase
    benchmark_solver
    benchmark_queries
    benchmark_scaling)

foreach(bench IN LISTS VELOX_BENCHMARKS)
    add_executable(${bench} ${bench}.cpp)
    target_link_libraries(${bench} PRIVATE velox)
    # The framework's memory tracker uses GetProcessMemoryInfo on Windows,
    # which lives in psapi.
    target_link_libraries(${bench} PRIVATE $<$<PLATFORM_ID:Windows>:psapi>)
endforeach()
