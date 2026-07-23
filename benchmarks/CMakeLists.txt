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

# Stamp the current commit into every benchmark binary so JSON/CSV reports
# carry the source revision they were captured against (Phase 2 competitive
# plan: no performance claim without hardware + commit provenance). Falls
# back to the framework's own "unknown" default outside a git checkout.
find_package(Git QUIET)
set(VELOX_GIT_COMMIT "unknown")
if(Git_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE VELOX_GIT_COMMIT_OUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE VELOX_GIT_COMMIT_RESULT)
    if(VELOX_GIT_COMMIT_RESULT EQUAL 0 AND NOT VELOX_GIT_COMMIT_OUT STREQUAL "")
        set(VELOX_GIT_COMMIT ${VELOX_GIT_COMMIT_OUT})
    endif()
endif()

foreach(bench IN LISTS VELOX_BENCHMARKS)
    add_executable(${bench} ${bench}.cpp)
    target_link_libraries(${bench} PRIVATE velox)
    # The framework's memory tracker uses GetProcessMemoryInfo on Windows,
    # which lives in psapi.
    target_link_libraries(${bench} PRIVATE $<$<PLATFORM_ID:Windows>:psapi>)
    target_compile_definitions(${bench} PRIVATE VELOX_GIT_COMMIT="${VELOX_GIT_COMMIT}")
endforeach()
