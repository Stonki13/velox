include(FetchContent)

# Jolt Physics as the differential reference engine. Fetched only when
# VELOX_BUILD_DIFFTEST=ON; keep options minimal and match velox's /MD runtime.
set(CROSS_PLATFORM_DETERMINISTIC ON CACHE BOOL "" FORCE)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_DEBUG_RENDERER OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.2.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build
)
FetchContent_MakeAvailable(JoltPhysics)

add_executable(velox_difftest
    main.cpp
    compare.cpp
    scene_library.cpp
    velox_runner.cpp
    jolt_runner.cpp
)
target_link_libraries(velox_difftest PRIVATE velox Jolt)
target_compile_features(velox_difftest PRIVATE cxx_std_17)

add_executable(velox_difftest_diag EXCLUDE_FROM_ALL
    diag.cpp
    compare.cpp
    scene_library.cpp
    velox_runner.cpp
    jolt_runner.cpp
)
target_link_libraries(velox_difftest_diag PRIVATE velox Jolt)

if(BUILD_TESTING)
    add_test(NAME velox.difftest COMMAND velox_difftest)
endif()
