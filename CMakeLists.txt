cmake_minimum_required(VERSION 3.20)
include(cmake/version.cmake)
velox_read_version("${CMAKE_CURRENT_SOURCE_DIR}/include/velox/version.h" VELOX_VERSION)
project(velox VERSION ${VELOX_VERSION} LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Keep Windows shared-library executables beside velox.dll. This makes CTest
# and direct example launches work without requiring callers to modify PATH.
if(WIN32 AND BUILD_SHARED_LIBS)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")
endif()

option(VELOX_BUILD_EXAMPLES "Build example programs" ON)
option(VELOX_BUILD_BENCHMARKS "Build the performance benchmark suite" ON)
option(VELOX_BUILD_SANDBOX "Build the optional GLFW/Vulkan interactive sandbox" OFF)
option(VELOX_BUILD_DIFFTEST "Build differential tests against Jolt Physics" OFF)
option(VELOX_ENABLE_CUDA "Build the optional NVIDIA CUDA GPU backend if a toolkit is found" ON)
set(VELOX_CUDA_ARCHITECTURES "native" CACHE STRING
    "CUDA architectures passed to CMake when the CUDA backend is enabled")
option(VELOX_STRICT_FLOATING_POINT
       "Build the CPU reference path with reproducible floating-point semantics" OFF)
set(VELOX_PACKAGE_NEEDS_CUDA OFF)
set(VELOX_HAS_CUDA 0)

add_library(velox
    src/world.cpp
    src/solver.cpp
    src/queries.cpp
    src/debug.cpp
    src/character.cpp
    src/ragdoll.cpp
    src/vehicle.cpp
    src/serialization.cpp
    src/serialization_v2.cpp
    src/benchmark.cpp
    src/velox_c.cpp
    src/stable_api.cpp
    src/manifold.cpp
    src/sleep.cpp
)
if(WIN32 AND BUILD_SHARED_LIBS)
    # The C++ API is header-declared rather than individually annotated with
    # VELOX_API. Generate the import library from its public implementation
    # symbols so installed shared packages link correctly on Windows.
    set_target_properties(velox PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()
target_include_directories(velox PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
add_library(velox::velox ALIAS velox)
target_link_libraries(velox PUBLIC Threads::Threads)

if(VELOX_ENABLE_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        set(CMAKE_CUDA_STANDARD 17)
        set_property(TARGET velox PROPERTY CUDA_ARCHITECTURES ${VELOX_CUDA_ARCHITECTURES})
        target_sources(velox PRIVATE src/backend_cuda.cu)
        # backend_cuda.cu is template-heavy. CUDA 12+ can parallelize both its
        # device optimizer and ptxas without changing program semantics.
        if(CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0)
            target_compile_options(velox PRIVATE
                $<$<COMPILE_LANGUAGE:CUDA>:--split-compile=0>
                $<$<COMPILE_LANGUAGE:CUDA>:-Xptxas=--split-compile=0>)
        endif()
        set(VELOX_HAS_CUDA 1)
        find_package(CUDAToolkit REQUIRED)
        target_link_libraries(velox PUBLIC CUDA::cudart)
        set(VELOX_PACKAGE_NEEDS_CUDA ON)
        message(STATUS "velox: CUDA backend enabled")
    else()
        message(STATUS "velox: no CUDA toolkit found, CPU backend only")
    endif()
endif()

# Consumers and implementation files must see an explicit value in CPU-only
# builds too. Undefined preprocessor symbols silently evaluate to zero and can
# conceal an accidental CUDA-only dependency.
target_compile_definitions(velox PUBLIC VELOX_HAS_CUDA=${VELOX_HAS_CUDA})

# Symbol export/import for shared builds (see include/velox/version.h). The
# library itself is compiled with VELOX_BUILDING_SHARED so its public symbols
# are exported; consumers linking the installed shared package receive
# VELOX_USING_SHARED so the headers switch to dllimport. Both are gated on the
# resolved target type, so static builds (the default for Conan and for static
# vcpkg triplets) are unaffected and the same install rules serve both package
# managers and both library types.
if(BUILD_SHARED_LIBS)
    target_compile_definitions(velox PRIVATE VELOX_BUILDING_SHARED)
    target_compile_definitions(velox INTERFACE VELOX_USING_SHARED)
endif()

include(cmake/determinism.cmake)
velox_configure_floating_point(velox ${VELOX_STRICT_FLOATING_POINT})
include(cmake/platform.cmake)
velox_configure_platform(velox)

if(VELOX_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(VELOX_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

if(VELOX_BUILD_SANDBOX)
    add_subdirectory(examples/sandbox)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests/unit)
    add_subdirectory(tests/cross_platform)
endif()

if(VELOX_BUILD_DIFFTEST)
    add_subdirectory(tests/difftest)
endif()

install(TARGETS velox EXPORT VeloxTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/velox DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
configure_package_config_file(
    cmake/VeloxConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/VeloxConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Velox)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/VeloxConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(EXPORT VeloxTargets
    FILE VeloxTargets.cmake
    NAMESPACE velox::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Velox)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/VeloxConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/VeloxConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Velox)
