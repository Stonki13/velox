cmake_minimum_required(VERSION 3.20)
project(velox VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(VELOX_BUILD_EXAMPLES "Build example programs" ON)
option(VELOX_ENABLE_CUDA "Build the CUDA GPU backend if a toolkit is found" ON)
set(VELOX_PACKAGE_NEEDS_CUDA OFF)

add_library(velox
    src/world.cpp
    src/solver.cpp
    src/queries.cpp
)
target_include_directories(velox PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
add_library(velox::velox ALIAS velox)

if(VELOX_ENABLE_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        set(CMAKE_CUDA_STANDARD 17)
        set_property(TARGET velox PROPERTY CUDA_ARCHITECTURES native)
        target_sources(velox PRIVATE src/backend_cuda.cu)
        target_compile_definitions(velox PUBLIC VELOX_HAS_CUDA=1)
        find_package(CUDAToolkit REQUIRED)
        target_link_libraries(velox PUBLIC CUDA::cudart)
        set(VELOX_PACKAGE_NEEDS_CUDA ON)
        message(STATUS "velox: CUDA backend enabled")
    else()
        message(STATUS "velox: no CUDA toolkit found, CPU backend only")
    endif()
endif()

if(MSVC)
    target_compile_options(velox PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/W4 /fp:fast>
        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)
else()
    target_compile_options(velox PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -ffast-math>)
endif()

if(VELOX_BUILD_EXAMPLES)
    add_subdirectory(examples)
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
