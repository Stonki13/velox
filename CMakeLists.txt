cmake_minimum_required(VERSION 3.20)
project(velox VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(VELOX_BUILD_EXAMPLES "Build example programs" ON)
option(VELOX_ENABLE_CUDA "Build the CUDA GPU backend if a toolkit is found" ON)

add_library(velox
    src/world.cpp
    src/solver.cpp
    src/queries.cpp
)
target_include_directories(velox PUBLIC include)

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
        message(STATUS "velox: CUDA backend enabled")
    else()
        message(STATUS "velox: no CUDA toolkit found, CPU backend only")
    endif()
endif()

if(MSVC)
    target_compile_options(velox PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/W4 /fp:fast>)
else()
    target_compile_options(velox PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -ffast-math>)
endif()

if(VELOX_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
