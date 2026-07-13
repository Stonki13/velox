cmake_minimum_required(VERSION 3.20)
project(velox VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(VELOX_BUILD_EXAMPLES "Build example programs" ON)
option(VELOX_ENABLE_CUDA "Build the CUDA GPU backend (requires CUDA toolkit)" OFF)

add_library(velox
    src/world.cpp
    src/ccd.cpp
    src/solver.cpp
)
target_include_directories(velox PUBLIC include)

if(VELOX_ENABLE_CUDA)
    enable_language(CUDA)
    target_sources(velox PRIVATE src/backend_cuda.cu)
    target_compile_definitions(velox PUBLIC VELOX_HAS_CUDA=1)
endif()

if(MSVC)
    target_compile_options(velox PRIVATE /W4 /fp:fast)
else()
    target_compile_options(velox PRIVATE -Wall -Wextra -ffast-math)
endif()

if(VELOX_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
