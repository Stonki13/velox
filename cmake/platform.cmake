# Centralized platform policy. Architecture selection remains the compiler's
# responsibility: forcing -mcpu=apple-m1 would make a package built on CI fail
# on newer or older Apple Silicon targets.
function(velox_configure_platform target)
    if(APPLE)
        target_compile_definitions(${target} PUBLIC VELOX_CMAKE_PLATFORM_MACOS=1)
        message(STATUS "velox: macOS portable CPU build (CUDA disabled when unavailable)")
    elseif(WIN32)
        target_compile_definitions(${target} PUBLIC VELOX_CMAKE_PLATFORM_WINDOWS=1)
    elseif(UNIX)
        target_compile_definitions(${target} PUBLIC VELOX_CMAKE_PLATFORM_LINUX=1)
    endif()
endfunction()
