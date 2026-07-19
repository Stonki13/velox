function(velox_configure_floating_point target strict)
    if(strict)
        target_compile_definitions(${target} PUBLIC VELOX_STRICT_FLOATING_POINT=1)
        if(MSVC)
            target_compile_options(${target} PRIVATE
                $<$<COMPILE_LANGUAGE:CXX>:/W4 /fp:precise>
                $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>
                $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang|IntelLLVM")
            target_compile_options(${target} PRIVATE
                $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -fno-math-errno
                                              -fno-fast-math -ffp-contract=off>
                $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
        else()
            message(WARNING "velox: strict floating-point flags are not configured for ${CMAKE_CXX_COMPILER_ID}")
        endif()
        message(STATUS "velox: strict floating-point mode enabled (CPU reference determinism)")
        return()
    endif()

    target_compile_definitions(${target} PUBLIC VELOX_STRICT_FLOATING_POINT=0)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:/W4 /fp:fast>
            $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang|IntelLLVM")
        # NOT -ffast-math: it implies -ffinite-math-only, which folds
        # std::isfinite to a constant and silently deletes every NaN/Inf
        # validation path. Keep this free of -march/-mtune so one CPU build
        # runs on both Intel and AMD hosts.
        target_compile_options(${target} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -fno-math-errno -ffp-contract=fast>)
    else()
        message(STATUS "velox: no compiler-specific floating-point flags for ${CMAKE_CXX_COMPILER_ID}")
    endif()
endfunction()
