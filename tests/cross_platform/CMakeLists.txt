# Cross-platform test suite for Velox.
#
# These tests verify bitwise determinism, floating-point consistency,
# endianness portability, and struct alignment across all supported
# platforms (Windows/Linux/macOS, x86_64/ARM64).
#
# They reuse the doctest header from tests/unit/ and link against the
# main velox library.

# Reuse the doctest header downloaded by the unit test suite.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../unit/doctest.h")
    message(STATUS "velox: downloading doctest single header for cross-platform tests")
    file(DOWNLOAD
        "https://raw.githubusercontent.com/doctest/doctest/v2.4.11/doctest/doctest.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../unit/doctest.h"
        STATUS download_status)
    list(GET download_status 0 status_code)
    if(NOT status_code EQUAL 0)
        message(FATAL_ERROR "Failed to download doctest.h")
    endif()
endif()

function(velox_add_cross_platform_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE velox)
    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/tests/unit)
    add_test(NAME velox.xplatform.${name} COMMAND ${name})
    set_tests_properties(velox.xplatform.${name} PROPERTIES TIMEOUT 60)
endfunction()

velox_add_cross_platform_test(test_determinism)
velox_add_cross_platform_test(test_floating_point)
velox_add_cross_platform_test(test_endianness)
velox_add_cross_platform_test(test_alignment)
