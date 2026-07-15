include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw)

find_package(OpenGL REQUIRED)

add_executable(velox_sandbox
    main.cpp
    window.cpp
    input.cpp
    renderer.cpp
)

target_include_directories(velox_sandbox PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(velox_sandbox PRIVATE velox glfw OpenGL::GL)
target_compile_features(velox_sandbox PRIVATE cxx_std_17)

add_custom_command(TARGET velox_sandbox POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/scenes.json
        $<TARGET_FILE_DIR:velox_sandbox>/scenes.json
)

if(BUILD_TESTING)
    add_test(NAME velox.sandbox COMMAND velox_sandbox --selftest)
    set_tests_properties(velox.sandbox PROPERTIES
        WORKING_DIRECTORY $<TARGET_FILE_DIR:velox_sandbox>)
endif()
