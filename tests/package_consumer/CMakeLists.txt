cmake_minimum_required(VERSION 3.20)
project(velox_package_consumer LANGUAGES CXX)

find_package(Velox CONFIG REQUIRED)

add_executable(velox_package_consumer main.cpp)
target_link_libraries(velox_package_consumer PRIVATE velox::velox)
target_compile_features(velox_package_consumer PRIVATE cxx_std_17)

include(CTest)
if(BUILD_TESTING)
    if(WIN32)
        add_test(NAME velox.package_consumer
            COMMAND ${CMAKE_COMMAND} -E env
                "PATH=$<TARGET_FILE_DIR:velox::velox>;$ENV{PATH}"
                $<TARGET_FILE:velox_package_consumer>)
    else()
        add_test(NAME velox.package_consumer COMMAND velox_package_consumer)
    endif()
endif()
