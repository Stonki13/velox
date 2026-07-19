cmake_minimum_required(VERSION 3.20)
project(velox_package_consumer LANGUAGES CXX)

find_package(Velox CONFIG REQUIRED)

add_executable(velox_package_consumer main.cpp)
target_link_libraries(velox_package_consumer PRIVATE velox::velox)
target_compile_features(velox_package_consumer PRIVATE cxx_std_17)

include(CTest)
if(BUILD_TESTING)
    add_test(NAME velox.package_consumer COMMAND velox_package_consumer)
endif()
