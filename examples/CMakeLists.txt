add_executable(bullet_demo bullet_demo.cpp)
target_link_libraries(bullet_demo PRIVATE velox)

add_executable(stress_demo stress_demo.cpp)
target_link_libraries(stress_demo PRIVATE velox)

if(BUILD_TESTING)
    add_test(NAME velox.stress COMMAND stress_demo)
endif()

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PRIVATE velox)
