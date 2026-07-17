add_executable(bullet_demo bullet_demo.cpp)
target_link_libraries(bullet_demo PRIVATE velox)

add_executable(stress_demo stress_demo.cpp)
target_link_libraries(stress_demo PRIVATE velox)

if(BUILD_TESTING)
    add_test(NAME velox.stress COMMAND stress_demo)
    add_test(NAME velox.fuzz COMMAND fuzz_demo 40)
    add_test(NAME velox.soak COMMAND fuzz_demo soak 6000)
    add_test(NAME velox.islands COMMAND islands_demo)
    add_test(NAME velox.vehicle COMMAND vehicle_demo)
endif()

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PRIVATE velox)

add_executable(fuzz_demo fuzz_demo.cpp)
target_link_libraries(fuzz_demo PRIVATE velox)

add_executable(islands_demo islands_demo.cpp)
target_link_libraries(islands_demo PRIVATE velox)

add_executable(vehicle_demo vehicle_demo.cpp)
target_link_libraries(vehicle_demo PRIVATE velox)

add_executable(geometry_fuzz geometry_fuzz.cpp)
target_link_libraries(geometry_fuzz PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.geometry_fuzz COMMAND geometry_fuzz 1000)
endif()

add_executable(proto_manifold proto_manifold.cpp)
target_link_libraries(proto_manifold PRIVATE velox)

add_executable(character_demo character_demo.cpp)
target_link_libraries(character_demo PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.character COMMAND character_demo)
endif()
