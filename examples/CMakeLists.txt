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
    add_test(NAME velox.serialize COMMAND serialize_demo)
    # Every test normally finishes in seconds; a generous cap turns a rare
    # hang into a fast failure instead of a 25-minute default timeout.
    set_tests_properties(velox.stress velox.fuzz velox.soak velox.islands
        velox.vehicle velox.serialize PROPERTIES TIMEOUT 180)
endif()

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PRIVATE velox)

add_executable(fuzz_demo fuzz_demo.cpp)
target_link_libraries(fuzz_demo PRIVATE velox)

add_executable(islands_demo islands_demo.cpp)
target_link_libraries(islands_demo PRIVATE velox)

add_executable(determinism_demo determinism_demo.cpp)
target_link_libraries(determinism_demo PRIVATE velox)

add_executable(vehicle_demo vehicle_demo.cpp)
target_link_libraries(vehicle_demo PRIVATE velox)

add_executable(serialize_demo serialize_demo.cpp)
target_link_libraries(serialize_demo PRIVATE velox)

add_executable(geometry_fuzz geometry_fuzz.cpp)
target_link_libraries(geometry_fuzz PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.geometry_fuzz COMMAND geometry_fuzz 1000)
    add_test(NAME velox.determinism COMMAND determinism_demo)
    set_tests_properties(velox.determinism PROPERTIES TIMEOUT 180)
endif()

add_executable(proto_manifold proto_manifold.cpp)
target_link_libraries(proto_manifold PRIVATE velox)

add_executable(character_demo character_demo.cpp)
target_link_libraries(character_demo PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.character COMMAND character_demo)
endif()
