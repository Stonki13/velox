add_executable(bullet_demo bullet_demo.cpp)
target_link_libraries(bullet_demo PRIVATE velox)

add_executable(hello_velox hello_velox.cpp)
target_link_libraries(hello_velox PRIVATE velox)

add_executable(contact_events contact_events.cpp)
target_link_libraries(contact_events PRIVATE velox)

add_executable(queries_demo queries_demo.cpp)
target_link_libraries(queries_demo PRIVATE velox)

add_executable(multi_threaded multi_threaded.cpp)
target_link_libraries(multi_threaded PRIVATE velox)

add_executable(joints_demo joints_demo.cpp)
target_link_libraries(joints_demo PRIVATE velox)

add_executable(solver_options_demo solver_options_demo.cpp)
target_link_libraries(solver_options_demo PRIVATE velox)

add_executable(ragdoll_demo ragdoll_demo.cpp)
target_link_libraries(ragdoll_demo PRIVATE velox)

add_executable(platform_demo platform_demo.cpp)
target_link_libraries(platform_demo PRIVATE velox)

add_executable(stress_demo stress_demo.cpp)
target_link_libraries(stress_demo PRIVATE velox)

if(BUILD_TESTING)
    add_test(NAME velox.hello COMMAND hello_velox)
    add_test(NAME velox.contact_events COMMAND contact_events)
    add_test(NAME velox.queries COMMAND queries_demo)
    add_test(NAME velox.multi_threaded COMMAND multi_threaded)
    add_test(NAME velox.joints_example COMMAND joints_demo)
    add_test(NAME velox.solver_options COMMAND solver_options_demo)
    add_test(NAME velox.ragdoll COMMAND ragdoll_demo)
    add_test(NAME velox.platform COMMAND platform_demo)
    # Run the bounded real-workload health check before the longer stress
    # process so a failure reports its own scene rather than a late-suite
    # timeout on heavily loaded Windows runners.
    add_test(NAME velox.release_gate COMMAND release_gate)
    add_test(NAME velox.stress COMMAND stress_demo)
    add_test(NAME velox.fuzz COMMAND fuzz_demo 40)
    add_test(NAME velox.soak COMMAND fuzz_demo soak 6000)
    add_test(NAME velox.islands COMMAND islands_demo)
    add_test(NAME velox.vehicle COMMAND vehicle_demo)
    add_test(NAME velox.serialize COMMAND serialize_demo)
    # Every test normally finishes in seconds; a generous cap turns a rare
    # hang into a fast failure instead of a 25-minute default timeout.
    set_tests_properties(velox.release_gate velox.stress velox.fuzz velox.soak
        velox.islands velox.vehicle velox.serialize PROPERTIES TIMEOUT 180)
endif()

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark PRIVATE velox)

add_executable(fuzz_demo fuzz_demo.cpp)
target_link_libraries(fuzz_demo PRIVATE velox)

add_executable(islands_demo islands_demo.cpp)
target_link_libraries(islands_demo PRIVATE velox)

add_executable(determinism_demo determinism_demo.cpp)
target_link_libraries(determinism_demo PRIVATE velox)

add_executable(thread_safety_demo thread_safety_demo.cpp)
target_link_libraries(thread_safety_demo PRIVATE velox)

add_executable(batched_queries_demo batched_queries_demo.cpp)
target_link_libraries(batched_queries_demo PRIVATE velox)

add_executable(runtime_mutation_demo runtime_mutation_demo.cpp)
target_link_libraries(runtime_mutation_demo PRIVATE velox)

add_executable(cuda_recovery_demo cuda_recovery_demo.cpp)
target_link_libraries(cuda_recovery_demo PRIVATE velox)

add_executable(release_gate release_gate.cpp)
target_link_libraries(release_gate PRIVATE velox)

add_executable(vehicle_demo vehicle_demo.cpp)
target_link_libraries(vehicle_demo PRIVATE velox)

add_executable(serialize_demo serialize_demo.cpp)
target_link_libraries(serialize_demo PRIVATE velox)

add_executable(geometry_fuzz geometry_fuzz.cpp)
target_link_libraries(geometry_fuzz PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.geometry_fuzz COMMAND geometry_fuzz 1000)
    add_test(NAME velox.determinism COMMAND determinism_demo)
    add_test(NAME velox.thread_safety COMMAND thread_safety_demo)
    add_test(NAME velox.batched_queries COMMAND batched_queries_demo)
    add_test(NAME velox.runtime_mutation COMMAND runtime_mutation_demo)
    add_test(NAME velox.cuda_recovery COMMAND cuda_recovery_demo)
    set_tests_properties(velox.determinism PROPERTIES TIMEOUT 180)
endif()

add_executable(proto_manifold proto_manifold.cpp)
target_link_libraries(proto_manifold PRIVATE velox)

add_executable(character_demo character_demo.cpp)
target_link_libraries(character_demo PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.character COMMAND character_demo)
endif()

add_executable(profiler_demo profiler_demo.cpp)
target_link_libraries(profiler_demo PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.profiler COMMAND profiler_demo)
endif()

# Debug visualization: exercises every debug-draw layer headlessly and verifies
# each one produces geometry. No third-party dependencies.
add_executable(debug_visualizer debug_visualizer.cpp)
target_link_libraries(debug_visualizer PRIVATE velox)
if(BUILD_TESTING)
    add_test(NAME velox.debug_visualizer COMMAND debug_visualizer)
endif()

# Optional Dear ImGui interactive debug overlay. Off by default so the core
# build never depends on ImGui; enable with -DVELOX_BUILD_IMGUI_EXAMPLE=ON and
# make ImGui discoverable (e.g. -DCMAKE_PREFIX_PATH=<imgui-install>).
option(VELOX_BUILD_IMGUI_EXAMPLE "Build the Dear ImGui debug overlay example" OFF)
if(VELOX_BUILD_IMGUI_EXAMPLE)
    find_package(imgui CONFIG QUIET)
    if(imgui_FOUND)
        add_executable(imgui_debug_example imgui_debug_example.cpp)
        target_link_libraries(imgui_debug_example PRIVATE velox imgui::imgui)
    else()
        message(STATUS
            "VELOX_BUILD_IMGUI_EXAMPLE is ON but ImGui was not found; "
            "the example will build as a dependency-free stub.")
        add_executable(imgui_debug_example imgui_debug_example.cpp)
        target_link_libraries(imgui_debug_example PRIVATE velox)
    endif()
endif()
