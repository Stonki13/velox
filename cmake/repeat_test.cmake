if(NOT DEFINED PROGRAM OR NOT EXISTS "${PROGRAM}")
    message(FATAL_ERROR "PROGRAM must name an existing executable")
endif()

if(NOT DEFINED REPETITIONS)
    set(REPETITIONS 1)
endif()

foreach(iteration RANGE 1 ${REPETITIONS})
    execute_process(COMMAND "${PROGRAM}" RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${PROGRAM} failed on repeat ${iteration} with ${result}")
    endif()
endforeach()
