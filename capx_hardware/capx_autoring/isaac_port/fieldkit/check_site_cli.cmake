file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt" "Offline test fixture, no hardware commissioning.\n")
foreach(limit 0 0.00009 0.020001 nan)
  execute_process(COMMAND "${PROGRAM}" --execute-csv INVALID_ROBOT_ADDRESS 0 0
    "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt" "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt"
    unused.csv unused-path.csv --tracking-error-rad "${limit}"
    RESULT_VARIABLE guard_code ERROR_VARIABLE guard_error)
  if(guard_code EQUAL 0 OR NOT guard_error MATCHES "Tracking error guard|Finite")
    message(FATAL_ERROR "Invalid tracking guard was not rejected before robot connection: ${guard_error}")
  endif()
endforeach()
execute_process(COMMAND "${PROGRAM}" --execute-jog 192.0.2.1 0 0
  "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt" "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt" unused.csv 7 0.2 3 RESULT_VARIABLE code)
if(code EQUAL 0)
  message(FATAL_ERROR "Invalid jog was accepted")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/stationary.csv" "time_s,q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6,ddq7\n0,0,-0.7,0,-2.1,0,1.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n3,0,-0.7,0,-2.1,0,1.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n")
execute_process(COMMAND "${PROGRAM}" --validate-csv "${CMAKE_CURRENT_BINARY_DIR}/stationary.csv" RESULT_VARIABLE code OUTPUT_VARIABLE out)
if(NOT code EQUAL 0 OR NOT out MATCHES "continuous_10percent_pass")
  message(FATAL_ERROR "Offline stationary validation failed: ${out}")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/invalid.csv" "0,1,2\n")
execute_process(COMMAND "${PROGRAM}" --validate-csv "${CMAKE_CURRENT_BINARY_DIR}/invalid.csv" RESULT_VARIABLE code)
if(code EQUAL 0)
  message(FATAL_ERROR "Invalid CSV was accepted")
endif()

# The first1mm Cartesian commissioning test must accept the SAME explicit
# stricter guard as CSV. Invalid limits reject before opening a robot socket.
foreach(limit 0 0.00009 0.020001 nan)
  execute_process(COMMAND "${PROGRAM}" --execute-cartesian INVALID_ROBOT_ADDRESS 0 0
    "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt" "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt"
    unused.csv 0 0 0.001 4 --tracking-error-rad "${limit}"
    RESULT_VARIABLE guard_code ERROR_VARIABLE guard_error)
  if(guard_code EQUAL 0 OR NOT guard_error MATCHES "Tracking error guard|Finite")
    message(FATAL_ERROR "Invalid Cartesian tracking guard was not rejected before connection: ${guard_error}")
  endif()
endforeach()
execute_process(COMMAND "${PROGRAM}" --execute-cartesian INVALID_ROBOT_ADDRESS 0 0
  "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt" "${CMAKE_CURRENT_BINARY_DIR}/test-record.txt"
  unused.csv nan 0 0 4 --tracking-error-rad 0.005
  RESULT_VARIABLE valid_profile_code ERROR_VARIABLE valid_profile_error)
if(valid_profile_code EQUAL 0 OR NOT valid_profile_error MATCHES "Finite")
  message(FATAL_ERROR "5mrad commissioning option did not reach preconnection target validation: ${valid_profile_error}")
endif()
