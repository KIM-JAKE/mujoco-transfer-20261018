# Invalid argument shapes must fail before any FCI connection is attempted.
execute_process(COMMAND "${PROGRAM}" invalid.invalid --not-a-model-mode RESULT_VARIABLE R ERROR_VARIABLE E TIMEOUT 2)
if(NOT R EQUAL 2 OR NOT E MATCHES "Usage")
  message(FATAL_ERROR "read_state invalid-mode preconnection parser failed: ${R} ${E}")
endif()
execute_process(COMMAND "${PROGRAM}" invalid.invalid --with-model extra RESULT_VARIABLE R ERROR_VARIABLE E TIMEOUT 2)
if(NOT R EQUAL 2 OR NOT E MATCHES "Usage")
  message(FATAL_ERROR "read_state extra-argument preconnection parser failed: ${R} ${E}")
endif()
