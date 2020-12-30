# -----------------------------------------------------------------------------
# Coverage brute-froce discovery
# -----------------------------------------------------------------------------
add_custom_target(lcov)
option(WITH_COVERAGE "Enable lcov code coverage." OFF)
if (WITH_COVERAGE)
  message(STATUS "Code coverage enabled")
  message(STATUS "-------------------------------------------------------------------------------")
  set(DGTAL_PATTERN "*/src/DGtal/*")
  add_custom_command(TARGET lcov
    COMMAND mkdir -p coverage
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  add_custom_command(TARGET lcov
    COMMAND lcov --directory . --capture --output-file ./coverage/stap_all.info --no-checksum
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  add_custom_command(TARGET lcov
    COMMAND lcov --directory . --extract ./coverage/stap_all.info ${DGTAL_PATTERN} --output-file ./coverage/stap.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endif()
