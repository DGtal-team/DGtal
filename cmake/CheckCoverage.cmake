# -----------------------------------------------------------------------------
# Coverage brute-froce discovery
# -----------------------------------------------------------------------------
ADD_CUSTOM_TARGET(lcov)
OPTION(WITH_COVERAGE "Enable lcov code coverage." OFF)
IF (WITH_COVERAGE) 
  MESSAGE(STATUS "Code coverage enabled")
  message(STATUS "-------------------------------------------------------------------------------")
  ADD_CUSTOM_COMMAND(TARGET lcov
    COMMAND mkdir -p coverage
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  ADD_CUSTOM_COMMAND(TARGET lcov
    COMMAND lcov --directory . --capture --output-file ./coverage/stap_all.info --no-checksum
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  ADD_CUSTOM_COMMAND(TARGET lcov
    COMMAND lcov --directory . -r ./coverage/stap_all.info ${REMOVE_PATTERN} --output-file ./coverage/stap.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  ADD_CUSTOM_COMMAND(TARGET lcov
    COMMAND genhtml -o ./coverage --title "Code Coverage" --legend --show-details --demangle-cpp ./coverage/stap.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
  ADD_CUSTOM_COMMAND(TARGET lcov
    COMMAND echo "Open ${CMAKE_BINARY_DIR}/coverage/index.html to view the coverage analysis results."
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
ENDIF()