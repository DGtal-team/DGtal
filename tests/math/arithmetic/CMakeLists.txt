SET(DGTAL_TESTS_SRC_ARITH
       testModuloComputer)

FOREACH(FILE ${DGTAL_TESTS_SRC_ARITH})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

