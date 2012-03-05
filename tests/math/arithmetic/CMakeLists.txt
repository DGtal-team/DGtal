SET(DGTAL_TESTS_SRC_ARITH
       testModuloComputer
       testIntegerComputer )

FOREACH(FILE ${DGTAL_TESTS_SRC_ARITH})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

