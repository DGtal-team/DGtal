SET(DGTAL_TESTS_SRC_ARITH
       testModuloComputer )

FOREACH(FILE ${DGTAL_TESTS_SRC_ARITH})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

#-----------------------
#GMP based tests
#----------------------
SET(DGTAL_TESTS_GMP_SRC 
    testIntegerComputer
    testSternBrocot )

IF(GMP_FOUND)
  FOREACH(FILE ${DGTAL_TESTS_GMP_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
    add_test(${FILE} ${FILE})
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)

