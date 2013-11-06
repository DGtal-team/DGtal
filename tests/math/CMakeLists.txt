SET(DGTAL_TESTS_SRC_MATH
       testMeasure
       testSignal 
       testStatistics 
       testHistogram
       testMPolynomial
       testAngleLinearMinimizer
       testBasicMathFunctions)


FOREACH(FILE ${DGTAL_TESTS_SRC_MATH})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
