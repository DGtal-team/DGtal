set(DGTAL_TESTS_SRC
  testMeasureSet
  testVoronoiCovarianceMeasure
  )

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()
