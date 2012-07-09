SET(DGTAL_TESTS_SRC
  testLengthEstimators
  testTrueLocalEstimator
  testSegmentComputerEstimators
  testMostCenteredMSEstimator
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
