set(DGTAL_TESTS_SRC
  testLengthEstimators
  testTrueLocalEstimator
  testEstimatorComparator
  testSegmentComputerEstimators
  testMostCenteredMSEstimator
  testLambdaMST2D
  testLambdaMST3D
  testLambdaMST3DBy2D
  )

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()
