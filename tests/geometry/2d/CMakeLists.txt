SET(DGTAL_TESTS_SRC
  testArithDSS
  testFreemanChain
  testDecomposition  
  testSegmentation
  testHalfPlane
  testPreimage
  testMaximalSegments
  testFP
  testGridCurve
  testLengthEstimators
  testTrueLocalEstimator
  testSegmentComputerFunctor
  testMostCenteredMSEstimator
  testBinomialConvolver
  testCombinDSS
  testShapesFromPoints
  testGeometricalDSS
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
