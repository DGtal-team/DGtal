add_subdirectory(estimation)

set(DGTAL_TESTS_SRC
  testArithDSS3d
  testFreemanChain
  testSegmentation
  testFP
  testGridCurve
  testCombinDSS
  testStabbingLineComputer
  testStabbingCircleComputer
  testBinomialConvolver
  testFrechetShortcut
  testArithmeticalDSS
  testArithmeticalDSLKernel
  testArithmeticalDSSComputer
  testArithmeticalDSL
  testDSLSubsegment
  testArithDSSIterator
  testArithmeticalDSSConvexHull
  testAlphaThickSegmentComputer
  testParametricCurveDigitization
  )


foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()
