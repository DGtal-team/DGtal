add_subdirectory(estimation)

set(DGTAL_TESTS_SRC
  testArithDSS3d
  testFreemanChain
  testSegmentation
  testSegmentComputerCategories
  testSegmentComputerHelpers
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
  testArithmeticalDSSComputerOnArrows
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
