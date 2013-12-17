add_subdirectory(estimation)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g")

SET(DGTAL_TESTS_SRC
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
  )


FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


