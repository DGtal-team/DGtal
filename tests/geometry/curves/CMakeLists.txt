add_subdirectory(estimation)


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -qstrict")

SET(DGTAL_TESTS_SRC
  # testArithDSS	
  # testArithDSS3d
  # testFreemanChain
  # testDecomposition  
  # testSegmentation
  # testMaximalSegments
  # testFP
  # testGridCurve
  # testCombinDSS
  # testGeometricalDSS
  # testGeometricalDCA
  # testBinomialConvolver
  testDSLSubsegment
  testLrSBreversedSmartDSS
  testSBreversedSmartDSS	
  testArithDSSIterator
	)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


