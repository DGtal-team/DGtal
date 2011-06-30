SET(DGTAL_TESTS_SRC
	testArithDSS
	testFreemanChain	
	testDecomposition
	testHalfPlane
	testPreimage
	testMaximalSegments
	testFP
	testCombinDSS
	testGridCurve
	testSegmentComputerFunctor
  testMostCenteredMSEstimator
	testL1LengthEstimator
	)

FOREACH(FILE ${DGTAL_TESTS_SRC})
	add_executable(${FILE} ${FILE})
	target_link_libraries (${FILE} ${LIBDGTAL_NAME})
	add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
