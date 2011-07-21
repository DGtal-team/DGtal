SET(DGTAL_TESTS_SRC
	testArithDSS
	testFreemanChain	
	testDecomposition
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
	)

FOREACH(FILE ${DGTAL_TESTS_SRC})
	add_executable(${FILE} ${FILE})
	target_link_libraries (${FILE} ${LIBDGTAL_NAME})
	add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
