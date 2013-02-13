add_subdirectory(estimation)

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
  testFrechetShortcut	
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


