set(TESTS_SRC
  testArithmeticalDSSComputerOnSurfels
  testChordGenericStandardPlaneComputer
  testDigitalPlanePredicate
  testPlaneProbingTetrahedronEstimator
  testPlaneProbingParallelepipedEstimator
  )

foreach(FILE ${TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()

set(TESTS_SURFACES_SRC
  testIntegralInvariantShortcuts
  testNormalVectorEstimatorEmbedder
  testIntegralInvariantVolumeEstimator
  testIntegralInvariantCovarianceEstimator
  testLocalEstimatorFromFunctorAdapter
  testVoronoiCovarianceMeasureOnSurface
  testTensorVoting
  testEstimatorCache
  testSphericalHoughNormalVectorEstimator
  testDigitalSurfaceRegularization
  testShroudsRegularization
  )

foreach(FILE ${TESTS_SURFACES_SRC})
  DGtal_add_test(${FILE})
endforeach()


if (   DGTAL_WITH_CGAL )
  set(CGAL_TESTS_SRC
    testMonge )
  foreach(FILE ${CGAL_TESTS_SRC})
    DGtal_add_test(${FILE})
  endforeach()
endif()


if (  DGTAL_WITH_POLYSCOPE )
  set(POLYSCOPE_VIEWER_TESTS_SRC
    testLocalConvolutionNormalVectorEstimator
    testTensorVotingViewer)

  foreach(FILE ${POLYSCOPE_VIEWER_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()
endif()



if ( DGTAL_WITH_PONCA )
    set(PONCA_TESTS_SRC
      testSphereFitting )
    foreach(FILE ${PONCA_TESTS_SRC})
      DGtal_add_test(${FILE})
    endforeach()
endif()

set(DGTAL_TESTS_BIGINTEGERS
  testCOBANaivePlaneComputer
  testCOBAGenericStandardPlaneComputer
  testChordNaivePlaneComputer
 )

foreach(FILE ${DGTAL_TESTS_BIGINTEGERS})
  DGtal_add_test(${FILE})
endforeach()
