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


if (  WITH_VISU3D_QGLVIEWER )
  set(QGLVIEWER_TESTS_SRC
    testLocalConvolutionNormalVectorEstimator
    testTensorVotingViewer)

  foreach(FILE ${QGLVIEWER_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()
endif()



if ( WITH_PATATE )
    set(PATATE_TESTS_SRC
      testSphereFitting )
    foreach(FILE ${PATATE_TESTS_SRC})
      DGtal_add_test(${FILE})
    endforeach()
endif()

#-----------------------
#GMP based tests
#----------------------
set(DGTAL_TESTS_GMP_SRC
  testCOBANaivePlaneComputer
  testCOBAGenericStandardPlaneComputer
  testChordNaivePlaneComputer
  )

if(GMP_FOUND)
  foreach(FILE ${DGTAL_TESTS_GMP_SRC})
    DGtal_add_test(${FILE})
  endforeach()
endif()

set(DGTAL_BENCH_GMP_SRC
  testCOBANaivePlaneComputer-benchmark
  testCOBAGenericNaivePlaneComputer-benchmark
  testChordNaivePlaneComputer-benchmark
  )

#Benchmark target
if(GMP_FOUND)
    foreach(FILE ${DGTAL_BENCH_GMP_SRC})
      DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
    endforeach()
endif()
