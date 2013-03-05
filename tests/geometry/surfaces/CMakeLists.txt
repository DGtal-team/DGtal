
SET(TESTS_SURFACES_SRC
 testNormalVectorEstimatorEmbedder
 testIntegralInvariantCurvatureEstimator2D
 testIntegralInvariantMeanCurvatureEstimator3D
 testIntegralInvariantGaussianCurvatureEstimator3D
)

FOREACH(FILE ${TESTS_SURFACES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
ENDFOREACH(FILE)




if (  WITH_VISU3D_QGLVIEWER )

SET(QGLVIEWER_TESTS_SRC
  testLocalConvolutionNormalVectorEstimator
  )

   FOREACH(FILE ${QGLVIEWER_TESTS_SRC})
     add_executable(${FILE} ${FILE})
     target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
   ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )

#-----------------------
#GMP based tests
#----------------------
SET(DGTAL_TESTS_GMP_SRC 
    testCOBANaivePlane
 )

IF(GMP_FOUND)
  FOREACH(FILE ${DGTAL_TESTS_GMP_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
    add_test(${FILE} ${FILE})
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)

SET(DGTAL_BENCH_GMP_SRC
   testCOBANaivePlane-benchmark
   testCOBAGenericNaivePlane-benchmark
)


#Benchmark target
IF(GMP_FOUND)
  FOREACH(FILE ${DGTAL_BENCH_GMP_SRC})
    add_executable(${FILE} ${FILE}) 
    target_link_libraries (${FILE} DGtal DGtalIO)
    add_custom_target(${FILE}-benchmark COMMAND ${FILE} ">benchmark-${FILE}.txt" )
    ADD_DEPENDENCIES(benchmark ${FILE}-benchmark)
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)
