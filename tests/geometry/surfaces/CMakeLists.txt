
SET(TESTS_SURFACES_SRC
 testNormalVectorEstimatorEmbedder
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
