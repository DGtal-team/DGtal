
SET(QGLVIEWER_TESTS_SRC
  testLocalConvolutionNormalVectorEstimator
  )

if (  WITH_VISU3D_QGLVIEWER )

   FOREACH(FILE ${QGLVIEWER_TESTS_SRC})
     add_executable(${FILE} ${FILE})
     target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
   ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )
