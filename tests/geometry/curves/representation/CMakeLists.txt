SET(DGTAL_TESTS_SRC
  testArithDSS
	#testArithDSS3d
  testFreemanChain
  testDecomposition  
  testSegmentation
  testMaximalSegments
  testFP
  testGridCurve
  testCombinDSS
  testGeometricalDSS
  testGeometricalDCA
	)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


### Test with QGLViewer
if (  WITH_VISU3D_QGLVIEWER )
   SET(DGTAL_TESTS_QGL_SRC
#  testArithDSS3dViewer
   )
  FOREACH(FILE ${DGTAL_EXAMPLES_QGL_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal)   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )
