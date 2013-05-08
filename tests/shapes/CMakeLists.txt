
### Visu QGLViewer

SET(QGLVIEWER_BASED_TESTS_SRC
 testMeshDisplay
   )

if (  WITH_VISU3D_QGLVIEWER )

  FOREACH(FILE ${QGLVIEWER_BASED_TESTS_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal
                            ${DGtalLibDependencies})   
    #add_test(${FILE} ${FILE})
    #not a test since IHM
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )



SET(DGTAL_TESTS_SRC
  testGaussDigitizer
  testHalfPlane
  testImplicitFunctionModels
  testShapesFromPoints
  testMesh
  testBall3DSurface
  testEuclideanShapesDecorator
  testDigitalShapesDecorator
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


##### Shapes with viewer.

SET(QGLVIEWER_SHAPES_TESTS_SRC
  testPolynomial
  testBall3D
  )

if ( WITH_VISU3D_QGLVIEWER )
  FOREACH(FILE ${QGLVIEWER_SHAPES_TESTS_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal
      ${DGtalLibDependencies})
  ENDFOREACH(FILE)
endif ( WITH_VISU3D_QGLVIEWER )
