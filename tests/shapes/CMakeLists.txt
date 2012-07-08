SET(DGTAL_TESTS_SRC
  testGaussDigitizer
  testHalfPlane
  testImplicitFunctionModels
  testShapesFromPoints
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
