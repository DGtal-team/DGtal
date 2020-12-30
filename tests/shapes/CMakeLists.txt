
### Visu QGLViewer

set(QGLVIEWER_BASED_TESTS_SRC
 testMeshDisplay
   )

if (  WITH_VISU3D_QGLVIEWER )

  foreach(FILE ${QGLVIEWER_BASED_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
    #not a test since IHM
  endforeach()

endif()



set(DGTAL_TESTS_SRC
  testGaussDigitizer
  testHalfPlane
  testImplicitFunctionModels
  testShapesFromPoints
  testMesh
  testMeshVoxelization
  testBall3DSurface
  testEuclideanShapesDecorator
  testDigitalShapesDecorator
  testTriangulatedSurface
  testPolygonalSurface
  testSurfaceMesh
  testProjection
  testShapeMoveCenter
  testAstroid2D
  testLemniscate2D
  )

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()


##### Shapes with viewer.

set(QGLVIEWER_SHAPES_TESTS_SRC
#  testPolynomial
  testBall3D
  )

if ( WITH_VISU3D_QGLVIEWER )
  foreach(FILE ${QGLVIEWER_SHAPES_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()
endif()
