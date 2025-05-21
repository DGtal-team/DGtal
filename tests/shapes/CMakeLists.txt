
### Visu QGLViewer

set(VIEWER_BASED_TESTS_SRC
 testMeshDisplay
   )

if (  DGTAL_WITH_POLYSCOPE )
  foreach(FILE ${VIEWER_BASED_TESTS_SRC})
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

if (DGTAL_WITH_LIBIGL)
   set(DGTAL_TESTS_SRC_LIBIGL 
       testlibigl
       testWindingNumbersShape
      )
    foreach(test_file ${DGTAL_TESTS_SRC_LIBIGL})
      DGtal_add_test(${test_file})
    endforeach()      
endif()

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()


##### Shapes with viewer.

set(QGLVIEWER_SHAPES_TESTS_SRC
#  testPolynomial
  testBall3D
  )

if ( DGTAL_WITH_POLYSCOPE )
  foreach(FILE ${QGLVIEWER_SHAPES_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()
endif()
