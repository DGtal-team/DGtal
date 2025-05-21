
### Visu QGLViewer

set(VIEWER_BASED_TESTS_SRC
  testViewer3D
  testViewerQuad
  testBallQuadViewer
  testPolygon3D
  test3DImageView
  test3DImage3DView
  testDisplayKSCell
  )

if (  DGTAL_WITH_POLYSCOPE )
  foreach(FILE ${VIEWER_BASED_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()

endif()
