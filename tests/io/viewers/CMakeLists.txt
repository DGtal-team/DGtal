
### Visu QGLViewer

set(QGLVIEWER_TESTS_SRC
  testViewer3D
  testViewerQuad
  testBallQuadViewer
  testPolygon3D
  test3DImageView
  test3DImage3DView
  testDisplayKSCell
  )

if (  WITH_VISU3D_QGLVIEWER )
  foreach(FILE ${QGLVIEWER_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()

endif()
