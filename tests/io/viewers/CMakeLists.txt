
### Visu QGLViewer

set(QGLVIEWER_TESTS_SRC
  testViewer3D
  testViewerQuad
  testBallQuadViewer
  testPolygon3D
  test3DImageView
  test3DImageViewEmbedder
  test3DImage3DView
  testDisplayKSCell
  test3DDomain2DView
  )

if (  WITH_VISU3D_QGLVIEWER )
  foreach(FILE ${QGLVIEWER_TESTS_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()

endif()

if ( DGTAL_WITH_POLYSCOPE_VIEWER)
  DGtal_add_test(testPolyscopeViewer)
endif()