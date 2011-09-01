### Tests without visualisation

SET(GEO3D_TESTS_SRC
#  testArithDSS3d
  )

FOREACH(FILE ${GEO3D_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


### Test with QGLViewer
SET(GEO3D_TESTS_WITH_QGLVIEWER_SRC
#  testArithDSS3dViewer
   )

if (  WITH_VISU3D_QGLVIEWER )

  FOREACH(FILE ${GEO3D_TESTS_WITH_QGLVIEWER_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} ${LIBDGTALVISU3D_NAME} ${LIBDGTAL_NAME})
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )

