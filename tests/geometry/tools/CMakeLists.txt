SET(DGTAL_TESTS_SRC
  testPreimage
  testSphericalAccumulator)

SET(DGTAL_TESTS_QSRC
  testSphericalAccumulatorQGL)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


IF (WITH_VISU3D_QGLVIEWER)
  FOREACH(FILE ${DGTAL_TESTS_QSRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  ENDFOREACH(FILE)
ENDIF()
