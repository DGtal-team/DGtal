add_subdirectory(determinant)

SET(DGTAL_TESTS_SRC
  testRayIntersection
  testPreimage
  testSphericalAccumulator)

SET(DGTAL_TESTS_QSRC
  testSphericalAccumulatorQGL)

SET(DGTAL_TESTS_TOOLS_EIGEN
  testGJK)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

IF (WITH_EIGEN)
 FOREACH(FILE ${DGTAL_TESTS_TOOLS_EIGEN})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
 ENDFOREACH(FILE)
ENDIF()


IF (WITH_VISU3D_QGLVIEWER)
  FOREACH(FILE ${DGTAL_TESTS_QSRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  ENDFOREACH(FILE)
ENDIF()
