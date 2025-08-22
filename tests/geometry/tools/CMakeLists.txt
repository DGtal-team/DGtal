add_subdirectory(determinant)

set(DGTAL_TESTS_SRC
  testRayIntersection
  testPreimage
  testSphericalAccumulator
  testHullFunctions2D
  testPolarPointComparatorBy2x2DetComputer
  testConvexHull2D
  testConvexHull2DThickness
  testConvexHull2DReverse
  testQuickHull
  testAffineSubset)

set(DGTAL_TESTS_VSRC
  testSphericalAccumulatorViewer)

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()


if (DGTAL_WITH_POLYSCOPE)
  foreach(FILE ${DGTAL_TESTS_VSRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()
endif()
