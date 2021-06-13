add_subdirectory(estimation)
add_subdirectory(distance)


set(DGTAL_TESTS_VOLUMES_SRC
  testKanungo
  testBoundedLatticePolytope
  testBoundedRationalPolytope
  testCellGeometry
  testDigitalConvexity
  testConvexityHelper
  testFullConvexity
)

foreach(FILE ${DGTAL_TESTS_VOLUMES_SRC})
  DGtal_add_test(${FILE})
endforeach()

