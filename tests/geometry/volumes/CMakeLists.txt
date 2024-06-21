add_subdirectory(estimation)
add_subdirectory(distance)


set(DGTAL_TESTS_VOLUMES_SRC
  testKanungo
  testBoundedLatticePolytope
  testBoundedLatticePolytopeCounter
  testBoundedRationalPolytope
  testCellGeometry
  testDigitalConvexity
  testPConvexity
  testConvexityHelper
  testFullConvexity
  testEhrhartPolynomial
  testShortestPaths
)

foreach(FILE ${DGTAL_TESTS_VOLUMES_SRC})
  DGtal_add_test(${FILE})
endforeach()

