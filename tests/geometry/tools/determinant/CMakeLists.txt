set(DGTAL_TESTS_SRC
  test2x2DetComputers
  testInHalfPlane)

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()

