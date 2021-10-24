set(DGTAL_TESTS_MESHES_SRC
  testCorrectedNormalCurrentComputer
)

foreach(FILE ${DGTAL_TESTS_MESHES_SRC})
  DGtal_add_test(${FILE})
endforeach()

