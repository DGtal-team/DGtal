SET(DGTAL_TESTS_SRC
  testDistanceTransformation
  testDistanceTransformationND
  testReverseDT
  testFMM
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
