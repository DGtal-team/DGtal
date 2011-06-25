SET(DGTAL_TESTS_SRC
  testDistanceTransformation
  testDistanceTransformationND
  testReverseDT
  testMeasureSet
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
