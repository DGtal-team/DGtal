SET(DGTAL_TESTS_SRC
  testDistanceTransformation
  testDistanceTransformND
  testDT2Dperf
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
