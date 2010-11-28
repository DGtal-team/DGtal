SET(DGTAL_TESTS_SRC
  distancetransform2D
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)

