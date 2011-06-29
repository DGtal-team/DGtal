SET(DGTAL_TESTS_SRC
  dgtalCairo-0-demo-kernel-2
  dgtalCairo-1-points
  dgtalCairo-2-sets
  dgtalCairo-2bis-sets
  dgtalCairo-3-objects
  dgtalCairo-4-modes
  dgtalCairo-5-custom
  dgtalCairo-6
)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)

