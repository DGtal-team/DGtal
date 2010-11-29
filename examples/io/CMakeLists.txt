SET(DGTAL_TESTS_SRC
   dgtalboard-1-points
   dgtalboard-2-sets
   dgtalboard-3-custom-classes
   dgtalboard-3-custom-points
   dgtalboard-4-colormaps
   dgtalboard-5-greedy-dss
   )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)

