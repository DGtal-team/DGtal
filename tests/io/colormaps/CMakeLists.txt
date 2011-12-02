SET(DGTAL_TESTS_SRC_COLORMAP
       testColorMaps       )


FOREACH(FILE ${DGTAL_TESTS_SRC_COLORMAP})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

