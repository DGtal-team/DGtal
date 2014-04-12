SET(DGTAL_TESTS_SRC_IO_BOARDS
       testBoard3D)


FOREACH(FILE ${DGTAL_TESTS_SRC_IO_BOARDS})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
