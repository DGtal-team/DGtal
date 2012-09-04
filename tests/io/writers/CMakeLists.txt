SET(DGTAL_TESTS_SRC_IO_WRITERS
       testPNMRawWriter 
       testMeshWriter)


FOREACH(FILE ${DGTAL_TESTS_SRC_IO_WRITERS})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

