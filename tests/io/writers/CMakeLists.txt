set(DGTAL_TESTS_SRC_IO_WRITERS
       testCompressedVolWriter
       testPNMRawWriter
       testMeshWriter
       testGenericWriter
       testSTBWriter)

foreach(FILE ${DGTAL_TESTS_SRC_IO_WRITERS})
  DGtal_add_test(${FILE})
endforeach()
