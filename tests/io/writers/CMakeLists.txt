set(DGTAL_TESTS_SRC_IO_WRITERS
       testCompressedVolWriter
       testPNMRawWriter
       testMeshWriter
       testGenericWriter
       testSTBWriter)

if(MAGICK++_FOUND)
  set(DGTAL_TESTS_SRC_IO_WRITERS
    ${DGTAL_TESTS_SRC_IO_WRITERS}
    testMagickWriter
    )
endif()


foreach(FILE ${DGTAL_TESTS_SRC_IO_WRITERS})
  DGtal_add_test(${FILE})
endforeach()
