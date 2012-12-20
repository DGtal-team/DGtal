SET(DGTAL_TESTS_SRC_IO_READERS
       testPNMReader
       testVolReader
       testRawReader     
       testPointListReader 
       testMeshReader
       testMPolynomialReader )


FOREACH(FILE ${DGTAL_TESTS_SRC_IO_READERS})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


IF(MAGICK++_FOUND)

  SET(DGTAL_TESTS_SRC_IO_READERS_Magick
    testMagickReader
    )

  FOREACH(FILE ${DGTAL_TESTS_SRC_IO_READERS_Magick})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
    add_test(${FILE} ${FILE})
  ENDFOREACH(FILE)

ENDIF(MAGICK++_FOUND)

