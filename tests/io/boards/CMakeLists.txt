set(DGTAL_TESTS_SRC_IO_BOARDS
       testBoard3D
       testBallQuad)


foreach(FILE ${DGTAL_TESTS_SRC_IO_BOARDS})
  DGtal_add_test(${FILE})
endforeach()
