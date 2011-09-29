

SET(DGTAL_TESTS_SRC_IOVIEWERS
       testSimpleBoard
       testBoard2DCustomStyle
       testLongvol
       testArcDrawing )


FOREACH(FILE ${DGTAL_TESTS_SRC_IOVIEWERS})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


add_subdirectory(viewers)
add_subdirectory(colormaps)
add_subdirectory(readers)
add_subdirectory(writers)



