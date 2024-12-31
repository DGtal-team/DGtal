set(DGTAL_TESTS_SRC_IOVIEWERS
  testColor
  testSimpleBoard
  testBoard2DCustomStyle
  testLongvol
  testArcDrawing )

if  (DGTAL_WITH_ITK)
    set(DGTAL_TESTS_SRC_IOVIEWERS ${DGTAL_TESTS_SRC_IOVIEWERS} testITKio)
endif()

foreach(FILE ${DGTAL_TESTS_SRC_IOVIEWERS})
  DGtal_add_test(${FILE})
endforeach()


add_subdirectory(viewers)
add_subdirectory(boards)
add_subdirectory(colormaps)
add_subdirectory(readers)
add_subdirectory(writers)
