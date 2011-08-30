

SET(DGTAL_TESTS_SRC_IOVIEWERS
       testSimpleBoard
       testBoard2DCustomStyle
       testLongvol )


FOREACH(FILE ${DGTAL_TESTS_SRC_IOVIEWERS})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME} ${LIBBOARD_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


add_subdirectory(viewers)
add_subdirectory(colormaps)
add_subdirectory(readers)
add_subdirectory(writers)



