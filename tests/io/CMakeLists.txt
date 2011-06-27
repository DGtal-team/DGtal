SET(DGTAL_TESTS_SRC_IOVIEWERS
       testSimpleBoard
       testDGtalBoardCustomStyle
       testLongvol )


FOREACH(FILE ${DGTAL_TESTS_SRC_IOVIEWERS})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME} ${LIBBOARD_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)




