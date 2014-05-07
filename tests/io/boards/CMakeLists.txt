
SET(DGTAL_TESTS_SRC_BOARDS
  testBoard3d
  )

FOREACH(FILE ${DGTAL_TESTS_SRC_BOARDS})
 add_executable(${FILE} ${FILE})
 target_link_libraries (${FILE} DGtalIO DGtal ${DGtalLibDependencies})
ENDFOREACH(FILE)




