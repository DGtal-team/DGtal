SET(DGTAL_TESTS_SRC_MATH_LINALG
       testSimpleMatrix)


FOREACH(FILE ${DGTAL_TESTS_SRC_MATH_LINALG})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
