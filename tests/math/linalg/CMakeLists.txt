SET(DGTAL_TESTS_SRC_MATH_LINALG
       testSimpleMatrix)

if (WITH_EIGEN)
    set(DGTAL_TESTS_SRC_MATH_LINALG "${DGTAL_TESTS_SRC_MATH_LINALG}"
    "testEigenSolver")
endif (WITH_EIGEN)


FOREACH(FILE ${DGTAL_TESTS_SRC_MATH_LINALG})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
