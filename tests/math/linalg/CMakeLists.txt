set(DGTAL_TESTS_SRC_MATH_LINALG
       testSimpleMatrix
       testEigenDecomposition )

if (WITH_EIGEN)
    set(DGTAL_TESTS_SRC_MATH_LINALG "${DGTAL_TESTS_SRC_MATH_LINALG}"
    "testEigenSolver")
endif()


foreach(FILE ${DGTAL_TESTS_SRC_MATH_LINALG})
  DGtal_add_test(${FILE})
endforeach()
