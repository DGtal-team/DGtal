set(DGTAL_TESTS_SRC_HELPERS
  testParametricShape
  testImplicitShape
  testParameters
  testShortcuts
  )

foreach(FILE ${DGTAL_TESTS_SRC_HELPERS})
  DGtal_add_test(${FILE})
endforeach()
