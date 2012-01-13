SET(DGTAL_TESTS_SRC
   testBasicBoolFunctions
   testOrderedAlphabet
   testCirculator
   testConstIteratorAdapter
   testModifier
   testProgressBar
   )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

