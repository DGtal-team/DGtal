SET(DGTAL_TESTS_SRC
   testBasicBoolFunctions
   testOrderedAlphabet
   testCirculator
   testConstIteratorAdapter
   testProgressBar
   testBasicFunctors
   testConstRangeAdapter
   testOutputIteratorAdapter
   testClock
   testTrace
   testcpp11
   testCountedPtr
   testBits
   testIndexedListWithBlocks
   testLabels
   testLabelledMap
   testLabelledMap-benchmark
   testMultiMap-benchmark
   testOpenMP
   testIteratorFunctions
   testIteratorCirculatorTraits
   testCloneAndAliases
   testClone2
   testOwningOrAliasingPtr
   )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


