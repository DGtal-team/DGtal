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


#--- Benchmark specific
if (WITH_BENCHMARK)
    set(DGTAL_TESTS_SRC_BENCHMARK ${DGTAL_TESTS_SRC_BENCHMARK} testBenchmark)
    FOREACH(FILE ${DGTAL_TESTS_SRC_BENCHMARK})
      add_executable(${FILE} ${FILE})
      target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
      add_test(${FILE} ${FILE})
    ENDFOREACH(FILE)

endif (WITH_BENCHMARK)
