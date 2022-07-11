set(DGTAL_TESTS_SRC
   testBasicBoolFunctors
   testOrderedAlphabet
   testCirculator
   testConstIteratorAdapter
   testProgressBar
   testBasicFunctors
   testConstRangeAdapter
   testOutputIteratorAdapter
   testClock
   testTrace
   testCountedPtr
   testCountedPtrOrPtr
   testCountedConstPtrOrConstPtr
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
   testPartialTemplateSpecialization
   testContainerTraits
   testSetFunctions
   testSimpleRandomAccessRangeFromPoint
   testFunctorHolder)

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()


#--- Benchmark specific
set(DGTAL_TESTS_SRC_BENCHMARK
    testBenchmark)
 
foreach(FILE ${DGTAL_TESTS_SRC_BENCHMARK})
  DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
endforeach()

