set(DGTAL_TESTS_SRC_KERNEL
   testDigitalSet
   testHyperRectDomain
   testInteger
   testPointVector
   testLinearAlgebra
   testImagesSetsUtilities
   testBasicPointFunctors
   testEmbedder
   testPointPredicateConcepts
   testPointHashFunctions
   testLinearizer
   testPointFunctorHolder
   testNumberTraits
   testUnorderedSetByBlock
   testIntegerConverter
   testIntegralIntervals
   testLatticeSetByIntervals
   )


foreach(FILE ${DGTAL_TESTS_SRC_KERNEL})
  DGtal_add_test(${FILE})
endforeach()

set(DGTAL_BENCH_SRC
    benchmarkHyperRectDomain
    benchmarkHyperRectDomain-catch
    benchmarkSetContainer
    benchmarkHyperRectDomain-google
   )

#Benchmark target
foreach(FILE ${DGTAL_BENCH_SRC})
  DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
endforeach()

#-----------------------
#GMP based tests
#----------------------
if(GMP_FOUND)
  set(DGTAL_TESTS_GMP_SRC testDGtalBigInteger)

  foreach(FILE ${DGTAL_TESTS_GMP_SRC})
    DGtal_add_test(${FILE})
  endforeach()
endif()
