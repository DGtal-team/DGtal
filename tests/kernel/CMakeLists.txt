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

if(BUILD_BENCHMARKS)
  set(DGTAL_BENCH_SRC
    benchmarkHyperRectDomain
    benchmarkHyperRectDomain-catch
  )

  if(WITH_BENCHMARK)
    set(DGTAL_BENCH_SRC
      ${DGTAL_BENCH_SRC}
      benchmarkSetContainer
      benchmarkHyperRectDomain-google
    )
  endif()

  #Benchmark target
  foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
    add_dependencies(benchmark ${FILE})
  endforeach()
endif()

#-----------------------
#GMP based tests
#----------------------
if(GMP_FOUND)
  set(DGTAL_TESTS_GMP_SRC testDGtalGMP)

  foreach(FILE ${DGTAL_TESTS_GMP_SRC})
    DGtal_add_test(${FILE})
  endforeach()
endif()
