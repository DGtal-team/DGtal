SET(DGTAL_TESTS_SRC_KERNEL
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
   )


FOREACH(FILE ${DGTAL_TESTS_SRC_KERNEL})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal  ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

IF(BUILD_BENCHMARKS)
  SET(DGTAL_BENCH_SRC
    benchmarkHyperRectDomain
    benchmarkHyperRectDomain-catch
  )

  IF(WITH_BENCHMARK)
    SET(DGTAL_BENCH_SRC
      ${DGTAL_BENCH_SRC}
      benchmarkSetContainer
      benchmarkHyperRectDomain-google
    )
  ENDIF(WITH_BENCHMARK)

  #Benchmark target
  FOREACH(FILE ${DGTAL_BENCH_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal  ${DGtalLibDependencies})
    ADD_DEPENDENCIES(benchmark ${FILE})
  ENDFOREACH(FILE)
ENDIF(BUILD_BENCHMARKS)

#-----------------------
#GMP based tests
#----------------------
IF(GMP_FOUND)
  SET(DGTAL_TESTS_GMP_SRC testDGtalGMP)

  FOREACH(FILE ${DGTAL_TESTS_GMP_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal ${DGtalLibDependencies})
    add_test(${FILE} ${FILE})
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)
