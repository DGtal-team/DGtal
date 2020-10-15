set(DGTAL_TESTS_SRC
  testDistanceTransformation
  testDistanceTransformationND
  testDistanceTransformationMetrics
  testReverseDT
  testFMM
  testVoronoiMap
  testMetrics
  testMetricBalls
  testPowerMap
  testReducedMedialAxis
  testSeparableMetricAdapter
  testChamferDT
  testChamferVoro
  testDigitalMetricAdapter
  testLpMetric
  )


foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()

set(DGTAL_BENCH_SRC
  testMetrics-benchmark
  )

if(BUILD_BENCHMARKS)
  #Benchmark target
  foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
    add_custom_target(${FILE}-benchmark COMMAND ${FILE} ">benchmark-${FILE}.txt" )
    add_dependencies(benchmark ${FILE}-benchmark)
  endforeach()
endif()
