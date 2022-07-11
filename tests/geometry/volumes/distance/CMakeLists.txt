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
  testVoronoiMapComplete
  )


foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()

set(DGTAL_BENCH_SRC
  testMetrics-benchmark
  )

#Benchmark target
foreach(FILE ${DGTAL_BENCH_SRC})
  DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
endforeach()
