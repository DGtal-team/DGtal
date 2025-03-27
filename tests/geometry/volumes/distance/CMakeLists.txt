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
