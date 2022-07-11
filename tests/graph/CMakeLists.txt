set(DGTAL_TESTS_SRC
   testBreadthFirstPropagation
   testDepthFirstPropagation
   # testDigitalSurfaceBoostGraphInterface
   testObjectBoostGraphInterface
   testDistancePropagation
   testExpander
   testSTLMapToVertexMapAdapter
   )

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()


set(DGTAL_BENCH_SRC
   testExpander-benchmark
)


#Benchmark target
foreach(FILE ${DGTAL_BENCH_SRC})
  DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
endforeach()
