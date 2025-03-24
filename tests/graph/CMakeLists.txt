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
