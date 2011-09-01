SET(DGTAL_TESTS_SRC
   testAdjacency
   testCellularGridSpaceND
   testDigitalSurface
   testDigitalTopology
   testExpander
   testObject
   testObjectBorder
   testSimpleExpander
   )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


SET(DGTAL_BENCH_SRC
   testExpander-benchmark
   testObject-benchmark
)


#Benchmark target
FOREACH(FILE ${DGTAL_BENCH_SRC})
add_executable(${FILE} ${FILE})
target_link_libraries (${FILE} ${LIBDGTAL_NAME})
add_custom_target(${FILE}-benchmark COMMAND ${FILE} ">benchmark-${FILE}.txt" )
ADD_DEPENDENCIES(benchmark ${FILE}-benchmark)
ENDFOREACH(FILE)
