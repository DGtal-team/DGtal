set(DGTAL_TESTS_SRC
  test2x2DetComputers
  testInHalfPlane)

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()



#Benchmark target
if(BUILD_BENCHMARKS)
  set(DGTAL_BENCH_SRC
    testInHalfPlane-benchmark
    )
  foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
    add_custom_target(${FILE}-benchmark COMMAND ${FILE} ">benchmark-${FILE}.txt" )
    add_dependencies(benchmark ${FILE}-benchmark)
  endforeach()
endif()
