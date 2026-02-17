set(DGTAL_BENCH_SRC
    testBenchmark
    testLabelledMap-benchmark
    testMultiMap-benchmark
)

foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_benchmark(${FILE})
endforeach()
