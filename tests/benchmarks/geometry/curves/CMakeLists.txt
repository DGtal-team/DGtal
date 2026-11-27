set(DGTAL_BENCH_SRC
    saturatedSegmentation-benchmark
    normalEstimators-benchmark
)

foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_benchmark(${FILE})
endforeach()
