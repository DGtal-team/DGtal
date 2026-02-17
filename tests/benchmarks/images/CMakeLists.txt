set(DGTAL_BENCH_SRC
    benchmarkImageContainer
)

foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_benchmark(${FILE})
endforeach()
