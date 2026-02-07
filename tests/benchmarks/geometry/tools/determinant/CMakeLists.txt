set(DGTAL_BENCH_SRC
    testInHalfPlane-benchmark
)

foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_benchmark(${FILE})
endforeach()
