set(DGTAL_BENCH_SRC
    testChordNaivePlaneComputer-benchmark
    testCOBAGenericNaivePlaneComputer-benchmark
    testCOBANaivePlaneComputer-benchmark
)

foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_benchmark(${FILE})
endforeach()
