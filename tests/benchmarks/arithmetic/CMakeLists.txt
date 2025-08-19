set(DGTAL_BENCH_SRC 
    testStandardDSLQ0-reversedSmartDSS-benchmark 
    testStandardDSLQ0-LSB-reversedSmartDSS-benchmark 
    testStandardDSLQ0-LrSB-reversedSmartDSS-benchmark
    testStandardDSLQ0-smartDSS-benchmark
    testArithmeticDSS-benchmark
)

if (GMP_FOUND)
    foreach(FILE ${DGTAL_BENCH_SRC})
        DGtal_add_benchmark(${FILE})
    endforeach()
endif()
