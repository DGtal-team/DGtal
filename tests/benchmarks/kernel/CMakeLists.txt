set(DGTAL_BENCH_SRC 
    benchmarkHyperRectDomain
    benchmarkHyperRectDomain-catch
    benchmarkHyperRectDomain-google
    benchmarkSetContainer
)

foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_benchmark(${FILE})
endforeach()