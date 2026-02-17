set(DGTAL_BENCH_SRC
   testObject-benchmark
   testImplicitDigitalSurface-benchmark
   testLightImplicitDigitalSurface-benchmark
)

foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_benchmark(${FILE})
endforeach()
