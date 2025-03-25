set(DGTAL_TESTS_SRC_ARITH
       testModuloComputer
       testPattern 
       testIntegerComputer
       testLatticePolytope2D
       testSternBrocot 
       testLightSternBrocot
       testLighterSternBrocot
       testStandardDSLQ0-reversedSmartDSS-benchmark
       testStandardDSLQ0-LSB-reversedSmartDSS-benchmark
       testStandardDSLQ0-LrSB-reversedSmartDSS-benchmark
       testStandardDSLQ0-smartDSS-benchmark
       testArithmeticDSS-benchmark
              )

foreach(FILE ${DGTAL_TESTS_SRC_ARITH})
  DGtal_add_test(${FILE})
endforeach()