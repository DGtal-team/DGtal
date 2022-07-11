set(DGTAL_TESTS_SRC_ARITH
       testModuloComputer
       testPattern 
              )

foreach(FILE ${DGTAL_TESTS_SRC_ARITH})
  DGtal_add_test(${FILE})
endforeach()

#-----------------------
#GMP based tests
#----------------------
set(DGTAL_TESTS_GMP_SRC 
    testIntegerComputer
    testLatticePolytope2D
    testSternBrocot 
    testLightSternBrocot
    testLighterSternBrocot
 )

if(GMP_FOUND)
  foreach(FILE ${DGTAL_TESTS_GMP_SRC})
    DGtal_add_test(${FILE})
  endforeach()
endif()

set(DGTAL_BENCH_GMP_SRC
   testStandardDSLQ0-reversedSmartDSS-benchmark
   testStandardDSLQ0-LSB-reversedSmartDSS-benchmark
   testStandardDSLQ0-LrSB-reversedSmartDSS-benchmark
   testStandardDSLQ0-smartDSS-benchmark
   testArithmeticDSS-benchmark
)

#Benchmark target
if(GMP_FOUND)
  foreach(FILE ${DGTAL_BENCH_GMP_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()
endif()
