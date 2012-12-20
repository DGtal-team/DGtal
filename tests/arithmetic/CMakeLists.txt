SET(DGTAL_TESTS_SRC_ARITH
       testModuloComputer
       testPattern )

FOREACH(FILE ${DGTAL_TESTS_SRC_ARITH})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal)
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

#-----------------------
#GMP based tests
#----------------------
SET(DGTAL_TESTS_GMP_SRC 
    testIntegerComputer
    testLatticePolytope2D
    testSternBrocot 
    testLightSternBrocot
    testLighterSternBrocot
 )

IF(GMP_FOUND)
  FOREACH(FILE ${DGTAL_TESTS_GMP_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
    add_test(${FILE} ${FILE})
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)

SET(DGTAL_BENCH_GMP_SRC
   testStandardDSLQ0-reversedSmartDSS-benchmark
   testStandardDSLQ0-LSB-reversedSmartDSS-benchmark
   testStandardDSLQ0-LrSB-reversedSmartDSS-benchmark
   testStandardDSLQ0-smartDSS-benchmark
   testArithmeticDSS-benchmark
)


#Benchmark target
IF(GMP_FOUND)
  FOREACH(FILE ${DGTAL_BENCH_GMP_SRC})
    add_executable(${FILE} ${FILE}) 
    target_link_libraries (${FILE} DGtal DGtalIO)
    add_custom_target(${FILE}-benchmark COMMAND ${FILE} ">benchmark-${FILE}.txt" )
    ADD_DEPENDENCIES(benchmark ${FILE}-benchmark)
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)
