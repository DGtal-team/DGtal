set(DGTAL_TESTS_SRC_ARITH
       testModuloComputer
       testPattern 
       testIntegerComputer
       testLatticePolytope2D
       testSternBrocot 
       testLightSternBrocot
       testLighterSternBrocot
)

foreach(FILE ${DGTAL_TESTS_SRC_ARITH})
  DGtal_add_test(${FILE})
endforeach()
