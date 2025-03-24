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
