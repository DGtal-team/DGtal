SET(DGTAL_TESTS_SRC_KERNEL
   testDigitalSet
   testDomainSpanIterator
   testHyperRectDomain
   testHyperRectDomain-snippet
   testInteger
   testPointVector
   testRealPointVector
   testSpaceND
   testMorton
   testHashTree
   testLinearAlgebra
   testImagesSetsUtilities
   )


FOREACH(FILE ${DGTAL_TESTS_SRC_KERNEL})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME} ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


#-----------------------
#GMP based tests
#----------------------
IF(GMP_FOUND)
  SET(DGTAL_TESTS_GMP_SRC testDGtalGMP)
ELSE(GMP_FOUND)
  SET(DGTAL_TESTS_GMP_SRC "")
ENDIF(GMP_FOUND)

FOREACH(FILE ${DGTAL_TESTS_GMP_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME} ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
