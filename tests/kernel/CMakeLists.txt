SET(DGTAL_TESTS_SRC_KERNEL
   testDigitalSet
   testDomainSpanIterator
   testHyperRectDomain
   testHyperRectDomain-snippet
   testInteger
   testPointVector
   testPointVectorContainers
   testLinearAlgebra
   testImagesSetsUtilities
   testBasicPointFunctors
   testEmbedder
   testSimpleMatrix
   testPointPredicateConcepts
   )


FOREACH(FILE ${DGTAL_TESTS_SRC_KERNEL})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


#-----------------------
#GMP based tests
#----------------------
IF(GMP_FOUND)
  SET(DGTAL_TESTS_GMP_SRC testDGtalGMP)
  
  FOREACH(FILE ${DGTAL_TESTS_GMP_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
    add_test(${FILE} ${FILE})
  ENDFOREACH(FILE)
ENDIF(GMP_FOUND)
