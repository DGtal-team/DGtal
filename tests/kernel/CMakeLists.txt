set(DGTAL_TESTS_SRC_KERNEL
   testDigitalSet
   testHyperRectDomain
   testInteger
   testPointVector
   testLinearAlgebra
   testImagesSetsUtilities
   testBasicPointFunctors
   testEmbedder
   testPointPredicateConcepts
   testPointHashFunctions
   testLinearizer
   testPointFunctorHolder
   testNumberTraits
   testUnorderedSetByBlock
   testIntegerConverter
   testIntegralIntervals
   testLatticeSetByIntervals
   testDGtalBigInteger
   testDigitalSetByOctree
   )


foreach(FILE ${DGTAL_TESTS_SRC_KERNEL})
  DGtal_add_test(${FILE})
endforeach()

