SET(DGTAL_TESTS_SRC
  testImageSimple
  testImageAdapter
  testImageCache
  testTiledImage
  testConstImageAdapter
  testImage
  testImageSpanIterators
  testCheckImageConcept
  testMorton
  testHashTree
  testSliceImageFromFunctor
  )

if( WITH_HDF5 )
  SET(DGTAL_TESTS_SRC ${DGTAL_TESTS_SRC} 
  testImageFactoryFromHDF5)
endif( WITH_HDF5 )

SET(DGTAL_BENCH_SRC
  testImageContainerBenchmark
  testImageContainerByHashTree
  )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

#Benchmark target
FOREACH(FILE ${DGTAL_BENCH_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO)
  add_custom_target(${FILE}-benchmark COMMAND ${FILE} ">benchmark-${FILE}.txt" )
  ADD_DEPENDENCIES(benchmark ${FILE}-benchmark)
ENDFOREACH(FILE)



#-----------------------
#ITK based tests
#----------------------
IF(ITK_FOUND)
  SET(DGTAL_TESTS_ITK_SRC testITKImage)
ELSE(ITK_FOUND)
  SET(DGTAL_TESTS_ITK_SRC "")
ENDIF(ITK_FOUND)

FOREACH(FILE ${DGTAL_TESTS_ITK_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
