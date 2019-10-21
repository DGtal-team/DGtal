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
#  testImageContainerByHashTree
  testRigidTransformation2D
  testRigidTransformation3D
  testArrayImageAdapter
  testConstImageFunctorHolder
  )

if( WITH_HDF5 )
  SET(DGTAL_TESTS_SRC ${DGTAL_TESTS_SRC}
  testImageFactoryFromHDF5)
endif( WITH_HDF5 )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)

IF(BUILD_BENCHMARKS)
  SET(DGTAL_BENCH_SRC)

  IF(WITH_BENCHMARK)
    SET(DGTAL_BENCH_SRC
      ${DGTAL_BENCH_SRC}
      benchmarkImageContainer
    )
  ENDIF(WITH_BENCHMARK)

  #Benchmark target
  FOREACH(FILE ${DGTAL_BENCH_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal  ${DGtalLibDependencies})
    ADD_DEPENDENCIES(benchmark ${FILE})
  ENDFOREACH(FILE)
ENDIF(BUILD_BENCHMARKS)


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
  target_link_libraries (${FILE} DGtal ${DGtalLibDependencies})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
