set(DGTAL_TESTS_SRC
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
  set(DGTAL_TESTS_SRC ${DGTAL_TESTS_SRC} testImageFactoryFromHDF5)
endif()

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()

if(BUILD_BENCHMARKS)
  set(DGTAL_BENCH_SRC)

  if(WITH_BENCHMARK)
    set(DGTAL_BENCH_SRC
      ${DGTAL_BENCH_SRC}
      benchmarkImageContainer
    )
  endif()

  #Benchmark target
  foreach(FILE ${DGTAL_BENCH_SRC})
    DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
    add_dependencies(benchmark ${FILE})
  endforeach()
endif()


#-----------------------
#ITK based tests
#----------------------
if(ITK_FOUND)
  set(DGTAL_TESTS_ITK_SRC testITKImage)
else()
  set(DGTAL_TESTS_ITK_SRC "")
endif()

foreach(FILE ${DGTAL_TESTS_ITK_SRC})
  DGtal_add_test(${FILE})
endforeach()
