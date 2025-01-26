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

if(  DGTAL_WITH_HDF5 )
  set(DGTAL_TESTS_SRC ${DGTAL_TESTS_SRC} testImageFactoryFromHDF5)
endif()

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()
set(DGTAL_BENCH_SRC
    benchmarkImageContainer)

#Benchmark target
foreach(FILE ${DGTAL_BENCH_SRC})
  DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
endforeach()

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
