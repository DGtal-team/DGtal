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
