set(QGLVIEWER_EXAMPLES_SRC
   extract2DImagesFrom3DandVisu
  )

if (  WITH_VISU3D_QGLVIEWER )
  foreach(FILE ${QGLVIEWER_EXAMPLES_SRC})
    DGtal_add_example(${FILE})
 endforeach()
endif()

set(DGTAL_EXAMPLES_SRC
  exampleConstImageAdapter
  exampleTiledImage
  extract2DImagesFrom3D
  extract2DSlicesImagesFrom3D
  imageBasicSubsampling
  exampleRigidtransformation2d
  exampleRigidtransformation3d
  exampleArrayImageAdapter
  exampleConstImageFunctorHolder
)

if(  DGTAL_WITH_HDF5 )
  set(DGTAL_EXAMPLES_SRC ${DGTAL_EXAMPLES_SRC}
  raw2HDF5
  exampleImageFactoryFromHDF5)
endif()

foreach(FILE ${DGTAL_EXAMPLES_SRC})
   DGtal_add_example(${FILE})
endforeach()
