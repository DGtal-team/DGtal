SET(DGTAL_EXAMPLES_SRC
  exampleConstImageAdapter
  exampleTiledImage
  extract2DImagesFrom3D
)

if( WITH_HDF5 )
  SET(DGTAL_EXAMPLES_SRC ${DGTAL_EXAMPLES_SRC} 
  exampleImageFactoryFromHDF5)
endif( WITH_HDF5 )

FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)
