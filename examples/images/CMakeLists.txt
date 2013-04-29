SET(DGTAL_EXAMPLES_SRC
  exampleConstImageAdapter
  extract2DImagesFrom3D
)


FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)
