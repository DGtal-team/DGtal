SET(DGTAL_EXAMPLES_SRC
  distancetransform2D
  exampleFMM2D
  voronoimap2D
)


FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)

if (  WITH_VISU3D_QGLVIEWER )
   SET(DGTAL_EXAMPLES_QGL_SRC
  distancetransform3D
  exampleFMM3D
   )
  FOREACH(FILE ${DGTAL_EXAMPLES_QGL_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal)   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )
