set(DGTAL_EXAMPLES_SRC
  toricdomainvolumetric
  distancetransform2D
  exampleFMM2D
  voronoimap2D
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

if (  WITH_VISU3D_QGLVIEWER )
   set(DGTAL_EXAMPLES_QGL_SRC
  distancetransform3D
  exampleFMM3D
   )
  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC}) 
     DGtal_add_example(${FILE})
  endforeach()

endif()
