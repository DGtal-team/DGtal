set(DGTAL_EXAMPLES_SRC
  toricdomainvolumetric
  distancetransform2D
  exampleFMM2D
  voronoimap2D
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

if (  DGTAL_WITH_POLYSCOPE_VIEWER )
   set(DGTAL_EXAMPLES_POLYSCOPE_SRC
  distancetransform3D
  exampleFMM3D
   )
  foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
     DGtal_add_example(${FILE})
  endforeach()

endif()
