set(DGTAL_EXAMPLES_SRC
  exampleEuclideanShapesDecorator
  exampleMeshVoxelizer
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

### with Polyscope viewer
if (  DGTAL_WITH_POLYSCOPE_VIEWER )
  set(DGTAL_SHAPES_EXAMPLES_POLYSCOPE_SRC
     mesh3DConstructionAndVisualisation
     viewMarchingCubes
     viewPolygonalMarchingCubes
     exampleSurfaceMesh
  )
  foreach(FILE ${DGTAL_SHAPES_EXAMPLES_POLYSCOPE_SRC})
     DGtal_add_example(${FILE})
  endforeach()

endif()

if (WITH_EIGEN AND DGTAL_WITH_POLYSCOPE_VIEWER)

    set(DGTAL_EXAMPLES_SRC_DEC
        sphereCotangentLaplaceOperator
        )

    foreach(FILE ${DGTAL_EXAMPLES_SRC_DEC})
      DGtal_add_example(${FILE})
   endforeach()

endif()
