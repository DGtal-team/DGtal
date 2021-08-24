set(DGTAL_EXAMPLES_SRC
  exampleEuclideanShapesDecorator
  exampleMeshVoxelizer
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

### with QGLViewer
if (  WITH_VISU3D_QGLVIEWER )
  set(DGTAL_SHAPES_EXAMPLES_QGL_SRC
     mesh3DConstructionAndVisualisation
     viewMarchingCubes
     viewPolygonalMarchingCubes
     exampleSurfaceMesh
  )
  foreach(FILE ${DGTAL_SHAPES_EXAMPLES_QGL_SRC})
     DGtal_add_example(${FILE})
  endforeach()

endif()

if (WITH_EIGEN AND WITH_QGLVIEWER)

    set(DGTAL_EXAMPLES_SRC_DEC
        sphereCotangentLaplaceOperator
        )

    foreach(FILE ${DGTAL_EXAMPLES_SRC_DEC})
      DGtal_add_example(${FILE})
   endforeach()

endif()
