set(DGTAL_EXAMPLES_SRC
  exampleEuclideanShapesDecorator
  exampleMeshVoxelizer
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal  )
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
    add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE}  DGtal
      ${DGtalLibDependencies})   
  endforeach()
  
endif()

if (WITH_EIGEN AND WITH_QGLVIEWER)

    set(DGTAL_EXAMPLES_SRC_DEC
        sphereCotangentLaplaceOperator
        )

    foreach(FILE ${DGTAL_EXAMPLES_SRC_DEC})
        add_executable(${FILE} ${FILE})
        target_link_libraries (${FILE} DGtal )
    endforeach()

endif()
