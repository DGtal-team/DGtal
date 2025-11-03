add_subdirectory(distance)

set( DGTAL_EXAMPLES_SRC
        dvcm-2d
        fullConvexityCollapsiblePoints2D
        fullConvexitySphereGeodesics
        fullConvexityLUT2D
        exampleBoundedLatticePolytopeCount2D
        exampleBoundedLatticePolytopeCount3D
        exampleBoundedLatticePolytopeCount4D
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

if (  DGTAL_WITH_POLYSCOPE_VIEWER )
   set( DGTAL_EXAMPLES_POLYSCOPE_SRC
      fullConvexityAnalysis3D
      fullConvexityThinning3D
      fullConvexityShortestPaths3D
      digitalPolyhedronBuilder3D
      standardDigitalPolyhedronBuilder3D
      checkFullConvexityTheorems
   )
  foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
      DGtal_add_example(${FILE})
  endforeach()

endif()
