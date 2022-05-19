add_subdirectory(distance)

set( DGTAL_EXAMPLES_SRC
        dvcm-2d
        fullConvexityCollapsiblePoints2D
        fullConvexitySphereGeodesics
        fullConvexityLUT2D
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

if (  WITH_VISU3D_QGLVIEWER )
   set( DGTAL_EXAMPLES_QGL_SRC
      fullConvexityAnalysis3D
      fullConvexityThinning3D
      fullConvexityShortestPaths3D
   )
  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC})
      DGtal_add_example(${FILE})
  endforeach()

endif()
