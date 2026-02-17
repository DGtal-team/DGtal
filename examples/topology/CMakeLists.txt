set(DGTAL_EXAMPLES_TOPO_SRC
  ctopo-1
  ctopo-2
  ctopo-fillContours
  khalimskySpaceScanner
  volMarchingCubes
  volToOFF
  trackImplicitPolynomialSurfaceToOFF
  generateSimplicityTables2D
  generateSimplicityTables3D
  generateVoxelComplexTables
  digitalSetToCubicalComplexes2D
  cubical-complex-illustrations
  cubicalComplexThinning
  area-estimation-with-digital-surface
  area-estimation-with-indexed-digital-surface
)

foreach(FILE ${DGTAL_EXAMPLES_TOPO_SRC})
       DGtal_add_example(${FILE})
endforeach()


if (  DGTAL_WITH_POLYSCOPE_VIEWER )
   set(DGTAL_EXAMPLES_POLYSCOPE_SRC
     cubical-complex-collapse
     homotopicThinning3D
     3dBorderExtraction
     3dBorderExtractionImg
     ctopo-1-3d
     ctopo-1s-3d
     ctopo-2-3d
     3dKSSurfaceExtraction
     volScanBoundary
     volTrackBoundary
     volBreadthFirstTraversal
     frontierAndBoundary
     digitalSurfaceSlice
   )
  foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
     DGtal_add_example(${FILE})
  endforeach()

endif()
