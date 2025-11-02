
set(DGTAL_TUTO_EXAMPLES_SRC
  imageSetDT
  fileGridCurveRanges
  imageGridCurveEstimator
  shapeGridCurveEstimator
  freemanChainFromImage
  FMMErosion
  2DSliceImageFromVol
  shortcuts
  shortcuts-geometry
)


foreach(FILE ${DGTAL_TUTO_EXAMPLES_SRC})
    DGtal_add_example(${FILE})
   if(MSVC)
     set_source_files_properties( ${FILE} PROPERTIES COMPILE_FLAGS "/bigobj" )
   endif()
endforeach()

if (DGTAL_WITH_POLYSCOPE_VIEWER)
  set(DGTAL_TUTO_EXAMPLES_POLYSCOPE_SRC
    shapeDTViewer
    volDTGranulo
    polyhedralizer
  )
  foreach(FILE ${DGTAL_TUTO_EXAMPLES_POLYSCOPE_SRC})
     DGtal_add_example(${FILE})
  endforeach()

endif()

