
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

if (WITH_VISU3D_QGLVIEWER)
  set(DGTAL_TUTO_EXAMPLES_QGL_SRC
    shapeDTViewer
    volDTGranulo
    polyhedralizer
  )
  foreach(FILE ${DGTAL_TUTO_EXAMPLES_QGL_SRC})
     DGtal_add_example(${FILE})
  endforeach()

endif()

