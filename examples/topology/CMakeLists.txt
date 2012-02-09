SET(DGTAL_EXAMPLES_SRC

)


if (  WITH_VISU3D_QGLVIEWER )
   SET(DGTAL_EXAMPLES_QGL_SRC
  homotopicThinning3D
  3dBorderExtraction
  3dBorderExtractionImg
  3dKSSurfaceExtraction
  volScanBoundary
  volTrackBoundary
  volBreadthFirstTraversal
   )
  FOREACH(FILE ${DGTAL_EXAMPLES_QGL_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal
                            ${DGtalLibDependencies})   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )

