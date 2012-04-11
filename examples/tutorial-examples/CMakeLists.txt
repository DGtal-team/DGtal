SET(DGTAL_TUTO_EXAMPLES_SRC
  pgm2freeman
  imageSetDT
  fileGridCurveRanges
  imageGridCurveEstimator
  shapeGridCurveEstimator
  freemanChainFromImage
)


FOREACH(FILE ${DGTAL_TUTO_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)




if (  WITH_VISU3D_QGLVIEWER )
   SET(DGTAL_TUTO_EXAMPLES_QGL_SRC
   shapeDTViewer
   3dVolViewer	
   )
  FOREACH(FILE ${DGTAL_TUTO_EXAMPLES_QGL_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtal DGtalIO
                            ${DGtalLibDependencies})   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )

