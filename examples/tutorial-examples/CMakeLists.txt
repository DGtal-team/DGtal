SET(DGTAL_TUTO_EXAMPLES_SRC
  imageSetDT
  fileGridCurveRanges
  imageGridCurveEstimator
)


FOREACH(FILE ${DGTAL_TUTO_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME} )
ENDFOREACH(FILE)

#########

if (  WITH_VISU3D_QGLVIEWER )
   SET(DGTAL_TUTO_EXAMPLES_QGL_SRC
  
   )
  FOREACH(FILE ${DGTAL_TUTO_EXAMPLES_QGL_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} ${LIBDGTALVISU3D_NAME} ${LIBDGTAL_NAME}
                            ${DGtalLibDependencies})   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )

