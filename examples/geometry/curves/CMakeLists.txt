add_subdirectory(estimation)

SET(DGTAL_EXAMPLES_SRC
  greedy-dss-decomposition
  ArithmeticalDSS
  convex-and-concave-parts
  exampleGridCurve2d
  exampleGeometricalDSS
  exampleGeometricalDCA
  exampleFrechetShortcut 
)


FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)



IF(MAGICK++_FOUND)
  SET(DGTALE_EXAMPLES_IO_Magick
    freemanChainDisplay
    )
  FOREACH(FILE ${DGTALE_EXAMPLES_IO_Magick})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO)
  ENDFOREACH(FILE)
ENDIF(MAGICK++_FOUND)


### Test with QGLViewer
if (  WITH_VISU3D_QGLVIEWER )
   SET(DGTAL_EXAMPLES_QGL_SRC
     exampleArithDSS3d
     exampleGridCurve3d
     exampleGridCurve3d-2
   )
  FOREACH(FILE ${DGTAL_EXAMPLES_QGL_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal)   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )

