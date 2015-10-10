
SET(DGTAL_EXAMPLES_SRC
	exampleEuclideanShapesDecorator
)


FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)

### with QGLViewer
if (  WITH_VISU3D_QGLVIEWER )
  SET(DGTAL_SHAPES_EXAMPLES_QGL_SRC
     mesh3DConstructionAndVisualisation
    )
  FOREACH(FILE ${DGTAL_SHAPES_EXAMPLES_QGL_SRC}) 
    add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal
      ${DGtalLibDependencies})   
  ENDFOREACH(FILE)
  
endif ( WITH_VISU3D_QGLVIEWER   )

