


SET(QGLVIEWER_EXAMPLES_SRC
   extract2DImagesFrom3DandVisu
  )

if (  WITH_VISU3D_QGLVIEWER )

  FOREACH(FILE ${QGLVIEWER_EXAMPLES_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal
                            ${DGtalLibDependencies})   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )




SET(DGTAL_EXAMPLES_SRC
  exampleConstImageAdapter
  exampleTiledImageFromImage
  extract2DImagesFrom3D
)


FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)
