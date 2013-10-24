
### Visu QGLViewer

SET(QGLVIEWER_TESTS_SRC
  testViewer3D
  testPolygon3D
  test3DImageView
  test3DImage3DView
  testDisplayKSCell
  test3DDomain2DView
  )

if (  WITH_VISU3D_QGLVIEWER )

  FOREACH(FILE ${QGLVIEWER_TESTS_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal
                            ${DGtalLibDependencies})   
    #add_test(${FILE} ${FILE})
    #not a test since IHM
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )



### Visu INVENTOR


SET(DGTALIV_TESTS_SRC
   testIVViewer
   volIVViewer
   )

if ( WITH_VISU3D_IV )

  FOREACH(FILE ${DGTALIV_TESTS_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} DGtalIO DGtal
                            ${DGtalLibDependencies})
    #add_test(${FILE} ${FILE})
    #not a test since IHM
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_IV )
