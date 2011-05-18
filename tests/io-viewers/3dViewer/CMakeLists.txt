SET(QGLVIEWER_TESTS_SRC
	test3dViewer
   )

if (  WITH_VISU3D_QGLVIEWER )

  FOREACH(FILE ${QGLVIEWER_TESTS_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} ${LIBDGTALVISU3D_NAME} ${LIBDGTAL_NAME}
                            ${DGtalLibDependencies})   
    #add_test(${FILE} ${FILE})
    #not a test since IHM
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )
