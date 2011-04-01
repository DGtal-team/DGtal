SET(DGTAL_TESTS_SRC
   dgtalboard-1-points
   dgtalboard-2-sets
   dgtalboard-3-custom-classes
   dgtalboard-3-custom-points
   dgtalboard-4-colormaps
   )

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)


if (  WITH_VISU3D_QGLVIEWER )
   SET(DGTAL_EXAMPLES_QGL_SRC
       digitalSetFromVol
       digitalSetFromPointList
   )
  FOREACH(FILE ${DGTAL_EXAMPLES_QGL_SRC}) 
   add_executable(${FILE} ${FILE})
    target_link_libraries ( ${FILE} ${LIBDGTALVISU3D_NAME} ${LIBDGTAL_NAME}
                            ${DGtalLibDependencies})   
  ENDFOREACH(FILE)

endif ( WITH_VISU3D_QGLVIEWER   )
