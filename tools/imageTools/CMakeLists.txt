

SET(DGTAL_TOOLS_SRC_image
  CComponentCounter3D
 )

FOREACH(FILE ${DGTAL_TOOLS_SRC_image})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME}  ${DGtalLibDependencies} )
  install(TARGETS ${FILE} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
ENDFOREACH(FILE)



