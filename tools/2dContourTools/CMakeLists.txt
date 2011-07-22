




SET(DGTAL_TOOLS_SRC
  pgm2freeman
  displayContours
  curvatureMCMS
  curvatureBC
  tangentBC
)

FOREACH(FILE ${DGTAL_TOOLS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME}  ${DGtalLibDependencies} )
  install(TARGETS ${FILE} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
ENDFOREACH(FILE)



