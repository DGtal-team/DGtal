SET(DGTAL_TOOLS_SRC
  shapeGenerator
  contourGenerator
)

FOREACH(FILE ${DGTAL_TOOLS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO  ${DGtalLibDependencies} )
  install(TARGETS ${FILE} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
ENDFOREACH(FILE)

