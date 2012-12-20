
if(WITH_OGRE)

  configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Requirements.h.in 
    ${PROJECT_BINARY_DIR}/src/DGtal/io/viewers/OGRE/Requirements.h)

  add_subdirectory(Ressources)
if(APPLE)
  configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/Info.plist.in 
    ${PROJECT_BINARY_DIR}/src/DGtal/io/viewers/OGRE/Ressources/Info.plist.in
    COPYONLY)
endif(APPLE)

endif(WITH_OGRE)
