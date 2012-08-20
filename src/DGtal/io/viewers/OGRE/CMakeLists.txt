
if(WITH_OGRE)

  configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Requirements.h.in 
    ${PROJECT_BINARY_DIR}/src/DGtal/io/viewers/OGRE/Requirements.h)

  add_subdirectory(Ressources)

endif(WITH_OGRE)
