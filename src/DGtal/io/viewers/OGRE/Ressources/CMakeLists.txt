if(WITH_OGRE)


configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/plugins.cfg.in 
  ${PROJECT_BINARY_DIR}/src/DGtal/io/viewers/OGRE/Ressources/plugins.cfg)


endif(WITH_OGRE)
