#------------------------------------------------------------------------------
# DGtal Configuration file for the install target
#------------------------------------------------------------------------------

#--- Headers installation
install(DIRECTORY "${PROJECT_SOURCE_DIR}/src/" 
  DESTINATION "${INSTALL_INCLUDE_DIR}/"
  FILES_MATCHING PATTERN "*.*h")

#-- specific install for Config.h.in
install(DIRECTORY "${PROJECT_BINARY_DIR}/src/DGtal/" 
  DESTINATION "${INSTALL_INCLUDE_DIR}/DGtal/" 
  FILES_MATCHING PATTERN "*.*h")
INCLUDE(${CMAKE_MODULE_PATH}/DGtalConfigGenerator.cmake)



if(WITH_VISU3D_OGRE)

  #--- Ogre viewer ressources installation
  FILE(GLOB_RECURSE Ressourcefiles "${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/media/*.*")
  install(FILES ${Ressourcefiles}
    DESTINATION "${INSTALL_DATA_DIR}/DGtal/OGRE/Ressources/media/")
  
  install(FILES "${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/ressources.cfg"
    DESTINATION "${INSTALL_DATA_DIR}/DGtal/OGRE/Ressources/")
  install(FILES "${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/ogre.cfg"
    DESTINATION "${INSTALL_DATA_DIR}/DGtal/OGRE/Ressources/")

  install(FILES
    "${PROJECT_SOURCE_DIR}/cmake/FindOgreDGtal.cmake"
    DESTINATION "${DGTAL_CMAKE_DIR}" COMPONENT dev)

  install (CODE "set(INSTALL_DATA_DIR ${INSTALL_DATA_DIR})
  configure_file(  ${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/RequirementsInstall.h.in
    ${INSTALL_INCLUDE_DIR}/DGtal/io/viewers/OGRE/Requirements.h )")
  
  install(CODE  "set(OGRE_PLUGINS ${OGRE_PLUGINS})
configure_file(  ${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/plugins.cfg.in
    ${INSTALL_DATA_DIR}/DGtal/OGRE/Ressources/plugins.cfg )")

  IF (APPLE)
    install(CODE
      "configure_file(${PROJECT_SOURCE_DIR}/cmake/CheckOBJCInstall.cmake 
    ${DGTAL_CMAKE_DIR}/CheckOBJC.cmake)")
    
    install(CODE  "configure_file(  ${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/info.plist.in
    ${INSTALL_DATA_DIR}/DGtal/OGRE/Ressources/info.plist.in )")
  ENDIF(APPLE)
endif(WITH_VISU3D_OGRE)


