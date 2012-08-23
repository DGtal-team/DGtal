


# This file contains macros that are useful for app bundle creating


macro(several_prepare_executable_mac FILE_NAME FILE_PATH)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app/Contents)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app/Contents/MacOS)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app/Contents/Frameworks)
  execute_process( COMMAND ${CMAKE_COMMAND}  -E create_symlink  ${OGRE_HOME}lib/release/Ogre.framework/ ${FILE_PATH}/${FILE_NAME}.app/Contents/Frameworks/Ogre.framework )
  configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/info.plist.in 
    ${FILE_PATH}/${FILE_NAME}.app/Contents/info.plist)
endmacro(several_prepare_executable_mac)


macro(prepare_executable_mac FILE_NAME FILE_PATH)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app/Contents)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app/Contents/MacOS)
  file(MAKE_DIRECTORY ${FILE_PATH}/${FILE_NAME}.app/Contents/Frameworks)
  execute_process( COMMAND ${CMAKE_COMMAND}  -E create_symlink  ${OGRE_HOME}lib/release/Ogre.framework/ ${FILE_PATH}/${FILE_NAME}.app/Contents/Frameworks/Ogre.framework )
  configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/info.plist.in 
    ${FILE_PATH}/${FILE_NAME}.app/Contents/info.plist)
endmacro(prepare_executable_mac)

macro(prepare_executable FILE_NAME PATH)
  if(APPLE)
   set( FILE ${FILE_NAME})
    file(MAKE_DIRECTORY ${PATH}/${FILE_NAME}.app)
    file(MAKE_DIRECTORY ${PATH}/${FILE_NAME}.app/Contents)
    file(MAKE_DIRECTORY ${PATH}/${FILE_NAME}.app/Contents/MacOS)
    file(MAKE_DIRECTORY ${PATH}/${FILE_NAME}.app/Contents/Frameworks)
    message(${OGRE_HOME}lib/release/Ogre.framework/)
    execute_process( COMMAND ${CMAKE_COMMAND}  -E create_symlink  ${OGRE_HOME}lib/release/Ogre.framework/ ${PATH}/${FILE_NAME}.app/Contents/Frameworks/Ogre.framework )
    configure_file(${DGtal_DIR}/src/DGtal/io/viewers/OGRE/Ressources/info.plist.in
    ${PATH}/${FILE_NAME}.app/Contents/info.plist)

  endif(APPLE)
endmacro(prepare_executable)