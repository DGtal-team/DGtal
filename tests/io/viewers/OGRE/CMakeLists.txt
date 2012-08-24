
SET(DGTAL_TESTS_SRC
  testDGtalSet
  testDomain
  testKalimskySpace
  testManipulate
  testSelection
  testMeshImport
  )	
if(APPLE)
  
  FOREACH(FILE ${DGTAL_TESTS_SRC})
    several_prepare_executable_mac(${FILE} ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE)
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE}  DGtalIO DGtal ${DGtalLibDependencies}  )
    ADD_CUSTOM_COMMAND(TARGET ${FILE}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE} ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE}.app/Contents/MacOS)
      ADD_CUSTOM_COMMAND(TARGET ${FILE}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE} )
      
  ENDFOREACH(FILE)
else()
  FOREACH(FILE ${DGTAL_TESTS_SRC})
    add_executable(${FILE} ${FILE} )
    target_link_libraries (${FILE}  DGtalIO DGtal ${DGtalLibDependencies}  )
  ENDFOREACH(FILE)
endif()

