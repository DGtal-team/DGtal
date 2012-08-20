
SET(DGTAL_TESTS_SRC
  testDomainViewer
  testSetViewer
  testPoint1Viewer
  testPoint2Viewer
  testSelectionViewer
  testManipulateViewer
  )
if(APPLE)
  
  FOREACH(FILE ${DGTAL_TESTS_SRC})
    several_prepare_executable_mac(${FILE} ${PROJECT_BINARY_DIR}/examples/io/viewers/OGRE)
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE}  DGtalIO DGtal ${DGtalLibDependencies}  )
    ADD_CUSTOM_COMMAND(TARGET ${FILE}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_BINARY_DIR}/examples/io/viewers/OGRE/${FILE} ${PROJECT_BINARY_DIR}/examples/io/viewers/OGRE/${FILE}.app/Contents/MacOS)
  ENDFOREACH(FILE)
else()
  FOREACH(FILE ${DGTAL_TESTS_SRC})
    add_executable(${FILE} ${FILE} )
    target_link_libraries (${FILE}  DGtalIO DGtal ${DGtalLibDependencies}  )
  ENDFOREACH(FILE)
endif()
