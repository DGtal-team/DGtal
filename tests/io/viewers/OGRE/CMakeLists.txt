


SET(DGTAL_TESTS_SRC
  testDGtalSet
  testDomain
  testKalimskySpace
  testManipulate
  testSelection
#  testMeshImport
  )	
        if(APPLE)
          ADD_DEFINITIONS("-x objective-c++")
          set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/Contents/MacOS)

          file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/Contents/MacOS)

	  FOREACH(FILE ${DGTAL_TESTS_SRC})

	    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE})
	    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE}/Contents)
	    file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE}/Contents/MacOS)

	    configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/io/viewers/OGRE/Ressources/info.plist.in 
	    ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE}/Contents/info.plist)
	    add_executable(${FILE} ${FILE} )
	    target_link_libraries (${FILE}  DGtalIO DGtal ${DGtalLibDependencies}  )
            ADD_CUSTOM_COMMAND(TARGET ${FILE}
	    		         POST_BUILD
	    		         COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/Contents/MacOS/${FILE} ${PROJECT_BINARY_DIR}/tests/io/viewers/OGRE/${FILE}/Contents/MacOS)
	   ENDFOREACH(FILE)
        else()
 	  FOREACH(FILE ${DGTAL_TESTS_SRC})
            add_executable(${FILE} ${FILE} )
	    target_link_libraries (${FILE}  DGtalIO DGtal ${DGtalLibDependencies}  )
	  ENDFOREACH(FILE)
	endif()
	 





