SET(DGTAL_TESTS_SRC
  testMeshImport
  testDGtalSet
  testDomain
  testKalimskySpace
  testManipulate
  testSelection
  )	


	FOREACH(FILE ${DGTAL_TESTS_SRC})
	add_executable(${FILE} ${FILE} )
	target_link_libraries (${FILE}  DGtalIO DGtal ${DGtalLibDependencies}  )
	add_test(${FILE} ${FILE})
	ENDFOREACH(FILE)

 
if(APPLE)
ADD_DEFINITIONS("-x objective-c++")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/Contents/MacOS)
endif(APPLE)


#configure_file(${PROJECT_SOURCE_DIR}/../Requirements.h.in 
#  ${PROJECT_SOURCE_DIR}/../Requirements.h)

#configure_file(${PROJECT_SOURCE_DIR}/../Requirements/plugins.cfg.in 
#  ${PROJECT_SOURCE_DIR}/../Requirements/plugins.cfg)


