#CMakeLists associated to the tests subdir
#David Coeurjolly <david.coeurjolly@liris.cnrs.fr>

# Make sure the compiler can find include files.
if ( CPP0X_INITIALIZER_LIST )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP0X_INITIALIZER_LIST )

# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/src/)
include_directories (${PROJECT_BINARY_DIR}/src/)
# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/tools/)
include_directories (${PROJECT_BINARY_DIR}/tools/)


# Make sure the linker can find the library.
link_directories (${PROJECT_BINARY_DIR})


#------Tools subdirectories ------

add_subdirectory(volumetric)
add_subdirectory(shapeGenerator)
add_subdirectory(estimators)
add_subdirectory(2dContourTools)
add_subdirectory(imageTools)


#------ Basic tools ------

##-- Cmake configuration for dgtal-config tool
CONFIGURE_FILE( ${PROJECT_SOURCE_DIR}/tools/dgtal-config.cpp.in
  ${PROJECT_BINARY_DIR}/tools/dgtal-config.cpp)

add_executable(dgtal-config ${PROJECT_BINARY_DIR}/tools/dgtal-config.cpp)
target_link_libraries (dgtal-config ${LIBDGTAL_NAME}  ${DGtalLibDependencies} )
install(TARGETS dgtal-config 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)


