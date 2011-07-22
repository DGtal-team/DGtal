# Make sure the compiler can find include files.
if ( CPP0X_INITIALIZER_LIST )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP0X_INITIALIZER_LIST )

include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${CMAKE_CURRENT_BINARY_DIR}/src)

##Recursive calls
## each subdir must contain a ModuleSRC.txt file with a set command on the variable ${DGTAL_SRC}
include(DGtal/utils/ModuleSRC.txt)
include(DGtal/base/ModuleSRC.txt)
include(DGtal/math/ModuleSRC.txt)
include(DGtal/kernel/ModuleSRC.txt)
include(DGtal/geometry/ModuleSRC.txt)
include(DGtal/topology/ModuleSRC.txt)
include(DGtal/io/ModuleSRC.txt)

include(DGtal/helpers/ModuleSRC.txt)

include(Board/ModuleSRC.txt)

# Create a library called "DGtal".
# The extension is already found. Any number of sources could be listed here.

add_library ( ${LIBDGTAL_NAME} SHARED ${DGTAL_SRC} ${BOARD_SRC})
target_link_libraries( ${LIBDGTAL_NAME} ${DGtalLibDependencies} )

install(TARGETS ${LIBDGTAL_NAME}
  # IMPORTANT: Add the DGtal library to the "export-set"
  EXPORT DGtalLibraryDepends
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  #PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/DGtal"
  COMPONENT dev)


if ( WITH_VISU3D )
  add_library ( ${LIBDGTALVISU3D_NAME} SHARED ${DGTALIV_SRC} ${DGTALQGLV_SRC} )
  target_link_libraries( ${LIBDGTALVISU3D_NAME} ${LIBDGTAL_NAME} ${DGtalLibDependencies} )
  
  install(TARGETS ${LIBDGTALVISU3D_NAME}
    # IMPORTANT: Add the DGtalVisu library to the "export-set"
    EXPORT DGtalLibraryDepends
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
   # PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/DGtal"
    COMPONENT dev)
  
endif( WITH_VISU3D )


