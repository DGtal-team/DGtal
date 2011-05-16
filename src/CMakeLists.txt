# Make sure the compiler can find include files.
if ( CPP0X_INITIALIZER_LIST )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP0X_INITIALIZER_LIST )

include_directories (${PROJECT_SOURCE_DIR}/src)

##Recursive calls
## each subdir must contain a ModuleSRC.txt file with a set command on the variable ${DGTAL_SRC}
include(DGtal/utils/ModuleSRC.txt)
include(DGtal/base/ModuleSRC.txt)
include(DGtal/math/ModuleSRC.txt)
include(DGtal/kernel/ModuleSRC.txt)
include(DGtal/geometry/ModuleSRC.txt)
include(DGtal/topology/ModuleSRC.txt)
include(DGtal/io-viewers/ModuleSRC.txt)
include(DGtal/helpers/ModuleSRC.txt)

include(Board/ModuleSRC.txt)

# Create a library called "DGtal".
# The extension is already found. Any number of sources could be listed here.
add_library ( ${LIBDGTAL_NAME}  ${DGTAL_SRC} ${BOARD_SRC})
target_link_libraries( ${LIBDGTAL_NAME} ${DGtalLibDependencies} )

install(TARGETS ${LIBDGTAL_NAME} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)


if ( WITH_VISU3D )
    include(DGtal/io-viewers/3dViewers/ModuleSRC.txt)
    add_library ( ${LIBDGTALVISU3D_NAME}  ${DGTALIV_SRC} ${DGTALQGLV_SRC} )
    target_link_libraries( ${LIBDGTALVISU3D_NAME}  ${DGtalLibDependencies} )
    install(TARGETS ${LIBDGTALVISU3D_NAME} 
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)
endif( WITH_VISU3D )


