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
include(DGtal/io/ModuleSRC.txt)
include(DGtal/helpers/ModuleSRC.txt)

include(Board/ModuleSRC.txt)

# Create a library called "DGtal".
# The extension is already found. Any number of sources could be listed here.
add_library ( ${LIBDGTAL_NAME}  ${DGTAL_SRC} ${BOARD_SRC})
target_link_libraries( ${LIBDGTAL_NAME}  ${Boost_LIBRARIES})

install(TARGETS ${LIBDGTAL_NAME} 
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

if ( COIN3D_FOUND AND QT4_FOUND AND SOQT_FOUND )
   ## each subdir must contain a ModuleSRC.txt file with a set command on the variable ${DGTALIV_SRC}
   include(DGtal/inventor/ModuleSRC.txt)
   add_library ( ${LIBDGTALIV_NAME}  ${DGTALIV_SRC} )
   target_link_libraries( ${LIBDGTALIV_NAME}  ${Boost_LIBRARIES}
   			  ${SOQT_LIBRARY} ${COIN3D_LIBRARY} ${QT_LIBRARIES} )
   install(TARGETS ${LIBDGTALIV_NAME} 
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

endif ( COIN3D_FOUND AND QT4_FOUND AND SOQT_FOUND )
