#CMakeLists associated to the examples subdir


# Make sure the compiler can find include files.
if ( CPP0X_INITIALIZER_LIST )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP0X_INITIALIZER_LIST )

# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/src/)
# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/examples/)


# Make sure the linker can find the Hello library once it is built.
link_directories (${PROJECT_BINARY_DIR})


# ConfigTest.h instanciation.
configure_file(${PROJECT_SOURCE_DIR}/examples/ConfigExamples.h.in ${PROJECT_SOURCE_DIR}/examples/ConfigExamples.h IMMEDIATE)

#------TESTS subdirectories ------

add_subdirectory(io)
add_subdirectory(geometry)
add_subdirectory(doc-examples)
add_subdirectory(topology)


if( WITH_VISU3D_QGLVIEWER  )
add_subdirectory(3dViewer)
endif ( WITH_VISU3D_QGLVIEWER )

if( WITH_CAIRO )
add_subdirectory(CairoViewers)
endif ( WITH_CAIRO )

