set(dgtal_export_file "${PROJECT_BINARY_DIR}/DGtalLibraryDepends.cmake")

include(GNUInstallDirs) # Define CMAKE_INSTALL_xxx: LIBDIR, INCLUDEDIR
set(CMAKE_INSTALL_PYTHONLIBDIR dgtal) # Install folder for python libs
# Copy python libs to a single location at build time (for ease of use)
set(CMAKE_BUILD_PYTHONLIBDIR ${PROJECT_BINARY_DIR}/dgtal) # copy python libs at build time
# Python wrapping will be one single module with submodules
set(CMAKE_INSTALL_PYTHONINCLUDEDIR
  ${CMAKE_INSTALL_PYTHONLIBDIR}/include)
set(enabled_libs_ DGtal)
set(enabled_include_dirs_)
set(enabled_include_system_dirs_)

add_subdirectory(kernel)
add_subdirectory(base)
#TODO add_subdirectory(arithmetic)
#TODO add_subdirectory(dec)
#TODO add_subdirectory(geometry)
#TODO add_subdirectory(graph)
#TODO add_subdirectory(helpers)
#TODO add_subdirectory(images)
#TODO add_subdirectory(io)
#TODO add_subdirectory(math)
#TODO add_subdirectory(shapes)
#TODO add_subdirectory(topology)

if(WITH_ITK)
  # Add itk wrapping of Image
  #TODO add_subdirectory(bridges/itk)
endif()

set(wrap_header_common
  dgtal_pybind11_common.h)

# Add the pybind module
# Use SHARED instead of the deault MODULE to allow external plugins to link to it.
# This would require the same CI/CD to build and link these external plugins.
pybind11_add_module(_dgtal
  SHARED
  dgtal_init_py.cpp # Where module is added
  ${all_modules_python_sources} # all the collected sources in submodules
  )

add_library(DGtal::_dgtal ALIAS _dgtal)
target_link_libraries(_dgtal PRIVATE ${enabled_libs_})
target_include_directories(_dgtal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  # TODO remove next two line when modernize cmake is integrated.
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src/>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PYTHONINCLUDEDIR}>
  ) # For dgtal_pybind11_common.h
target_include_directories(_dgtal PRIVATE ${enabled_include_dirs_})
target_include_directories(_dgtal SYSTEM PRIVATE ${enabled_include_system_dirs_})
if(WITH_ITK)
  target_compile_definitions(_dgtal PRIVATE WITH_ITK)
endif()
if(WITH_OPENMP)
  target_compile_definitions(_dgtal PRIVATE WITH_OPENMP)
endif()
if(WITH_FFTW3)
  target_compile_definitions(_dgtal PRIVATE WITH_FFTW3)
endif()

install(FILES ${wrap_header_common}
  DESTINATION ${CMAKE_INSTALL_PYTHONINCLUDEDIR}
  COMPONENT development)

install(FILES __init__.py
  DESTINATION ${CMAKE_INSTALL_PYTHONLIBDIR}
  COMPONENT runtime
  )
# Copy __init__ at configure time
file(COPY __init__.py DESTINATION ${CMAKE_BUILD_PYTHONLIBDIR})
# Copy at build time
add_custom_command(
  TARGET _dgtal
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  $<TARGET_FILE:_dgtal>
  ${CMAKE_BUILD_PYTHONLIBDIR}/$<TARGET_FILE_NAME:_dgtal>
  )

# Copy and install fixtures
file(COPY fixtures DESTINATION ${CMAKE_BUILD_PYTHONLIBDIR})
install(DIRECTORY fixtures
  DESTINATION ${CMAKE_INSTALL_PYTHONLIBDIR}
  COMPONENT runtime
  )
# Copy and install tables (package_data)
set(table_folder "${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables")
file(COPY ${table_folder}
    DESTINATION ${CMAKE_BUILD_PYTHONLIBDIR}
    FILES_MATCHING PATTERN "*.zlib")
install(DIRECTORY ${table_folder}
  DESTINATION ${CMAKE_INSTALL_PYTHONLIBDIR}
  COMPONENT runtime
  FILES_MATCHING PATTERN "*.zlib"
  )

# Export _dgtal to let external plugins link to DGtal::_dgtal
# export to install tree: (always to CMAKE_INSTALL_PYTHONLIBDIR)
install(TARGETS _dgtal
  EXPORT DGtalLibraryDepends
  RUNTIME DESTINATION ${CMAKE_INSTALL_PYTHONLIBDIR} COMPONENT runtime
  LIBRARY DESTINATION ${CMAKE_INSTALL_PYTHONLIBDIR} COMPONENT runtime
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PYTHONLIBDIR} COMPONENT development)
# export _dgtal to the build tree
export(TARGETS _dgtal
  NAMESPACE DGtal::
  APPEND FILE ${dgtal_export_file})

option(DGTAL_BUILD_TESTING_PYTHON "Enable python testing" ON)
if(NOT BUILD_TESTING AND DGTAL_BUILD_TESTING_PYTHON)
  enable_testing()
endif()
if(BUILD_TESTING OR DGTAL_BUILD_TESTING_PYTHON)
  add_subdirectory(test)
endif()
