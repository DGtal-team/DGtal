MESSAGE(STATUS "Generating DGtalConfig files")

set(dgtal_export_file "${PROJECT_BINARY_DIR}/DGtalLibraryDepends.cmake")
export(TARGETS DGtal FILE ${dgtal_export_file})

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
# export(PACKAGE DGtal)
# Not working on cmake 2.6, I remove this option but keep the codeline (DC)


# Provide HINTS to find_dependency in DGtalConfig.cmake if DGTAL_CONFIG_HINTS
# is ON. Disable it when deploying.
set(_dependencies_list
  Boost ZLIB
  GMP Magick ITK Cairo HDF5 QGLVIEWER Qt5 OpenMP Eigen3 CGAL FFTW3
  )
foreach(dep ${_dependencies_list})
  if(DGTAL_CONFIG_HINTS)
    if("${${dep}_DIR}" STREQUAL "" OR
       "${${dep}_DIR}" STREQUAL "${dep}_DIR-NOTFOUND")
      set(${dep}_HINTS "# NO_HINTS (no ${dep}_DIR or ${dep}_DIR-NOTFOUND)")
    else()
      set(${dep}_HINTS "HINTS \"${${dep}_DIR}\"")
    endif()
  else()
    set(${dep}_HINTS "# NO_HINTS (disabled with DGTAL_CONFIG_HINTS=OFF)")
  endif()
endforeach()
# Create a DGtalConfig.cmake file for the use from the build tree
configure_file(${PROJECT_SOURCE_DIR}/cmake/DGtalConfig.cmake.in
  "${PROJECT_BINARY_DIR}/DGtalConfig.cmake" @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/cmake/DGtalConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/DGtalConfigVersion.cmake" @ONLY)

# Install the export set for use with the install-tree
set(DGTAL_CMAKE_DIR_INSTALL "${INSTALL_DATA_DIR}")
install(EXPORT DGtalLibraryDepends DESTINATION
  ${DGTAL_CMAKE_DIR_INSTALL}
  COMPONENT dev)

# Create a DGtalConfig.cmake file for the use from the install tree
# and install it
configure_file(${PROJECT_SOURCE_DIR}/cmake/DGtalConfig.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/DGtalConfig.cmake" @ONLY)

configure_file(${PROJECT_SOURCE_DIR}/cmake/DGtalConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/DGtalConfigVersion.cmake" @ONLY)
install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/DGtalConfig.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/DGtalConfigVersion.cmake"
  DESTINATION "${DGTAL_CMAKE_DIR_INSTALL}" COMPONENT dev)

# Distribute FindFoo.cmake files to the build and install tree
set(_find_cmake_files
  "${PROJECT_SOURCE_DIR}/cmake/FindCairo.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/FindFFTW3.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/FindMagick.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/FindQGLVIEWER.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/FindGMP.cmake"
  )
file(COPY ${_find_cmake_files}
  DESTINATION "${PROJECT_BINARY_DIR}/Modules")
install(FILES ${_find_cmake_files}
  DESTINATION "${DGTAL_CMAKE_DIR_INSTALL}/Modules" COMPONENT dev)
