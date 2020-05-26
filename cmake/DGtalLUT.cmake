option(WITH_DGtalLUT "Generate library DGtalLUT with all LookUpTables pre-loaded" OFF)

if(WITH_DGtalLUT)
  set(_lut_sources
    ${PROJECT_SOURCE_DIR}/src/DGtal/topology/tables/NeighborhoodLUT.cpp
    )

  if (BUILD_SHARED_LIBS)
    add_library(DGtalLUT SHARED ${_lut_sources})
  else()
    add_library(DGtalLUT STATIC ${_lut_sources})
  endif()

  target_link_libraries(DGtalLUT DGtal)
  target_include_directories(DGtalLUT PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
    $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
    )

  # Exporting to the build tree is done in DGtalConfigGenerator.cmake

  # Export to the install tree
  install(TARGETS DGtalLUT
    # IMPORTANT: Add the DGtal library to the "export-set"
    EXPORT DGtalLibraryDepends
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
    #PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/DGtal"
    COMPONENT dev
    COMPONENT DGtalLUT)

endif()
