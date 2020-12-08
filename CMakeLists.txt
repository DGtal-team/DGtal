# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
project (DGtal)
cmake_minimum_required (VERSION 3.11)

# -----------------------------------------------------------------------------
# By default, do not warn when built on machines using only VS Express:
# -----------------------------------------------------------------------------
if(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
endif()

# -----------------------------------------------------------------------------
# Define variables and cmake parameters, and display some information
# -----------------------------------------------------------------------------
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
set(DGtal_VERSION_MAJOR 1)
set(DGtal_VERSION_MINOR 2)
set(DGtal_VERSION_PATCH beta)
set(DGTAL_VERSION "${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH}")
set(PROJECT_VERSION "${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH}")
set(VERSION ${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH})
# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
include(Common)

#------------------------------------------------------------------------------
# Add DGtal library
#------------------------------------------------------------------------------

# Include CPP files to add to the library target
#
# each subdir can contain a ModuleSRC.txt file
# with a set command on the variable ${DGTAL_SRC}
#
include(src/DGtal/kernel/ModuleSRC.cmake)
include(src/DGtal/base/ModuleSRC.cmake)
include(src/DGtal/io/ModuleSRC.cmake)
include(src/DGtal/helpers/ModuleSRC.cmake)
## Board dependency
include(src/Board/ModuleSRC.cmake)
## Boost Add-ons
include(src/BoostAddons/ModuleSRC.cmake)

list(TRANSFORM DGTAL_SRC PREPEND "src/")
list(TRANSFORM BOARD_SRC PREPEND "src/")
list(TRANSFORM DGTALIO_SRC PREPEND "src/")

# DGtal Target. Library is added on the top directory to use target_link_libraries for CMake < 3.13
add_library (DGtal ${DGTAL_SRC} ${DGTALIO_SRC} ${BOARD_SRC} )
target_include_directories(DGtal PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
  # INSTALL_INTERFACE must not contain absolute paths
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR_RELATIVE}>
  )

install(TARGETS DGtal
  # IMPORTANT: Add the DGtal library to the "export-set"
  EXPORT DGtalLibraryDepends
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
 #PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/DGtal"
  COMPONENT dev)

# Misc
if( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
  # Add -fPIC
  set_target_properties(DGtal PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# -----------------------------------------------------------------------------
# Development and debug options
# -----------------------------------------------------------------------------
include(DevelopmentOptions)

# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
include(OSDependentSettings)

# -----------------------------------------------------------------------------
# Check Mandatory Dependencies
# -----------------------------------------------------------------------------
include(CheckDGtalDependencies)

# -----------------------------------------------------------------------------
# Check Optional Dependencies
# -----------------------------------------------------------------------------
include(CheckDGtalOptionalDependencies)

# -----------------------------------------------------------------------------
# Debug, Cpack and Ctest settings
# -----------------------------------------------------------------------------
include(CpackCtest)

# -----------------------------------------------------------------------------
# Code coverage
# -----------------------------------------------------------------------------
include(CheckCoverage)

# -----------------------------------------------------------------------------
# Custom command/targets depending on the cmake generator
# -----------------------------------------------------------------------------
include(GeneratorSpecific)

#------------------------------------------------------------------------------
# Examples
#------------------------------------------------------------------------------
include(BuildExamples)

# -----------------------------------------------------------------------------
# Install settings
# -----------------------------------------------------------------------------
include(Install)

# -----------------------------------------------------------------------------
# Unzip and install topology Look up Tables.
# -----------------------------------------------------------------------------
include(NeighborhoodTablesConfig)
