# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
project (DGtal)
cmake_minimum_required (VERSION 2.6) 

# By default, do not warn when built on machines using only VS Express:
IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    SET(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
ENDIF()

# -----------------------------------------------------------------------------
# Define variables and cmake parameters, and display some information
# -----------------------------------------------------------------------------
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
SET(DGtal_VERSION_MAJOR 0)
SET(DGtal_VERSION_MINOR 5)
SET(DGtal_VERSION_PATCH 0devel)
SET(DGTAL_VERSION "${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH}")
SET(PROJECT_VERSION "${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH}")
SET(VERSION ${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH})
# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/Common.cmake)

# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/OSDependentSettings.cmake)

# -----------------------------------------------------------------------------
# Check Mandatory Dependencies
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/CheckDGtalDependencies.cmake)

# -----------------------------------------------------------------------------
# Check Optional Dependencies
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/CheckDGtalOptionalDependencies.cmake)

# -----------------------------------------------------------------------------
# Cpack and Ctest settings
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/CpackCtest.cmake)

# -----------------------------------------------------------------------------
# This test is for instance used for ITK v3.x. ITK forces a limited
# template depth. We remove this option.
# -----------------------------------------------------------------------------
if(CMAKE_CXX_FLAGS MATCHES "-ftemplate-depth-[0-9]*")
  message( "Warning: some package has enabled a limited template depth for the C++ compiler." )
  message( "         Disabling option -ftemplate-depth-xx in CMAKE_CXX_FLAGS." )
  set( CMAKE_CXX_FLAGS_TMP ${CMAKE_CXX_FLAGS} )
  STRING( REGEX REPLACE "-ftemplate-depth-[0-9]*" "" 
    CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_TMP}" )
  message ("         CMAKE_CXX_FLAGS=" ${CMAKE_CXX_FLAGS} )
endif(CMAKE_CXX_FLAGS MATCHES "-ftemplate-depth-[0-9]*")


#------------------------------------------------------------------------------
# Add subdirectories
#------------------------------------------------------------------------------
add_subdirectory(src)

OPTION(BUILD_EXAMPLES "Build examples." ON)
IF (BUILD_EXAMPLES)
  add_subdirectory (examples)
ENDIF(BUILD_EXAMPLES)

# -----------------------------------------------------------------------------
# Install settings
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/Install.cmake)


