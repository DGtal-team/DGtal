# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
project (DGtal)
cmake_minimum_required (VERSION 2.6) 

# -----------------------------------------------------------------------------
# Define variables and cmake parameters, and display some information
# -----------------------------------------------------------------------------
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
SET(DGtal_VERSION_MAJOR 0)
SET(DGtal_VERSION_MINOR 4)
SET(DGtal_VERSION_PATCH 1devel)
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
# Check CPP0X
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/CheckCPP0X.cmake)

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

#------------------------------------------------------------------------------
# Add subdirectories
#------------------------------------------------------------------------------
add_subdirectory(src)
add_subdirectory(tools)

OPTION(BUILD_EXAMPLES "Build examples." ON)
IF (BUILD_EXAMPLES)
  add_subdirectory (examples)
ENDIF(BUILD_EXAMPLES)


# -----------------------------------------------------------------------------
# Install settings
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/Install.cmake)
