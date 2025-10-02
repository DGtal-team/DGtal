# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
cmake_minimum_required (VERSION 3.20)

# Reading version 
file(READ "VERSION" version_file_raw)
string(REGEX REPLACE "[\r\n]" "" version_file "${version_file_raw}")
string(REGEX MATCHALL "([0-9]+)" version_parts ${version_file})

set(DGtal_VERSION ${version_file})
list(GET version_parts 0 DGtal_VERSION_MAJOR)
list(GET version_parts 1 DGtal_VERSION_MINOR)
list(GET version_parts 2 DGtal_VERSION_PATCH)

set(PROJECT_VERSION ${DGtal_VERSION})
set(VERSION ${DGtal_VERSION})

project (DGtal VERSION ${DGtal_VERSION})

# Allow for cmake 3.5. This is required indirectly by polyscope (via glad)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
# -----------------------------------------------------------------------------
# By default, do not warn when built on machines using only VS Express:
# -----------------------------------------------------------------------------
if(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/deps)

# @TODO : See if those variable are still usefull,
#         Note that they are set in source code !
set(DGtal_VERSION_MAJOR ${CMAKE_PROJECT_VERSION_MAJOR})
set(DGtal_VERSION_MINOR ${CMAKE_PROJECT_VERSION_MINOR})
set(DGtal_VERSION_PATCH ${CMAKE_PROJECT_VERSION_PATCH})
set(PROJECT_VERSION ${CMAKE_PROJECT_VERSION})
set(VERSION ${CMAKE_PROJECT_VERSION})

# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
include(Common)

#------------------------------------------------------------------------------
# Add DGtal library
#------------------------------------------------------------------------------
add_subdirectory(src)

# -----------------------------------------------------------------------------
# Development and debug options
# -----------------------------------------------------------------------------
include(DevelopmentOptions)

# -----------------------------------------------------------------------------
# Check mandatory and optional dependencies
# -----------------------------------------------------------------------------
include(CheckDGtalDependencies)
include(CheckDGtalOptionalDependencies)

# -----------------------------------------------------------------------------
# Unit-testing, Cpack and Ctest settings
# -----------------------------------------------------------------------------
include(CpackCtest)

#------------------------------------------------------------------------------
# Tests & Benchmarks
#------------------------------------------------------------------------------
include(BuildTests)

#------------------------------------------------------------------------------
# Examples
#------------------------------------------------------------------------------
include(BuildExamples)


# -----------------------------------------------------------------------------
# Custom command/targets depending on the cmake generator
# -----------------------------------------------------------------------------
include(GeneratorSpecific)

# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
include(OSDependentSettings)

#------------------------------------------------------------------------------
# Python wrappings
#------------------------------------------------------------------------------
include(PythonWrappings)

# -----------------------------------------------------------------------------
# Unzip and install topology Look up Tables.
# -----------------------------------------------------------------------------
include(NeighborhoodTablesConfig)

# -----------------------------------------------------------------------------
# Unzip and install topology Look up Tables.
# -----------------------------------------------------------------------------
include(Install)
