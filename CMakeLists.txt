# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
cmake_minimum_required (VERSION 3.20)
cmake_policy(SET CMP0057 NEW) # the new interpretation of IN_LIST is by default not set (at least until  3.21.1).
project (DGtal VERSION 1.4.2)

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
set(DGTAL_VERSION ${CMAKE_PROJECT_VERSION})
set(PROJECT_VERSION ${CMAKE_PROJECT_VERSION})
set(VERSION ${CMAKE_PROJECT_VERSION})

# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
include(Common)

# -----------------------------------------------------------------------------
# Development and debug options
# -----------------------------------------------------------------------------
include(DevelopmentOptions)

# -----------------------------------------------------------------------------
# Downloading external deps
# -----------------------------------------------------------------------------
message(STATUS "Downloading external projects")
include(FetchExternalDeps)
message(STATUS "Done.")


#------------------------------------------------------------------------------
# Add DGtal library
#------------------------------------------------------------------------------
add_subdirectory(src)

# -----------------------------------------------------------------------------
# Check mandatory and optional dependencies
# -----------------------------------------------------------------------------
include(CheckDGtalDependencies)
include(CheckDGtalOptionalDependencies)

# add_library(DGtal SHARED ${DGTAL_SRC})
# target_include_directories(DGtal PUBLIC src/DGtal)
# target_link_libraries(DGtal PUBLIC LibBoard PRIVATE BoostAddons DGtal_STB)