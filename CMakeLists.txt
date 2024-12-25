# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
cmake_minimum_required (VERSION 3.11)
cmake_policy(SET CMP0057 NEW) # the new interpretation of IN_LIST is by default not set (at least until  3.21.1).
project (DGtal)

# -----------------------------------------------------------------------------
# By default, do not warn when built on machines using only VS Express:
# -----------------------------------------------------------------------------
if(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
endif()

# -----------------------------------------------------------------------------
# Define variables and cmake parameters, and display some information
# -----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/deps)



set(DGtal_VERSION_MAJOR 2)
set(DGtal_VERSION_MINOR 0)
set(DGtal_VERSION_PATCH beta)
set(DGTAL_VERSION "${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH}")
set(PROJECT_VERSION "${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH}")
set(VERSION ${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH})
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
# Include CPP files to add to the library target
#
# each subdir can contain a ModuleSRC.txt file
# with a set command on the variable ${DGTAL_SRC}
#
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

# -----------------------------------------------------------------------------
# Check mandatory and optional dependencies
# -----------------------------------------------------------------------------
include(CheckDGtalDependencies)
include(CheckDGtalOptionalDependencies)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(TARGETS DGtal
  # IMPORTANT: Add the DGtal library to the "export-set"
  EXPORT DGtalLibraryDepends
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
 #PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/DGtal"
  COMPONENT dev)

# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
include(OSDependentSettings)

# -----------------------------------------------------------------------------
# Randomized build 1/2.
# -----------------------------------------------------------------------------
SET(DGTAL_RANDOMIZED_TESTING_THRESHOLD "100" CACHE INTERNAL "Threshold for the random selection of unit tests to build.")
SET(DGTAL_RANDOMIZED_TESTING_WHITELIST "" CACHE INTERNAL "List of whitelisted unit-test/examples to build.")

#------------------------------------------------------------------------------
# Examples
#------------------------------------------------------------------------------
include(BuildExamples)

# -----------------------------------------------------------------------------
# Unit-testing, Cpack and Ctest settings
# -----------------------------------------------------------------------------
include(CpackCtest)

# -----------------------------------------------------------------------------
# Randomized build 2/2.
# -----------------------------------------------------------------------------
if (DGTAL_BUILD_EXAMPLES OR DGTAL_BUILD_TESTS)
 message(STATUS "Randomized examples/testing")
 if (DGTAL_RANDOMIZED_TESTING_THRESHOLD EQUAL "100")
  message(STATUS "All examples or unit-tests will be compiled (cmake variable DGTAL_RANDOMIZED_TESTING_THRESHOLD)")
 else()
  message(STATUS "Only ~${DGTAL_RANDOMIZED_TESTING_THRESHOLD}% of the (randomly selected) examples/tests will be  compiled/run")
  message(STATUS "  (you can adjust this with the cmake variable DGTAL_RANDOMIZED_TESTING_THRESHOLD)")
  message(STATUS " Whitelist: ${DGTAL_RANDOMIZED_TESTING_WHITELIST}")
endif()
message(STATUS "-------------------------------------------------------------------------------")
endif()

# -----------------------------------------------------------------------------
# Custom command/targets depending on the cmake generator
# -----------------------------------------------------------------------------
include(GeneratorSpecific)

# -----------------------------------------------------------------------------
# Common build options/settings
# -----------------------------------------------------------------------------
include(OSDependentSettings)

# -----------------------------------------------------------------------------
# Install settings
# -----------------------------------------------------------------------------
include(Install)

#------------------------------------------------------------------------------
# Python wrappings
#------------------------------------------------------------------------------
include(PythonWrappings)

# -----------------------------------------------------------------------------
# Unzip and install topology Look up Tables.
# -----------------------------------------------------------------------------
include(NeighborhoodTablesConfig)
