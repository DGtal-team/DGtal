# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
cmake_minimum_required (VERSION 3.20)
project (DGtal VERSION 2.0.0)

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

#------------------------------------------------------------------------------
# Add DGtal library
#------------------------------------------------------------------------------
add_subdirectory(src)

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

# -----------------------------------------------------------------------------
# Check mandatory and optional dependencies
# -----------------------------------------------------------------------------
include(CheckDGtalDependencies)
include(CheckDGtalOptionalDependencies)

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
