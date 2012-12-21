#------------------------------------------------------------------------------
# -- Messages
#------------------------------------------------------------------------------
message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "DGtal Version ${PROJECT_VERSION}")
message(STATUS "-------------------------------------------------------------------------------" )
message(STATUS "Source DIR is ${PROJECT_SOURCE_DIR}")
message(STATUS "Binary/Build DIR is ${PROJECT_BINARY_DIR}")
message(STATUS "Build type is ${CMAKE_BUILD_TYPE}")
message(STATUS "Installation prefix directory is " ${CMAKE_INSTALL_PREFIX})
message(STATUS "Host system is " ${CMAKE_HOST_SYSTEM} " with processor " ${CMAKE_HOST_SYSTEM_PROCESSOR})
message(STATUS "Target system is " ${CMAKE_SYSTEM} " with processor " ${CMAKE_SYSTEM_PROCESSOR})

#------------------------------------------------------------------------------
# Offer the user the choice of overriding the installation directories
#------------------------------------------------------------------------------
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(INSTALL_DATA_DIR share CACHE PATH "Installation directory for data files")

#------------------------------------------------------------------------------
# Make relative paths absolute (needed later on)
#------------------------------------------------------------------------------
foreach(p LIB BIN INCLUDE DATA)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# -----------------------------------------------------------------------------
# Doxygen targets
# -----------------------------------------------------------------------------
set(INSTALL_DOC_PATH ${CMAKE_INSTALL_PREFIX}/doc/${CMAKE_PROJECT_NAME} )
INCLUDE(${CMAKE_MODULE_PATH}/doxygen.cmake)
INCLUDE(${CMAKE_MODULE_PATH}/TargetDoxygenDoc.cmake OPTIONAL)
INCLUDE(${CMAKE_MODULE_PATH}/TargetDoxygenDox.cmake OPTIONAL)

# -----------------------------------------------------------------------------
# uninstall target
# -----------------------------------------------------------------------------
CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/TargetUninstall.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake
  @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake")

# -----------------------------------------------------------------------------
# Parsing cmake options
# -----------------------------------------------------------------------------
OPTION(BUILD_SHARED_LIBS "Build shared libraries." ON)
OPTION(DEBUG_VERBOSE "Verbose messages in debug mode." OFF)

if ( ${CMAKE_BUILD_TYPE} MATCHES "Debug" )
  IF (DEBUG_VERBOSE)
    ADD_DEFINITIONS(-DDEBUG_VERBOSE)
    MESSAGE(STATUS "Debug verbose mode activated")
  ENDIF(DEBUG_VERBOSE)
endif( ${CMAKE_BUILD_TYPE} MATCHES "Debug" )

# Functions are INLINE only in Release mode
if ( ${CMAKE_BUILD_TYPE} MATCHES "Release" )
    ADD_DEFINITIONS(-DINLINE=inline)
    ADD_DEFINITIONS(-DBUILD_INLINE=)
else ( ${CMAKE_BUILD_TYPE} MATCHES "Release" )
    ADD_DEFINITIONS(-DINLINE=)
endif ( ${CMAKE_BUILD_TYPE} MATCHES "Release" )

# -----------------------------------------------------------------------------
# Benchmark target
# -----------------------------------------------------------------------------
ADD_CUSTOM_TARGET(benchmark COMMAND echo "Benchmarks launched.....")

#------------------------------------------------------------------------------
# Some directories and files should also be cleaned when invoking 'make clean'
#------------------------------------------------------------------------------
ADD_CUSTOM_TARGET(distclean
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/TargetDistclean.cmake")

#------------------------------------------------------------------------------
# Configuration of the Config.h
#------------------------------------------------------------------------------
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/src/DGtal/base/Config.h.in
  ${PROJECT_BINARY_DIR}/src/DGtal/base/Config.h)

#------------------------------------------------------------------------------
# Building in the source tree is forbidden
#------------------------------------------------------------------------------
IF(PROJECT_BINARY_DIR STREQUAL ${PROJECT_SOURCE_DIR})
  MESSAGE(STATUS "Building in the source tree is not a good idea ! Remove the file 'CMakeCache.txt' and the folder 'CMakeFiles' an
d build outside the sources (for example 'mkdir build ; cmake <DGTAL_DIR>'.")
ENDIF(PROJECT_BINARY_DIR STREQUAL ${PROJECT_SOURCE_DIR})
