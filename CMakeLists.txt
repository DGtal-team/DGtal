# The name of our project is "DGtal". CMakeLists files in this
# project can refer to the root source directory of the project as
# ${DGTAL_SOURCE_DIR} and to the root binary directory of the
# project as ${DGTAL_BINARY_DIR}.

# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
project (DGtal)
cmake_minimum_required (VERSION 2.4) 

#-- We remove the cmake warning on explicit lib paths
#-- With ITK dependency, I don't know how to fix the warning
#-- D.C
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 OLD)
endif(COMMAND cmake_policy)


message( "-------------------------------------------------------------------------------" )
message( "DGtal: see AUTHORS file" )
message( "       You may specify the following options:" )
message( "         - CMAKE_BUILD_TYPE = Debug|Release: choose configuration.")
message( "         - CMAKE_INSTALL_PREFIX = <path>: the place to install DGtal (/usr/local).")
message( "       with the syntax (for instance)")  
message( "         cmake -DCMAKE_BUILD_TYPE:STRING=Debug")
message( "-------------------------------------------------------------------------------" )

set(INSTALL_DOC_PATH ${CMAKE_INSTALL_PREFIX}/doc/${CMAKE_PROJECT_NAME} )

# -----------------------------------------------------------------------------
# Display some information
# -----------------------------------------------------------------------------
message(STATUS "Source DIR is ${PROJECT_SOURCE_DIR}")
message(STATUS "Binary DIR is ${PROJECT_BINARY_DIR}")
message(STATUS "Build type is ${CMAKE_BUILD_TYPE}")
message(STATUS "Libs extension is '${SUFFIXLIB}'. Bins extension is '${SUFFIXBIN}'." )
message(STATUS "Installation prefix directory is " ${CMAKE_INSTALL_PREFIX})
message(STATUS "Host system is " ${CMAKE_HOST_SYSTEM} " with processor " ${CMAKE_HOST_SYSTEM_PROCESSOR})
message(STATUS "Target system is " ${CMAKE_SYSTEM} " with processor " ${CMAKE_SYSTEM_PROCESSOR})

# -- Environement variables
if (UNIX)
  add_definitions(-DUNIX)
endif (UNIX)
if (WIN32)
  add_definitions(-DWIN32)
endif (WIN32)
if (APPLE)
  add_definitions(-DAPPLE)
endif (APPLE)



# -----------------------------------------------------------------------------
# Check and configure CMake packages
# -----------------------------------------------------------------------------
# Look for modules
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

SET(DGtal_VERSION_MAJOR 0)
SET(DGtal_VERSION_MINOR 2)
SET(DGtal_VERSION_PATCH 1)
SET(PROJECT_VERSION "${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH}")

MESSAGE(STATUS "DGtal Version ${PROJECT_VERSION}")

# Needed file 'doxy.config.in' for documentation
INCLUDE(doc/doxygen.cmake)
INCLUDE("${CMAKE_MODULE_PATH}/TargetDoxygenDoc.cmake" OPTIONAL)

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/TargetUninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake")


# -----------------------------------------------------------------------------
# Defines some variables
# -----------------------------------------------------------------------------
set ( LIBDGTAL_NAME "DGtal${SUFFIXLIB}" )
set ( LIBDGTALIV_NAME "DGtal-iv${SUFFIXLIB}" )

# -----------------------------------------------------------------------------
# Parsing cmake options
# -----------------------------------------------------------------------------

OPTION(BUILD_SHARED_LIBS "Build shared libraries." ON)
OPTION(BUILD_INLINE_FUNCTIONS "Build inline functions." ON)
OPTION(DEBUG_VERBOSE "Verbose messages in debug mode." OFF)

if ( ${CMAKE_BUILD_TYPE} MATCHES "Debug" )
  IF (DEBUG_VERBOSE)
    ADD_DEFINITIONS(-DDEBUG_VERBOSE)
    MESSAGE(STATUS "Debug verbose mode activated")
  ENDIF(DEBUG_VERBOSE)
endif( ${CMAKE_BUILD_TYPE} MATCHES "Debug" )

if (BUILD_INLINE_FUNCTIONS)
  ADD_DEFINITIONS(-DINLINE=inline)
else (BUILD_INLINE_FUNCTIONS)
  ADD_DEFINITIONS(-DINLINE="")
endif (BUILD_INLINE_FUNCTIONS)

# -----------------------------------------------------------------------------
# Check CPP0X
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/CheckCPP0X.cmake)

# -----------------------------------------------------------------------------
# Check Dependencies
# -----------------------------------------------------------------------------
INCLUDE(${CMAKE_MODULE_PATH}/CheckDGtalDependencies.cmake)

# -----------------------------------------------------------------------------
# Benchmark target
# -----------------------------------------------------------------------------
ADD_CUSTOM_TARGET(benchmark COMMAND echo "Benchmarks launched.....")

# -----------------------------------------------------------------------------
# CTest options
# -----------------------------------------------------------------------------
ENABLE_TESTING()
include(CTest)
if (BUILD_TESTING)
  if (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pg -O0 -Wall -W -Wshadow -Wunused-variable -Wunused-parameter -Wunused-function        -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -fprofile-arcs -ftest-coverage")
  endif (CMAKE_COMPILER_IS_GNUCXX)
  if (CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -Wall -W -fprofile-arcs -ftest-coverage")
  endif (CMAKE_COMPILER_IS_GNUCC)
  add_subdirectory (tests)
endif (BUILD_TESTING)

#------------------------------------------------------------------------------
# Prepare installations concerning global scope.
#------------------------------------------------------------------------------
if (CMAKE_MAJOR_VERSION EQUAL 2)
  if (CMAKE_MINOR_VERSION GREATER 5)
    install(DIRECTORY src/ DESTINATION include 
      FILES_MATCHING PATTERN "*.*h")
  else (CMAKE_MINOR_VERSION GREATER 5)
    install(DIRECTORY src/ DESTINATION include 
      PATTERN "*.*h")
  endif (CMAKE_MINOR_VERSION GREATER 5)
endif (CMAKE_MAJOR_VERSION EQUAL 2)

#------------------------------------------------------------------------------
# Configuration of the Config.h
#------------------------------------------------------------------------------
CONFIGURE_FILE( "${PROJECT_SOURCE_DIR}/src/DGtal/base/Config.h.in" "${PROJECT_SOURCE_DIR}/src/DGtal/base/Config.h" IMMEDIATE)
SET(VERSION ${DGtal_VERSION_MAJOR}.${DGtal_VERSION_MINOR}.${DGtal_VERSION_PATCH})

#------------------------------------------------------------------------------
# Some directories and files should also be cleaned when invoking 'make clean'
#------------------------------------------------------------------------------
ADD_CUSTOM_TARGET(distclean
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/TargetDistclean.cmake")

#------------------------------------------------------------------------------
# CPack Configurations
#------------------------------------------------------------------------------

SET(CPACK_PACKAGE_VERSION_MAJOR 	 ${DGtal_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR 	 ${DGtal_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH 	 ${DGtal_VERSION_PATCH})
# for other generator set specific backend information
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "DGtal Project")
SET(CPACK_PACKAGE_VENDOR ".")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_PACKAGE_CONTACT "dgtal@liris.cnrs.fr" )
SET(CPACK_NSIS_CONTACT "dgtal@liris.cnrs.fr")

IF(WIN32 AND NOT UNIX)
  SET(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} DGtal")
  SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\liris.cnrs.fr/dgtal")
  SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\liris.cnrs.fr/dgtal")
  SET(CPACK_NSIS_MODIFY_PATH ON)
ELSE(WIN32 AND NOT UNIX)
  ##SET(CPACK_STRIP_FILES "bin/main") ??
  SET(CPACK_SOURCE_STRIP_FILES "")
ENDIF(WIN32 AND NOT UNIX)

SET(CPACK_SOURCE_IGNORE_FILES CVS;[.]svn;dev;[.]git;[.]AppleDouble;[.]\#.*;\#.*;.*[.]kdev4;.*~;build.*;doc/html/*;doc/latex/*;doc/html-Board/*)

if(WIN32)
  SET(CPACK_GENERATOR "NSIS;")
else(WIN32)
  SET(CPACK_GENERATOR "TGZ;DEB")
endif(WIN32)

INCLUDE(CPack)

#------------------------------------------------------------------------------
# Add subdirectories
#------------------------------------------------------------------------------
add_subdirectory (src)
add_subdirectory (tools)


OPTION(BUILD_EXAMPLES "Build examples." ON)
IF (BUILD_EXAMPLES)
  add_subdirectory (examples)
ENDIF(BUILD_EXAMPLES)


#------------------------------------------------------------------------------
# DGtal Configuration file
#------------------------------------------------------------------------------
CONFIGURE_FILE( "${PROJECT_SOURCE_DIR}/cmake/DGtalConfig.cmake.in" "${PROJECT_SOURCE_DIR}/build/DGtalConfig.cmake" IMMEDIATE)
export_library_dependencies(${PROJECT_BINARY_DIR}/DGtalLibDependencies.cmake )


