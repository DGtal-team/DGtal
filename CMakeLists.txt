# The name of our project is "DGtal". CMakeLists files in this
# project can refer to the root source directory of the project as
# ${DGTAL_SOURCE_DIR} and to the root binary directory of the
# project as ${DGTAL_BINARY_DIR}.

# -----------------------------------------------------------------------------
# Project definition
# -----------------------------------------------------------------------------
project (DGtal)
cmake_minimum_required (VERSION 2.4) 

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
# Test some c++0x functionalities
# -----------------------------------------------------------------------------
try_compile( CPP0X_INITIALIZER_LIST 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/tests/cpp0x/initializer_list.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP0X_INITIALIZER_LIST )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP0X_INITIALIZER_LIST")
  message( "-- [c++0x] with initializer_list" )
else ( CPP0X_INITIALIZER_LIST )
  message( "-- [c++0x] initializer_list not found" )
endif ( CPP0X_INITIALIZER_LIST )

try_compile( CPP0X_AUTO 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/tests/cpp0x/auto.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP0X_AUTO )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP0X_AUTO")
  message( "-- [c++0x] with auto" )
else ( CPP0X_AUTO )
  message( "-- [c++0x] auto not found" )
endif ( CPP0X_AUTO )

# -----------------------------------------------------------------------------
# Check and configure CMake packages
# -----------------------------------------------------------------------------
# Look for modules
set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
list( APPEND CMAKE_MODULE_PATH . )


# nead file 'doxy.config.in' for documentation
include(doc/doxygen.cmake)
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
INCLUDE("${CMAKE_MODULE_PATH}/TargetDoxygenDoc.cmake" OPTIONAL)

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/TargetUninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake")

# -----------------------------------------------------------------------------
# Check Dependencies
# -----------------------------------------------------------------------------

set ( CMAKE_PREFIX_PATH "/usr:/usr/local:/opt/local:${CMAKE_PREFIX_PATH}" )
message(STATUS "CMAKE_PREFIX_PATH = " ${CMAKE_PREFIX_PATH} )

##########Check if boost exists (2 round)
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)

FIND_PACKAGE(Boost 1.40.0 REQUIRED COMPONENTS serialization program_options)
if ( Boost_FOUND )
  message(STATUS "Boost and boost_serialization/boost_program_options found. include=${Boost_INCLUDE_DIR} libs=${Boost_LIBRARIES}" )
  include_directories( ${Boost_INCLUDE_DIR} )
endif( Boost_FOUND )


##########Check if libMagick++ exists
FIND_PACKAGE(Magick)
IF(MAGICK++_FOUND)
  INCLUDE_DIRECTORIES(${MAGICK++_INCLUDE_DIR})
  message( "-- GraphicsMagick++ found." )
ENDIF(MAGICK++_FOUND)

# -----------------------------------------------------------------------------
# Defines some variables
# -----------------------------------------------------------------------------
set ( LIBDGTAL_NAME "DGtal${SUFFIXLIB}" )

# -----------------------------------------------------------------------------
# Parsing cmake options
# -----------------------------------------------------------------------------

OPTION(BUILD_SHARED_LIBS "Build shared libraries." ON)

OPTION(INLINE_FUNCTION_BUILD "Build inline functions." ON)

if (INLINE_FUNCTION_BUILD)
  ADD_DEFINITIONS(-DINLINE=inline)
else (INLINE_FUNCTION_BUILD)
  ADD_DEFINITIONS(-DINLINE="")
endif (INLINE_FUNCTION_BUILD)


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
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -W -Wshadow -Wunused-variable -Wunused-parameter -Wunused-function        -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -fprofile-arcs -ftest-coverage")
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
SET(DGtal_VERSION_MAJOR 0)
SET(DGtal_VERSION_MINOR 1)
SET(DGtal_VERSION_PATCH 0)
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

SET(CPACK_SOURCE_IGNORE_FILES CVS;[.]svn;[.]git;[.]AppleDouble;[.]\#.*;\#.*;.*[.]kdev4;.*~;build.*;doc/html/*;doc/html-Board/*)

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

OPTION(BUILD_EXAMPLES "Build examples." ON)
IF (BUILD_EXAMPLES)
  add_subdirectory (examples)
ENDIF(BUILD_EXAMPLES)
