# - Config file for the DGtal package
# It defines the following variables
#  DGTAL_INCLUDE_DIRS - include directories for DGtal
#  DGTAL_LIBRARY_DIRS - library directories for DGtal (normally not used!)
#  DGTAL_LIBRARIES    - libraries to link against
#  DGTAL_VERSION      - version of the DGtal library
 
# Tell the user project where to find our headers and libraries
set(DGTAL_INCLUDE_DIRS "/usr/local/include;/usr/local/include")
set(DGTAL_LIBRARY_DIRS "/usr/local/lib")
set(DGTAL_VERSION "0.7.beta")

 
# Our library dependencies (contains definitions for IMPORTED targets)
include("/usr/local/share/DGtal/CMake/DGtalLibraryDepends.cmake")



#------------------------------------------------------------------------------
# -- Environement variables
#------------------------------------------------------------------------------
if (UNIX)
  add_definitions(-DUNIX)
endif (UNIX)
if (WIN32)
  add_definitions(-DWIN32)
endif (WIN32)
if (APPLE)
  add_definitions(-DAPPLE)
endif (APPLE)


#------------------------------------------------------------------------------
# -- Removing some strange warnings when compiling with VS Express
#------------------------------------------------------------------------------
IF(MSVC)
 IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    SET(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
  ENDIF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
ENDIF(MSVC)

#------------------------------------------------------------------------------
# Remove some MS Visual c++ flags
#------------------------------------------------------------------------------
IF(MSVC)

  ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)
  #------------------------------------------------------------------------------
  # for GMP / MPIR (MT)
  #------------------------------------------------------------------------------
  SET(CMAKE_EXE_LINKER_FLAGS /NODEFAULTLIB:\"libcmtd.lib;libcmt.lib\")
ENDIF(MSVC)


#------------------------------------------------
#---- We propagate definitions for #define marco 
#---- according to dependencies detected during 
#---- the build.
#------------------------------------------------
IF(0)
  ADD_DEFINITIONS(-DDEBUG_VERBOSE)
  MESSAGE(STATUS "Debug verbose mode activated")
ENDIF(0)

IF (0)
  ADD_DEFINITIONS(-DVERBOSE)
  MESSAGE(STATUS "Verbose mode activated")
ENDIF(0)

IF(0)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  ADD_DEFINITIONS("-DWITH_C11 ")
  SET(WITH_C11 1) 
  IF(0)
    ADD_DEFINITIONS("-DCPP11_AUTO")
    SET(CPP11_AUTO 1)
  ENDIF(0)
  IF(0)
    ADD_DEFINITIONS("-DCPP11_INITIALIZER_LIST")
    SET(CPP11_INITIALIZER_LIST 1)
  ENDIF(0) 
  IF(0)
    ADD_DEFINITIONS("-DCPP11_FORWARD_LIST")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    SET(CPP11_FORWARD_LIST 1)
  ENDIF(0) 
 IF()
    ADD_DEFINITIONS("-DCPP11_ARRAY")
    SET(CPP11_ARRAY 1)
  ENDIF() 
ENDIF(0)


IF(0)
  ADD_DEFINITIONS("-DWITH_GMP ")
  SET(WITH_GMP 1)
ENDIF(0)

IF(0)
  ADD_DEFINITIONS("-DWITH_MAGICK ")
  SET(WITH_MAGICK 1)
ENDIF(0)

IF(0)
  ADD_DEFINITIONS("-DWITH_ITK ")
  SET(WITH_ITK 1)
  #--------------------------------------------
  #ITK issue (we need an explicit find_package)
  #--------------------------------------------
  FIND_PACKAGE(ITK REQUIRED)
  INCLUDE( ${ITK_USE_FILE} )
ENDIF(0)

IF(0)
  ADD_DEFINITIONS("-DWITH_CAIRO ")
  SET(WITH_CAIRO 1)
ENDIF(0)

IF(1)
  ADD_DEFINITIONS("-DWITH_HDF5 ")
  SET(WITH_HDF5 1)
ENDIF(1)

IF(0)
  ADD_DEFINITIONS("-DWITH_SOQT ")
  SET(WITH_SOQT 1)
ENDIF(0)

IF ( 0 AND 0 AND 0 )
  ADD_DEFINITIONS("-DWITH_VISU3D_IV ")
  SET(WITH_VISU3D_IV 1)

ENDIF( 0 AND 0 AND 0 )

IF(0 AND 0 )
  ADD_DEFINITIONS("-DWITH_VISU3D_QGLVIEWER ")
  SET(WITH_VISU3D_QGLVIEWER 1)
  SET(DGTAL_INCLUDE_DIRS ${DGTAL_INCLUDE_DIRS}  )
ENDIF(0  AND 0 )

#-- We force the Qt rediscovering
IF(0 OR 0)
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtGUI QtXml QtOpenGL)
  set(QT_USE_QTXML 1)
  ADD_DEFINITIONS("-DWITH_QT4 ")
  SET(WITH_QT4 1)
  include( ${QT_USE_FILE})
  SET(DGTAL_INCLUDE_DIRS ${DGTAL_INCLUDE_DIRS} ${QT_INCLUDE_DIR} )
ENDIF(0 OR 0)


IF(0)
  ADD_DEFINITIONS("-DWITH_OPENMP ")
  SET(WITH_OPENMP 1)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
  ADD_DEFINITIONS("-DWITH_OPENMP ")
ENDIF(0)

 
# These are IMPORTED targets created by DGtalLibraryDepends.cmake
set(DGTAL_LIBRARIES DGtal DGtalIO)

