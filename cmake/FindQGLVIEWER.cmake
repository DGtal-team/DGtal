# - Try to find QGLViewer
# Once done this will define
#
#  QGLVIEWER_FOUND - system has QGLViewer
#  QGLVIEWER_INCLUDE_DIR - the QGLViewer include directory
#  QGLVIEWER_LIBRARIES - Link these to use QGLViewer
#  QGLVIEWER_DEFINITIONS - Compiler switches required for using QGLViewer
#

find_path(QGLVIEWER_INCLUDE_DIR
  NAMES QGLViewer/qglviewer.h
  PATHS /usr/include
  /usr/local/include
  /usr/local/lib/
  /opt/homebrew/lib/
  /Library/Frameworks/
  ENV QGLVIEWERROOT
  )


find_library(QGLVIEWER_LIBRARY_RELEASE
  NAMES qglviewer-qt4 qglviewer QGLViewer QGLViewer2 QGLViewer-qt5
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/homebrew/lib
  /Library/Frameworks/
  ENV QGLVIEWERROOT
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/release
  )

find_library(QGLVIEWER_LIBRARY_DEBUG
  NAMES dqglviewer dQGLViewer dQGLViewer2
  PATHS
  /opt/homebrew/lib
  /usr/lib
  /usr/local/lib
  /usr/local/lib/QGLViewer.framework
  /Library/Frameworks/QGLViewer.framework
  ENV QGLVIEWERROOT
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/debug
  )

if(QGLVIEWER_LIBRARY_RELEASE)
  if(QGLVIEWER_LIBRARY_DEBUG)
    set(QGLVIEWER_LIBRARIES_ optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
  else()
    set(QGLVIEWER_LIBRARIES_ ${QGLVIEWER_LIBRARY_RELEASE})
  endif()

  set(QGLVIEWER_LIBRARIES ${QGLVIEWER_LIBRARIES_} CACHE FILEPATH "The QGLViewer library")

endif()

if(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARIES)
  set(QGLVIEWER_FOUND TRUE)
endif()

if(QGLVIEWER_FOUND)
  if(NOT QGLViewer_FIND_QUIETLY)
    #MESSAGE(STATUS "Found QGLViewer: ${QGLVIEWER_LIBRARIES}")
  endif()
else()
  if(QGLViewer_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find QGLViewer")
  endif()
endif()
