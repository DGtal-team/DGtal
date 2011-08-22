# -----------------------------------------------------------------------------
# Check Optional Dependencies
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Global options
# -----------------------------------------------------------------------------

OPTION(WITH_GMP "With Gnu Multiprecision Library (GMP)." OFF)
OPTION(WITH_QGLVIEWER "With LibQGLViewer for 3D visualization (Qt required)." OFF)
OPTION(WITH_MAGICK "With GraphicsMagick++." OFF)
OPTION(WITH_ITK "With Insight Toolkit ITK." OFF)
OPTION(WITH_CAIRO "With CairoGraphics." OFF)
OPTION(WITH_COIN3D-SOQT "With COIN3D & SOQT for 3D visualization (Qt required)." OFF)
OPTION(WITH_ALL "With all optional dependencies." OFF)



IF(WITH_ALL)
  SET( WITH_GMP   TRUE )
  SET( WITH_ITK   TRUE )
  SET( WITH_CAIRO  TRUE)
  SET( WITH_COIN3D-SOQT  TRUE)
  SET( WITH_QGLVIEWER  TRUE)
  SET( WITH_MAGICK  TRUE)
ENDIF(WITH_ALL)


# -----------------------------------------------------------------------------
# Look for GMP (The GNU Multiple Precision Arithmetic Library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
IF(WITH_GMP)
  FIND_PACKAGE(GMP)
  IF(GMP_FOUND)
    INCLUDE_DIRECTORIES(${GMP_INCLUDE_DIR})
    SET(GMP_FOUND_DGTAL 1)
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${GMPXX_LIBRARIES} ${GMP_LIBRARIES})
    message(STATUS "(optional) gmp found." )
    ADD_DEFINITIONS("-DWITH_GMP ")
    SET(DGtalLibInc ${DGtalLibInc} ${GMP_INCLUDE_DIR})
  ELSE(GMP_FOUND)
    SET(GMP_FOUND_DGTAL 1)
    message(STATUS "(optional) gmp not found." )
  ENDIF(GMP_FOUND)
ENDIF(WITH_GMP)

# -----------------------------------------------------------------------------
# Look for GraphicsMagic
# (They are not compulsory).
# -----------------------------------------------------------------------------
IF(WITH_MAGICK)
  FIND_PACKAGE(Magick)
  IF(MAGICK++_FOUND)
    INCLUDE_DIRECTORIES(${MAGICK++_INCLUDE_DIR})
    message(STATUS "(optional) GraphicsMagick++ found." )
    ADD_DEFINITIONS("-DWITH_MAGICK ")
    SET(DGtalLibInc ${DGtalLibInc} ${MAGICK++_INCLUDE_DIR})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${MAGICK++_LIBRARIES})
  ELSE(MAGICK++_FOUND)
    message(STATUS "(optional) GraphicsMagick++ not found." )
  ENDIF(MAGICK++_FOUND)
ELSE(WITH_MAGICK)  
  UNSET(MAGICK++_INCLUDE_DIR)
  UNSET(MAGICK++_LIBRARIES)
ENDIF(WITH_MAGICK)  

# -----------------------------------------------------------------------------
# Look for ITK
# (They are not compulsory).
# -----------------------------------------------------------------------------
IF(WITH_ITK)
  FIND_PACKAGE(ITK)
  IF(ITK_FOUND)
    INCLUDE(${ITK_USE_FILE})
    MESSAGE(STATUS "(optional) ITK found ${ITK_USE_FILE}.")
    SET(DGtalLibDependencies ${DGtalLibDependencies} ITKCommon ITKIO)
    ADD_DEFINITIONS(" -DWITH_ITK ")
    SET(DGtalLibInc ${DGtalLibInc} ${ITK_INCLUDE_DIRS})
  ELSE(ITK_FOUND)
    MESSAGE(STATUS "(optional) ITK not found.")
  ENDIF(ITK_FOUND)
ENDIF(WITH_ITK)  

# -----------------------------------------------------------------------------
# Look for Cairo (2D graphics library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
IF(WITH_CAIRO)
  FIND_PACKAGE(Cairo)
  IF(CAIRO_FOUND)
    INCLUDE_DIRECTORIES(${CAIRO_INCLUDE_DIRS})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${CAIRO_LIBRAIRIES})
    message(STATUS "(optional) cairo found")
    set ( WITH_CAIRO TRUE )
    SET(DGtalLibInc ${DGtalLibInc} ${CAIRO_INCLUDE_DIRS})
    ADD_DEFINITIONS("-DWITH_CAIRO ")
  ELSE(CAIRO_FOUND)
    message(STATUS "(optional) cairo not found." )
  ENDIF(CAIRO_FOUND)
ELSE(WITH_CAIRO)
  unset(CAIRO_INCLUDE_DIRS)
  unset(CAIRO_LIBRAIRIES)
ENDIF(WITH_CAIRO)


# -----------------------------------------------------------------------------
# Look for Coin3D, SoQt for 3D display.
# (They are not compulsory).
# -----------------------------------------------------------------------------

IF(WITH_COIN3D-SOQT)
  find_package(COIN3D)
  if ( COIN3D_FOUND )
    set(COIN3D_FOUND_DGTAL 1)
    message(STATUS "(optional) Coin3d found.")
    ADD_DEFINITIONS(-DWITH_COIN3D)
    include_directories( ${COIN3D_INCLUDE_DIR} )
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${COIN3D_LIBRARY})
    SET(DGtalLibInc ${DGtalLibInc} ${COIN3D_INCLUDE_DIR})
  else ( COIN3D_FOUND )
    set(COIN3D_FOUND_DGTAL 0)
    message(STATUS "(optional) Coin3d not found." )
  endif ( COIN3D_FOUND )
  
  find_package(SOQT)
  if ( SOQT_FOUND )
    SET(SOQT_FOUND_DGTAL 1)
    message(STATUS  "(optional) SoQt found. ")
    ADD_DEFINITIONS("-DWITH_SOQT ")
    include_directories( ${SOQT_INCLUDE_DIR} )
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${SOQT_LIBRARY})
    SET(DGtalLibInc ${DGtalLibInc} ${SOQT_INCLUDE_DIR})
  else ( SOQT_FOUND )
    SET(SOQT_FOUND_DGTAL 0)
    message(STATUS  "(optional) SoQt not found." )
  endif ( SOQT_FOUND )
ENDIF(WITH_COIN3D-SOQT)  

if ( COIN3D_FOUND AND SOQT_FOUND )
    SET ( WITH_VISU3D_IV TRUE )
    ADD_DEFINITIONS("-DWITH_VISU3D_IV")
endif( COIN3D_FOUND  AND SOQT_FOUND )




# -----------------------------------------------------------------------------
# Look for QGLViewer for 3D display.
# (They are not compulsory).
# -----------------------------------------------------------------------------

IF(WITH_QGLVIEWER)
  find_package(QGLVIEWER)
  if(QGLVIEWER_FOUND)
    find_package(OpenGL REQUIRED)
    message(STATUS  "(optional) libQGLViewer found.")
    include_directories( ${QGLVIEWER_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
    set ( WITH_VISU3D_QGLVIEWER TRUE )
    set(QGLVIEWER_FOUND_DGTAL 1)
    ADD_DEFINITIONS("-DWITH_VISU3D_QGLVIEWER")
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${QGLVIEWER_LIBRARIES} ${OPENGL_LIBRARIES}  )
  else ( QGLVIEWER_FOUND )
    set(QGLVIEWER_FOUND_DGTAL °)
    message(STATUS  "(optional) libQGLViewer not found (or Qt4 not found)." )
  endif (QGLVIEWER_FOUND)
ENDIF(WITH_QGLVIEWER)


if(NOT WITH_VISU3D_QGLVIEWER)
  if(NOT WITH_VISU3D_IV)
    set( WITH_VISU3D FALSE)
  else (NOT WITH_VISU3D_IV)
    set( WITH_VISU3D TRUE )
  endif(NOT WITH_VISU3D_IV)
else(NOT WITH_VISU3D_QGLVIEWER)
  set( WITH_VISU3D TRUE )
endif(NOT WITH_VISU3D_QGLVIEWER)


# -----------------------------------------------------------------------------
# Look for Qt (if LibqglViewer or coin3D are set).
# -----------------------------------------------------------------------------

IF( WITH_VISU3D_IV OR QGLVIEWER_FOUND)
  find_package(Qt4  COMPONENTS QtCore QtGUI QtXml QtOpenGL REQUIRED)
  if ( QT4_FOUND )
    set(QT4_FOUND_DGTAL 1)
    message(STATUS  "(optional) Qt4 found.")
    set(QT_USE_QTXML 1)
    ADD_DEFINITIONS("-DWITH_QT4 ")
    include( ${QT_USE_FILE})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${QT_LIBRARIES} )
    SET(DGtalLibInc ${DGtalLibInc} ${QT_INCLUDE_DIR})
  else ( QT4_FOUND )
    set(QT4_FOUND_DGTAL 0)
    message(STATUS  "(optional) Qt4 not found." )
  endif ( QT4_FOUND )
ENDIF( WITH_VISU3D_IV OR QGLVIEWER_FOUND)



IF(WITH_ALL) 
message(STATUS "      WITH_ALL          true")
ELSE(WITH_ALL)
message(STATUS "      WITH_ALL          false")
ENDIF(WITH_ALL)
message(STATUS " ")

IF(WITH_GMP) 
SET (LIST_OPTION ${LIST_OPTION} [GMP]\ ) 
message(STATUS "      WITH_GMP          true")
ELSE(WITH_GMP)
message(STATUS "      WITH_GMP          false")
ENDIF(WITH_GMP)

IF(WITH_ITK) 
SET (LIST_OPTION ${LIST_OPTION} [ITK]\ ) 
message(STATUS "      WITH_ITK          true")
ELSE(WITH_ITK)
message(STATUS "      WITH_ITK          false")
ENDIF(WITH_ITK)

IF(WITH_CAIRO) 
SET (LIST_OPTION ${LIST_OPTION} [CAIRO]\ ) 
message(STATUS "      WITH_CAIRO        true")
ELSE(WITH_CAIRO)
message(STATUS "      WITH_CAIRO        false")
ENDIF(WITH_CAIRO)

IF(WITH_COIN3D-SOQT) 
SET (LIST_OPTION ${LIST_OPTION} [COIN3D-SOQT]\ ) 
message(STATUS "      WITH_COIN3D-SOQT  true")
ELSE(WITH_COIN3D-SOQT)
message(STATUS "      WITH_COIN3D-SOQT  false")
ENDIF(WITH_COIN3D-SOQT)

IF(WITH_QGLVIEWER) 
SET (LIST_OPTION ${LIST_OPTION} [QGLVIEWER]\ ) 
message(STATUS "      WITH_QGLVIEWER    true")
ELSE(WITH_QGLVIEWER)
message(STATUS "      WITH_QGLVIEWER    false")
ENDIF(WITH_QGLVIEWER)
 
IF(WITH_MAGICK) 
SET (LIST_OPTION ${LIST_OPTION} [MAGICK]\ ) 
message(STATUS "      WITH_MAGICK       true")
ELSE(WITH_MAGICK)
message(STATUS "      WITH_MAGICK       false")
ENDIF(WITH_MAGICK)
