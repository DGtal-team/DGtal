# -----------------------------------------------------------------------------
# Check Dependencies
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Look for boost 
# -----------------------------------------------------------------------------
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)

FIND_PACKAGE(Boost 1.40.0 REQUIRED COMPONENTS program_options)
if ( Boost_FOUND )
  message(STATUS "Boost and boost_program_options found. include=${Boost_INCLUDE_DIR} libs=${Boost_LIBRARIES}" )
  include_directories( ${Boost_INCLUDE_DIR} )
  SET(DGtalLibDependencies ${DGtalLibDependencies} ${Boost_LIBRAIRIES}  ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif( Boost_FOUND )


# -----------------------------------------------------------------------------
# Look for GMP (The GNU Multiple Precision Arithmetic Library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
FIND_PACKAGE(GMP)
IF(GMP_FOUND)
  INCLUDE_DIRECTORIES(${GMP_INCLUDE_DIR})
  SET(DGtalLibDependencies ${DGtalLibDependencies} ${GMPXX_LIBRARIES} ${GMP_LIBRARIES})
  message(STATUS "(optional) gmp found." )
  ADD_DEFINITIONS("-DWITH_GMP ")
ELSE(GMP_FOUND)
   message(STATUS "(optional) gmp not found." )
ENDIF(GMP_FOUND)

# -----------------------------------------------------------------------------
# Look for GraphicsMagic
# (They are not compulsory).
# -----------------------------------------------------------------------------
FIND_PACKAGE(Magick)
IF(MAGICK++_FOUND)
  INCLUDE_DIRECTORIES(${MAGICK++_INCLUDE_DIR})
  message(STATUS "(optional) GraphicsMagick++ found." )
  ADD_DEFINITIONS("-DWITH_MAGICK ")
 SET(DGtalLibDependencies ${DGtalLibDependencies} ${MAGICK++_LIBRARIES})
 ELSE(MAGICK++_FOUND)
   message(STATUS "(optional) GraphicsMagick++ not found." )
ENDIF(MAGICK++_FOUND)

# -----------------------------------------------------------------------------
# Look for ITK
# (They are not compulsory).
# -----------------------------------------------------------------------------
FIND_PACKAGE(ITK)
IF(ITK_FOUND)
  INCLUDE(${ITK_USE_FILE})
  MESSAGE(STATUS "(optional) ITK found.")
  SET(ITK_LIBRARIES ITKCommon ITKIO)
  SET(DGtalLibDependencies ${DGtalLibDependencies} ${ITK_LIBRARIES})
  ADD_DEFINITIONS(" -DWITH_ITK ")
ELSE(ITK_FOUND)
  MESSAGE(STATUS "(optional) ITK not found.")
ENDIF(ITK_FOUND)

# -----------------------------------------------------------------------------
# Look for Cairo (2D graphics library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
IF(NOT WIN32)
  INCLUDE(FindPkgConfig)
  PKG_CHECK_MODULES(CAIRO REQUIRED cairo)
ELSE(NOT WIN32)
  FIND_PACKAGE(Cairo)
ENDIF(NOT WIN32)
IF(CAIRO_FOUND)
  INCLUDE_DIRECTORIES(${CAIRO_INCLUDE_DIRS})
  SET(DGtalLibDependencies ${DGtalLibDependencies} cairo)
  message(STATUS "(optional) cairo found." )
  ADD_DEFINITIONS("-DWITH_CAIRO ")
ELSE(CAIRO_FOUND)
   message(STATUS "(optional) cairo not found." )
ENDIF(CAIRO_FOUND)

# -----------------------------------------------------------------------------
# Look for Coin3D, Qt, SoQt for 3D display.
# (They are not compulsory).
# -----------------------------------------------------------------------------
find_package(COIN3D)
if ( COIN3D_FOUND )
   message(STATUS "(optional) Coin3d found. include=${COIN3D_INCLUDE_DIR} libs=${COIN3D_LIBRARY}." )
   ADD_DEFINITIONS(-DWITH_COIN3D)
   include_directories( ${COIN3D_INCLUDE_DIR} )
   SET(DGtalLibDependencies ${DGtalLibDependencies} ${COIN3D_LIBRARY})
else ( COIN3D_FOUND )
   message(STATUS "(optional) Coin3d not found." )
endif ( COIN3D_FOUND )

find_package(Qt4  COMPONENTS QtCore QtGUI QtXml QtOpenGL)
if ( QT4_FOUND )
   message(STATUS  "(optional) Qt4 found. include=${QT_INCLUDE_DIR} libs=${QT_LIBRARIES}." )
   set(QT_USE_QTXML 1)
   ADD_DEFINITIONS("-DWITH_QT4 ")
   include( ${QT_USE_FILE})
   SET(DGtalLibDependencies ${DGtalLibDependencies} ${QT_LIBRARIES} )
else ( QT4_FOUND )
   message(STATUS  "(optional) Qt4 not found." )
endif ( QT4_FOUND )

find_package(SOQT)
if ( SOQT_FOUND )
   message(STATUS  "(optional) SoQt found. include=${SOQT_INCLUDE_DIR} libs=${SOQT_LIBRARY}." )
   ADD_DEFINITIONS("-DWITH_SOQT ")
   include_directories( ${SOQT_INCLUDE_DIR} )
   SET(DGtalLibDependencies ${DGtalLibDependencies} ${SOQT_LIBRARY})
else ( SOQT_FOUND )
  message(STATUS  "(optional) SoQt not found." )
endif ( SOQT_FOUND )




if ( COIN3D_FOUND AND QT4_FOUND AND SOQT_FOUND )
    SET ( WITH_VISU3D_IV TRUE )
    ADD_DEFINITIONS("-DWITH_VISU3D_IV")
endif( COIN3D_FOUND AND QT4_FOUND AND SOQT_FOUND )



find_package(QGLVIEWER)
if(QGLVIEWER_FOUND AND QT4_FOUND AND QT_QTOPENGL_FOUND)
  find_package(OpenGL REQUIRED)
  message(STATUS  "(optional) libQGLViewer found. include=${QGLVIEWER_INCLUDE_DIR} Libs=${QGLVIEWER_LIBRARIES}." )
  include_directories( ${QGLVIEWER_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
  set ( WITH_VISU3D_QGLVIEWER TRUE )
  ADD_DEFINITIONS("-DWITH_VISU3D_QGLVIEWER")
  SET(DGtalLibDependencies ${DGtalLibDependencies} ${QGLVIEWER_LIBRARIES} ${OPENGL_LIBRARIES}  )
else ( QGLVIEWER_FOUND  AND QT4_FOUND AND QT_QTOPENGL_FOUND)
  message(STATUS  "(optional) libQGLViewer not found (or Qt4 not found)." )
endif ( QGLVIEWER_FOUND  AND QT4_FOUND AND QT_QTOPENGL_FOUND)

if(NOT WITH_VISU3D_QGLVIEWER)
  if(NOT WITH_VISU3D_IV)
    message(STATUS  "(optional) No 3D visualisation possible  (QGLViewer and IV )." )
    set( WITH_VISU3D FALSE)
  else (NOT WITH_VISU3D_IV)
    set( WITH_VISU3D TRUE )
  endif(NOT WITH_VISU3D_IV)
else(NOT WITH_VISU3D_QGLVIEWER)
  set( WITH_VISU3D TRUE )
endif(NOT WITH_VISU3D_QGLVIEWER)


