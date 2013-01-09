# -----------------------------------------------------------------------------
# Check Optional Dependencies
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Global options
# -----------------------------------------------------------------------------
message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "DGtal Library optional configuration:")
message(STATUS "   (to change these values, use ccmake, a graphical")
message(STATUS "   cmake frontend, or define cmake commandline variables")
message(STATUS "   -e.g. '-DWITH_GMP:string=true'-, cf documentation)")
message(STATUS "")

OPTION(WITH_C11 "With C++ compiler C11 (ex. cpp0x) features." OFF)
OPTION(WITH_GMP "With Gnu Multiprecision Library (GMP)." OFF)
OPTION(WITH_QGLVIEWER "With LibQGLViewer for 3D visualization (Qt required)." OFF)
OPTION(WITH_MAGICK "With GraphicsMagick++." OFF)
OPTION(WITH_ITK "With Insight Toolkit ITK." OFF)
OPTION(WITH_CAIRO "With CairoGraphics." OFF)
OPTION(WITH_COIN3D-SOQT "With COIN3D & SOQT for 3D visualization (Qt required)." OFF)
OPTION(WITH_OPENMP "With OpenMP (compiler multithread programming) features." OFF)


IF(WITH_C11)
SET (LIST_OPTION ${LIST_OPTION} [c++11]\ )
message(STATUS "      WITH_C11          true")
ELSE(WITH_C11)
message(STATUS "      WITH_C11          false")
ENDIF(WITH_C11)

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

IF(WITH_OPENMP)
SET (LIST_OPTION ${LIST_OPTION} [OpenMP]\ )
message(STATUS "      WITH_OPENMP       true")
ELSE(WITH_OPENMP)
message(STATUS "      WITH_OPENMP       false")
ENDIF(WITH_OPENMP)

message(STATUS "")
message(STATUS "Checking the dependencies: ")

# -----------------------------------------------------------------------------
# Check CPP11
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(C11_FOUND_DGTAL 0)
SET(C11_AUTO_DGTAL 0)
SET(C11_FORWARD_DGTAL 0)
SET(C11_INITIALIZER_DGTAL 0)
SET(C11_ARRAY 0)
IF(WITH_C11)
  INCLUDE(${CMAKE_MODULE_PATH}/CheckCPP11.cmake)
  IF (CPP11_INITIALIZER_LIST OR CPP11_AUTO OR CP11_FORWARD_LIST)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x ")
    SET(C11_FOUND_DGTAL 1)
    IF (CPP11_AUTO)
      SET(C11_AUTO_DGTAL 1)
      SET(C11_FEATURES "${C11_FEATURES} auto")
    ENDIF()
    IF (CPP11_INITIALIZER_LIST)
      SET(C11_INITIALIZER_DGTAL 1)
      SET(C11_FEATURES "${C11_FEATURES} initializer-list")
    ENDIF()
    IF (CPP11_FORWARD_LIST)
      SET(C11_FORWARD_DGTAL 1)
      SET(C11_FEATURES "${C11_FEATURES} std::forward-list")
    ENDIF()
    IF (CPP11_ARRAY)
      SET(C11_ARRAY 1)
      SET(C11_FEATURES "${C11_FEATURES} std::array")
    ENDIF()
    MESSAGE(STATUS "Supported c++11 features: [${C11_FEATURES} ]")
  ELSE()
    MESSAGE(FATAL_ERROR "Your compiler does not support any c++11 feature. Please specify another C++ compiler of disable this WITH_C11 option.")
  ENDIF()
ENDIF(WITH_C11)

  
# -----------------------------------------------------------------------------
# Look for GMP (The GNU Multiple Precision Arithmetic Library)
# (They are not compulsory).
# ----------------------------------------------------------------------------- 
SET(GMP_FOUND_DGTAL 0)
IF(WITH_GMP)
  FIND_PACKAGE(GMP REQUIRED)
  IF(GMP_FOUND)
    INCLUDE_DIRECTORIES(${GMP_INCLUDE_DIR})
    SET(GMP_FOUND_DGTAL 1)
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${GMPXX_LIBRARIES} ${GMP_LIBRARIES})
    message(STATUS "GMP found." )
    ADD_DEFINITIONS("-DWITH_GMP ")
    SET(DGtalLibInc ${DGtalLibInc} ${GMP_INCLUDE_DIR})
  ELSE(GMP_FOUND)
    message(FATAL_ERROR "GMP not found. Check the cmake variables associated to this package or disable it." )
  ENDIF(GMP_FOUND)
ENDIF(WITH_GMP)

# -----------------------------------------------------------------------------
# Look for GraphicsMagic
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(MAGICK++_FOUND_DGTAL 0)
IF(WITH_MAGICK)
  FIND_PACKAGE(Magick REQUIRED)
  IF(MAGICK++_FOUND)
     SET(MAGICK++_FOUND_DGTAL 1)
     INCLUDE_DIRECTORIES(${MAGICK++_INCLUDE_DIR})
    message(STATUS "GraphicsMagick++ found." )
    ADD_DEFINITIONS("-DWITH_MAGICK ")
    SET(DGtalLibInc ${DGtalLibInc} ${MAGICK++_INCLUDE_DIR})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${MAGICK++_LIBRARIES})
  ELSE(MAGICK++_FOUND)
    message(FATAL_ERROR "GraphicsMagick++ not found. Check the cmake variables associated to this package or disable it." )
  ENDIF(MAGICK++_FOUND)
ELSE(WITH_MAGICK)
  UNSET(MAGICK++_INCLUDE_DIR)
  UNSET(MAGICK++_LIBRARIES)
ENDIF(WITH_MAGICK)

# -----------------------------------------------------------------------------
# Look for ITK
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(ITK_FOUND_DGTAL 0)
IF(WITH_ITK)
  FIND_PACKAGE(ITK REQUIRED)
  IF(ITK_FOUND)
    SET(ITK_FOUND_DGTAL 1)
    INCLUDE(${ITK_USE_FILE})
    MESSAGE(STATUS "ITK found ${ITK_USE_FILE}.")
 
   SET(DGtalLibDependencies ${DGtalLibDependencies} ${ITK_LIBRARIES})
    ADD_DEFINITIONS(" -DWITH_ITK ")
    SET(DGtalLibInc ${DGtalLibInc} ${ITK_INCLUDE_DIRS})


    ## We test if ITK build accepts cpp11 compilers
    IF(WITH_C11)
      try_compile( CPP11_ITK
            ${CMAKE_BINARY_DIR}/CMakeTmp
            ${CMAKE_SOURCE_DIR}/cmake/src/ITKcpp11Bug/
            ITKCPP11BUG
            OUTPUT_VARIABLE OUTPUT )
      if ( CPP11_ITK )
        message(STATUS "ITK accepts [c++11]" )
      else ( CPP11_ITK )
        message(STATUS "ITK does not accept [c++11]" )
      if (CPP11_AUTO OR CPP11_INITIALIZER_LIST)
        MESSAGE(FATAL_ERROR "ITK was found but it appears that the package was not built with std-cpp11 extension and DGtal will notcompile. You can either disable the ITK extension (WITH_ITK)  or the C11 support (WITH_C11 option).")
      endif(CPP11_AUTO OR CPP11_INITIALIZER_LIST)
     endif ( CPP11_ITK )
    ENDIF(WITH_C11)

    # -------------------------------------------------------------------------
    # This test is for instance used for ITK v3.x. ITK forces a limited
    # template depth. We remove this option.
    # --------------------------------------------------------------------------
    if (CMAKE_CXX_FLAGS MATCHES "-ftemplate-depth-[0-9]*")
       message( "Warning: some package has enabled a limited template depth for the C++ compiler." )
       message( "         Disabling option -ftemplate-depth-xx in CMAKE_CXX_FLAGS." )
       set( CMAKE_CXX_FLAGS_TMP ${CMAKE_CXX_FLAGS} )
       STRING( REGEX REPLACE "-ftemplate-depth-[0-9]*" "" 
	 CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_TMP}" )
       message ("         CMAKE_CXX_FLAGS=" ${CMAKE_CXX_FLAGS} )
     endif (CMAKE_CXX_FLAGS MATCHES "-ftemplate-depth-[0-9]*")

  ELSE(ITK_FOUND)
    MESSAGE(FATAL_ERROR "ITK not found. Check the cmake variables associated to this package or disable it.")
  ENDIF(ITK_FOUND)
ENDIF(WITH_ITK)

# -----------------------------------------------------------------------------
# Look for Cairo (2D graphics library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(CAIRO_FOUND_DGTAL 0)
IF(WITH_CAIRO)
  FIND_PACKAGE(Cairo REQUIRED)
  IF(CAIRO_FOUND)
    INCLUDE_DIRECTORIES(${CAIRO_INCLUDE_DIRS})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${CAIRO_LIBRAIRIES})
    message(STATUS "cairo found")
    SET(CAIRO_FOUND_DGTAL 1)
    SET(DGtalLibInc ${DGtalLibInc} ${CAIRO_INCLUDE_DIRS})
    ADD_DEFINITIONS("-DWITH_CAIRO ")
  ELSE(CAIRO_FOUND)
    message(FATAL_ERROR "cairo not found. Check the cmake variables associated to this package or disable it." )
  ENDIF(CAIRO_FOUND)
ELSE(WITH_CAIRO)
  unset(CAIRO_INCLUDE_DIRS)
  unset(CAIRO_LIBRAIRIES)
ENDIF(WITH_CAIRO)


# -----------------------------------------------------------------------------
# Look for Coin3D, SoQt for 3D display.
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(COIN3D_FOUND_DGTAL 0)
SET(SOQT_FOUND_DGTAL 0)
 IF(WITH_COIN3D-SOQT)
  find_package(COIN3D REQUIRED)
  if ( COIN3D_FOUND )
    set(COIN3D_FOUND_DGTAL 1)
    message(STATUS "Coin3d found.")
    ADD_DEFINITIONS(-DWITH_COIN3D)
    include_directories( ${COIN3D_INCLUDE_DIR} )
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${COIN3D_LIBRARY})
    SET(DGtalLibInc ${DGtalLibInc} ${COIN3D_INCLUDE_DIR})
  else ( COIN3D_FOUND )
    message(FATAL_ERROR " Coin3d not found. Check the cmake variables associated to this package or disable it." )
  endif ( COIN3D_FOUND )

  find_package(SOQT REQUIRED)
  if ( SOQT_FOUND )
    SET(SOQT_FOUND_DGTAL 1)
    message(STATUS  "SoQt found. ")
    ADD_DEFINITIONS("-DWITH_SOQT ")
    include_directories( ${SOQT_INCLUDE_DIR} )
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${SOQT_LIBRARY})
    SET(DGtalLibInc ${DGtalLibInc} ${SOQT_INCLUDE_DIR})
  else ( SOQT_FOUND )
    message(FATAL_ERROR  "SoQt not found." Check the cmake variables associated to this package or disable it. )
  endif ( SOQT_FOUND )
ENDIF(WITH_COIN3D-SOQT)

if ( COIN3D_FOUND AND SOQT_FOUND )
  SET ( WITH_VISU3D_IV 1 )
  ADD_DEFINITIONS("-DWITH_VISU3D_IV")
endif( COIN3D_FOUND  AND SOQT_FOUND )

# -----------------------------------------------------------------------------
# Look for QGLViewer for 3D display.
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(QGLVIEWER_FOUND_DGTAL 0)
IF(WITH_QGLVIEWER)
  find_package(QGLVIEWER REQUIRED)
  if(QGLVIEWER_FOUND)

    find_package(OpenGL REQUIRED)
      message(STATUS  "libQGLViewer found.")
    if (OPENGL_GLU_FOUND)
      message(STATUS  "  (OpenGL-GLU ok) "${OPENGL_INCLUDE_DIR})
    else(OPENGL_GLU_FOUND)
      message(FATAL_ERROR  "  libQGLViewer found but your system does not have OpenGL/GLU modules." )
    endif(OPENGL_GLU_FOUND)

    include_directories( ${QGLVIEWER_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
    set(WITH_VISU3D_QGLVIEWER 1)
    set(QGLVIEWER_FOUND_DGTAL 1)
    ADD_DEFINITIONS("-DWITH_VISU3D_QGLVIEWER ")
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${QGLVIEWER_LIBRARIES} ${OPENGL_LIBRARIES}  )
  else ( QGLVIEWER_FOUND )
    message(FATAL_ERROR  "libQGLViewer not found (or Qt4 not found).  Check the cmake variables associated to this package or disable it." )
  endif (QGLVIEWER_FOUND)
ENDIF(WITH_QGLVIEWER)

if(NOT WITH_VISU3D_QGLVIEWER)
  if(NOT WITH_VISU3D_IV)
    set( WITH_VISU3D 0)
  else (NOT WITH_VISU3D_IV)
    set( WITH_VISU3D 1 )
  endif(NOT WITH_VISU3D_IV)
else(NOT WITH_VISU3D_QGLVIEWER)
  set( WITH_VISU3D 1 )
endif(NOT WITH_VISU3D_QGLVIEWER)

# -----------------------------------------------------------------------------
# Look for Qt (if LibqglViewer or coin3D are set).
# -----------------------------------------------------------------------------
set(QT4_FOUND_DGTAL 0)
IF( WITH_COIN3D-SOQT OR WITH_QGLVIEWER)
  find_package(Qt4  COMPONENTS QtCore QtGUI QtXml QtOpenGL REQUIRED)
  if ( QT4_FOUND )
    set(QT4_FOUND_DGTAL 1)
    message(STATUS  "Qt4 found.")
    set(QT_USE_QTXML 1)
    ADD_DEFINITIONS("-DWITH_QT4 ")
    include( ${QT_USE_FILE})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${QT_LIBRARIES} )
    SET(DGtalLibInc ${DGtalLibInc} ${QT_INCLUDE_DIR})
  else ( QT4_FOUND )
    message(FATAL_ERROR  "Qt4 not found.  Check the cmake variables associated to this package or disable it." )
  endif ( QT4_FOUND )
ENDIF( WITH_COIN3D-SOQT OR WITH_QGLVIEWER)

# -----------------------------------------------------------------------------
# Look for OpenMP
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(OPENMP_FOUND_DGTAL 0)
IF(WITH_OPENMP)
  FIND_PACKAGE(OpenMP REQUIRED)
  IF(OPENMP_FOUND)
    SET(OPENMP_FOUND_DGTAL 1)
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_C_FLAGS  "${CMAKE_CXX_FLAGS} ${OpenMP_C_FLAGS}")
    ADD_DEFINITIONS("-DWITH_OPENMP ")
    message(STATUS "OpenMP found.")
  ELSE(OPENMP_FOUND)
    message(FATAL_ERROR "OpenMP support not available.")
  ENDIF(OPENMP_FOUND)
ENDIF(WITH_OPENMP)

message(STATUS "-------------------------------------------------------------------------------")
