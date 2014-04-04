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

OPTION(WITH_C11 "With C++ compiler C11 features." OFF)
OPTION(WITH_OPENMP "With OpenMP (compiler multithread programming) features." OFF)
OPTION(WITH_GMP "With Gnu Multiprecision Library (GMP)." OFF)
OPTION(WITH_EIGEN "With Eigen3 Linear Algebra Library." OFF)
OPTION(WITH_CGAL "With CGAL." OFF)
OPTION(WITH_MAGICK "With GraphicsMagick++." OFF)
OPTION(WITH_ITK "With Insight Toolkit ITK." OFF)
OPTION(WITH_CAIRO "With CairoGraphics." OFF)
OPTION(WITH_HDF5 "With HDF5." OFF)
OPTION(WITH_QGLVIEWER "With LibQGLViewer for 3D visualization (Qt required)." OFF)
OPTION(WITH_BENCHMARK "With Google Benchmark." OFF)



#----------------------------------
# Checking clang version on APPLE
#
# When using clang 5.0, DGtal must
# be compiled with C11 features
#----------------------------------
IF (APPLE)
  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    EXECUTE_PROCESS( COMMAND ${CMAKE_CXX_COMPILER} --version OUTPUT_VARIABLE clang_full_version_string )
    string (REGEX REPLACE ".*LLVM version ([0-9]).*" "\\1" CLANG_VERSION_STRING ${clang_full_version_string})
    if (CLANG_VERSION_STRING VERSION_GREATER 4)
      SET(WITH_C11 ON)
      MESSAGE(STATUS "You are using Clang >= 5.0, I'm forcing the WITH_C11 option")
    endif()
  endif()
endif()
MESSAGE(STATUS " ")
#---------------------------------

IF(WITH_C11)
SET (LIST_OPTION ${LIST_OPTION} [c++11]\ )
message(STATUS "      WITH_C11          true    (C++ compiler C11 features)")
ELSE(WITH_C11)
message(STATUS "      WITH_C11          false   (C++ compiler C11 features)")
ENDIF(WITH_C11)

IF(WITH_OPENMP)
SET (LIST_OPTION ${LIST_OPTION} [OpenMP]\ )
message(STATUS "      WITH_OPENMP       true    (OpenMP multithread features)")
ELSE(WITH_OPENMP)
message(STATUS "      WITH_OPENMP       false   (OpenMP multithread features)")
ENDIF(WITH_OPENMP)

IF(WITH_GMP)
SET (LIST_OPTION ${LIST_OPTION} [GMP]\ )
message(STATUS "      WITH_GMP          true    (Gnu Multiprecision Library)")
ELSE(WITH_GMP)
message(STATUS "      WITH_GMP          false   (Gnu Multiprecision Library)")
ENDIF(WITH_GMP)

IF(WITH_EIGEN)
SET (LIST_OPTION ${LIST_OPTION} [EIGEN]\ )
message(STATUS "      WITH_EIGEN        true    (Eigen3)")
ELSE(WITH_EIGEN)
message(STATUS "      WITH_EIGEN        false   (Eigen3)")
ENDIF(WITH_EIGEN)

IF(WITH_CGAL)
SET (LIST_OPTION ${LIST_OPTION} [CGAL]\ )
message(STATUS "      WITH_CGAL         true    (cgal)")
ELSE(WITH_CGAL)
message(STATUS "      WITH_CGAL         false   (cgal)")
ENDIF(WITH_CGAL)


IF(WITH_ITK)
SET (LIST_OPTION ${LIST_OPTION} [ITK]\ )
message(STATUS "      WITH_ITK          true    (Insight Toolkit ITK image wrapper)")
ELSE(WITH_ITK)
message(STATUS "      WITH_ITK          false   (Insight Toolkit ITK image wrapper)")
ENDIF(WITH_ITK)

IF(WITH_CAIRO)
SET (LIST_OPTION ${LIST_OPTION} [CAIRO]\ )
message(STATUS "      WITH_CAIRO        true    (CairoGraphics drawing features)")
ELSE(WITH_CAIRO)
message(STATUS "      WITH_CAIRO        false   (CairoGraphics drawing features)")
ENDIF(WITH_CAIRO)

IF(WITH_HDF5)
SET (LIST_OPTION ${LIST_OPTION} [HDF5]\ )
message(STATUS "      WITH_HDF5         true    (HDF5 image i/o)")
ELSE(WITH_HDF5)
message(STATUS "      WITH_HDF5         false   (HDF5 image i/o)")
ENDIF(WITH_HDF5)

IF(WITH_MAGICK)
SET (LIST_OPTION ${LIST_OPTION} [MAGICK]\ )
message(STATUS "      WITH_MAGICK       true    (GraphicsMagick based 2D image i/o)")
ELSE(WITH_MAGICK)
message(STATUS "      WITH_MAGICK       false   (GraphicsMagick based 2D image i/o)")
ENDIF(WITH_MAGICK)

If(WITH_QGLVIEWER)
SET (LIST_OPTION ${LIST_OPTION} [QGLVIEWER]\ )
message(STATUS "      WITH_QGLVIEWER    true    (Qt/QGLViewer based 3D Viewer)")
ELSE(WITH_QGLVIEWER)
message(STATUS "      WITH_QGLVIEWER    false   (Qt/QGLViewer based 3D Viewer)")
ENDIF(WITH_QGLVIEWER)
message(STATUS "")
message(STATUS "For Developpers:")
IF(WITH_BENCHMARK)
SET (LIST_OPTION ${LIST_OPTION} [GoogleBenchmark]\ )
message(STATUS "      WITH_BENCHMARK    true    (Google Benchmark)")
ELSE(WITH_HDF5)
message(STATUS "      WITH_BENCHMARK    false   (Google Benchmark)")
ENDIF(WITH_BENCHMARK)
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
  INCLUDE(CheckCPP11)
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
    IF (CPP11_RREF_MOVE)
      SET(C11_RREF_MOVE 1)
      SET(C11_FEATURES "${C11_FEATURES} std::move rvalue-reference(&&)")
    ENDIF()
    MESSAGE(STATUS "Supported c++11 features: [${C11_FEATURES} ]")
    ADD_DEFINITIONS("-DWITH_C11 ")
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
    message(STATUS "GMP and GMPXX found." )
    ADD_DEFINITIONS("-DWITH_GMP ")
    SET(DGtalLibInc ${DGtalLibInc} ${GMP_INCLUDE_DIR})
  ELSE(GMP_FOUND)
    message(FATAL_ERROR "GMP not found. Check the cmake variables associated to this package or disable it." )
  ENDIF(GMP_FOUND)

  try_compile(
    GMP_HAS_IOSTREAM
    ${CMAKE_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/cmake/src/gmp/gmpstream.cpp
    CMAKE_FLAGS
    -DINCLUDE_DIRECTORIES:STRING=${GMP_INCLUDE_DIR}
    -DLINK_LIBRARIES:STRING=${GMPXX_LIBRARIES}\;${GMP_LIBRARIES}
    OUTPUT_VARIABLE OUTPUT
    )

  if ( GMP_HAS_IOSTREAM )
    add_definitions("-DGMP_HAS_IOSTREAM")
    message(STATUS "   * GMPXX has iostream capabilities")
  ELSE(GMP_HAS_IOSTREAM)
    message(STATUS ${OUTPUT})
    message(STATUS "   * GMPXX does not have iostream capabilities")
    message(FATAL_ERROR "GMP has been found but there is a link isuse with some g++ versions. Please check your system or disable the GMP dependency." )
  endif (GMP_HAS_IOSTREAM )
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
# Look for HDF5 (data model and file format for storing and managing data)
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(HDF5_FOUND_DGTAL 0)
IF(WITH_HDF5)
  FIND_PACKAGE (HDF5 REQUIRED HL C)
  IF(HDF5_FOUND)
    INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIRS})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${HDF5_LIBRARIES})
    message(STATUS "HDF5 found")
    SET(HDF5_FOUND_DGTAL 1)
    SET(DGtalLibInc ${DGtalLibInc} ${HDF5_INCLUDE_DIRS})
    ADD_DEFINITIONS("-DWITH_HDF5 ")
  ELSE(HDF5_FOUND)
    message(FATAL_ERROR "HDF5 not found. Check the cmake variables associated to this package or disable it." )
  ENDIF(HDF5_FOUND)
ELSE(WITH_HDF5)
  unset(HDF5_INCLUDE_DIRS)
  unset(HDF5_LIBRARIES)
ENDIF(WITH_HDF5)

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
      message(STATUS  "  (OpenGL-GLU ok) " ${OPENGL_INCLUDE_DIR})
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
IF( WITH_QGLVIEWER)
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
ENDIF( WITH_QGLVIEWER)

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

# -----------------------------------------------------------------------------
# Look for Eigen3
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(EIGEN_FOUND_DGTAL 0)
IF(WITH_EIGEN)
  FIND_PACKAGE(Eigen3 REQUIRED)
  IF(EIGEN3_FOUND)
    SET(EIGEN_FOUND_DGTAL 1)
    ADD_DEFINITIONS("-DWITH_EIGEN ")
    include_directories( ${EIGEN3_INCLUDE_DIR})
    message(STATUS "Eigen3 found.")
  ENDIF(EIGEN3_FOUND)
ENDIF(WITH_EIGEN)

# -----------------------------------------------------------------------------
# Look for CGAL
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(CGAL_FOUND_DGTAL 0)
IF(WITH_CGAL)
  IF (WITH_GMP AND  WITH_EIGEN)
    message(STATUS "GMP and Eigen3 detected for CGAL.")
  ELSE()
    message(FATAL_ERROR "CGAL needs GMP and Eigen3. You must active WITH_GMP and WITH_EIGEN flags and have the associated package installed.")
  ENDIF()

  find_package(CGAL COMPONENTS Core Eigen3 BLAS LAPACK)
  IF(CGAL_FOUND)
    include( ${CGAL_USE_FILE} )
    SET(CGAL_FOUND_DGTAL 1)
    ADD_DEFINITIONS("-DCGAL_EIGEN3_ENABLED   ")
    ADD_DEFINITIONS("-DWITH_CGAL ")
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${CGAL_LIBRARIES} ${CGAL_3D_PARTY-LIBRARIES} )
    ## Making sure that CGAL got the Eigen3 flag
    ADD_DEFINITIONS("-DWITH_Eigen3 -DWITH_LAPACK ")
    message(STATUS "CGAL found.")
  ENDIF(CGAL_FOUND)
ENDIF(WITH_CGAL)


# -----------------------------------------------------------------------------
# Look for Google Benchmark
# (They are not compulsory).
# -----------------------------------------------------------------------------
SET(BENCHMARK_FOUND_DGTAL 0)
IF(WITH_BENCHMARK)

  IF (WITH_C11)
    message(STATUS "C11 enabled for Google benchmark, all fine.")
  ELSE(WITH_C11)
   message(FATAL_ERROR "Google benchmark requires C++11. Please enable it setting 'WITH_C11' to true.")
 ENDIF(WITH_C11)

  FIND_PACKAGE(Benchmark REQUIRED)
  IF(BENCHMARK_FOUND)
    SET(BENCHMARK_FOUND_DGTAL 1)
    ADD_DEFINITIONS("-DWITH_BENCHMARK ")
    include_directories( ${BENCHMARK_INCLUDE_DIR})
    SET(DGtalLibDependencies ${DGtalLibDependencies} ${BENCHMARK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )
    message(STATUS "Google Benchmark found.   ${BENCHMARK_LIBRARIES}")
  ELSE(BENCHMARK_FOUND)
   message(FATAL_ERROR "Google benchmark not installed. Please disable WITH_BENCHMARK or install it.")
 ENDIF(BENCHMARK_FOUND)
ENDIF(WITH_BENCHMARK)



message(STATUS "-------------------------------------------------------------------------------")
