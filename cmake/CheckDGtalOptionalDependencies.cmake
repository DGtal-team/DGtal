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
message(STATUS "   -e.g. '-DDGTAL_WITH_GMP:string=true'-, cf documentation)")
message(STATUS "")

option(DGTAL_WITH_OPENMP "With OpenMP (compiler multithread programming) features." OFF)
option(DGTAL_WITH_GMP "With Gnu Multiprecision Library (GMP)." OFF)
option(DGTAL_WITH_CGAL "With CGAL." OFF)
option(DGTAL_WITH_ITK "With Insight Toolkit ITK." OFF)
option(DGTAL_WITH_CAIRO "With CairoGraphics." OFF)
option(DGTAL_WITH_HDF5 "With HDF5." OFF)
option(WITH_QGLVIEWER "With LibQGLViewer for 3D visualization (Qt5 required)." OFF)
option(WITH_PATATE "With Patate library for geometry processing." OFF)
option(DGTAL_WITH_FFTW3 "With FFTW3 discrete Fourier Transform library." OFF)
option(DGTAL_WITH_LIBIGL "With libIGL (with copyleft/CGAL included)." OFF)

#----------------------------------
# Removing -frounding-math compile flag for clang
#----------------------------------
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    message( STATUS "Removing -frounding-math flag when compiling with Clang" )
    string( REPLACE "-frounding-math" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
    message( STATUS " " )
endif()
#---------------------------------

if(DGTAL_WITH_OPENMP)
  set(LIST_OPTION ${LIST_OPTION} [OpenMP]\ )
  message(STATUS "      DGTAL_WITH_OPENMP        true    (OpenMP multithread features)")
else()
  message(STATUS "      DGTAL_WITH_OPENMP        false   (OpenMP multithread features)")
endif()

if(DGTAL_WITH_GMP)
  set(LIST_OPTION ${LIST_OPTION} [GMP]\ )
  message(STATUS "      DGTAL_WITH_GMP           true    (Gnu Multiprecision Library)")
else()
  message(STATUS "      DGTAL_WITH_GMP           false   (Gnu Multiprecision Library)")
endif()

if (DGTAL_WITH_CGAL)
  set(LIST_OPTION ${LIST_OPTION} [CGAL]\ )
  message(STATUS "      DGTAL_WITH_CGAL          true    (cgal)")
else()
  message(STATUS "      DGTAL_WITH_CGAL          false   (cgal)")
endif()

if(WITH_PATATE)
  set(LIST_OPTION ${LIST_OPTION} [PATATE]\ )
  message(STATUS "      WITH_PATATE        true    (Patate geometry library)")
else()
  message(STATUS "      WITH_PATATE        false   (Patate geometry library)")
endif()

if (DGTAL_WITH_ITK)
  set(LIST_OPTION ${LIST_OPTION} [ITK]\ )
  message(STATUS "      DGTAL_WITH_ITK           true    (Insight Toolkit ITK image wrapper)")
else()
  message(STATUS "      DGTAL_WITH_ITK           false   (Insight Toolkit ITK image wrapper)")
endif()

if(DGTAL_WITH_CAIRO)
  set(LIST_OPTION ${LIST_OPTION} [CAIRO]\ )
  message(STATUS "      DGTAL_WITH_CAIRO         true    (CairoGraphics drawing features)")
else()
  message(STATUS "      DGTAL_WITH_CAIRO         false   (CairoGraphics drawing features)")
endif()

if (DGTAL_WITH_HDF5)
  set(LIST_OPTION ${LIST_OPTION} [HDF5]\ )
  message(STATUS "      DGTAL_WITH_HDF5          true    (HDF5 image i/o)")
else()
  message(STATUS "      DGTAL_WITH_HDF5          false   (HDF5 image i/o)")
endif()


if(WITH_QGLVIEWER)
  set(LIST_OPTION ${LIST_OPTION} [QGLVIEWER]\ )
  message(STATUS "      WITH_QGLVIEWER     true    (QGLViewer based 3D Viewer -- Qt5 required)")
else()
  message(STATUS "      WITH_QGLVIEWER     false   (QGLViewer based 3D Viewer -- Qt5 required)")
endif()

if (DGTAL_WITH_FFTW3)
  set(LIST_OPTION ${LIST_OPTION} [FFTW3]\ )
  message(STATUS "      DGTAL_WITH_FFTW3         true    (FFTW3 discrete Fourier transform library)")
else (DGTAL_WITH_FFTW3)
  message(STATUS "      DGTAL_WITH_FFTW3         false   (FFTW3 discrete Fourier transform library)")
endif()


if (DGTAL_WITH_LIBIGL)
  set(LIST_OPTION ${LIST_OPTION} [LIBIGL]\ )
  message(STATUS "      DGTAL_WITH_LIBIGL        true    (libIGL)")
else (DGTAL_WITH_LIBIGL)
  message(STATUS "      DGTAL_WITH_LIBIGL        false   (libIGL)")
endif()

message(STATUS "")
message(STATUS "Checking the dependencies: ")



# -----------------------------------------------------------------------------
# Look for GMP (The GNU Multiple Precision Arithmetic Library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(GMP_FOUND_DGTAL 0)
if(DGTAL_WITH_GMP)
  find_package(GMP REQUIRED)
  if(GMP_FOUND)
    target_include_directories(DGtal PUBLIC ${GMP_INCLUDE_DIR})
    set(GMP_FOUND_DGTAL 1)
    target_link_libraries(DGtal PUBLIC ${GMPXX_LIBRARIES} ${GMP_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${GMPXX_LIBRARIES} ${GMP_LIBRARIES})
    message(STATUS "GMP and GMPXX found." )
    target_compile_definitions(DGtal PUBLIC -DDGTAL_WITH_GMP)
    target_include_directories(DGtal PUBLIC ${GMP_INCLUDE_DIR})
  else()
    message(FATAL_ERROR "GMP not found. Check the cmake variables associated to this package or disable it." )
  endif()

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
    target_compile_definitions(DGtal PUBLIC -DGMP_HAS_IOSTREAM)
    message(STATUS "   * GMPXX has iostream capabilities")
  else()
    message(STATUS ${OUTPUT})
    message(STATUS "   * GMPXX does not have iostream capabilities")
    message(FATAL_ERROR "GMP has been found but there is a link isuse with some g++ versions. Please check your system or disable the GMP dependency." )
  endif()
endif()

# -----------------------------------------------------------------------------
# Look for ITK
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(ITK_FOUND_DGTAL 0)
if (DGTAL_WITH_ITK)
  find_package(ITK REQUIRED)
  if(ITK_FOUND)
    set(ITK_FOUND_DGTAL 1)
    include(${ITK_USE_FILE})
    message(STATUS "ITK found ${ITK_USE_FILE}.")

    target_link_libraries(DGtal PUBLIC ${ITK_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${ITK_LIBRARIES})
    target_compile_definitions(DGtal PUBLIC  -DDGTAL_WITH_ITK)
    target_include_directories(DGtal PUBLIC ${ITK_INCLUDE_DIRS})

    # -------------------------------------------------------------------------
    # This test is for instance used for ITK v3.x. ITK forces a limited
    # template depth. We remove this option.
    # --------------------------------------------------------------------------
    if (CMAKE_CXX_FLAGS MATCHES "-ftemplate-depth-[0-9]*")
       message( "Warning: some package has enabled a limited template depth for the C++ compiler." )
       message( "         Disabling option -ftemplate-depth-xx in CMAKE_CXX_FLAGS." )
       set( CMAKE_CXX_FLAGS_TMP ${CMAKE_CXX_FLAGS} )
       string( REGEX REPLACE "-ftemplate-depth-[0-9]*" ""
   CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_TMP}" )
       message ("         CMAKE_CXX_FLAGS=" ${CMAKE_CXX_FLAGS} )
     endif()

  else()
    message(FATAL_ERROR "ITK not found. Check the cmake variables associated to this package or disable it.")
  endif()
endif()

# -----------------------------------------------------------------------------
# Look for Cairo (2D graphics library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(CAIRO_FOUND_DGTAL 0)
if(DGTAL_WITH_CAIRO)
  find_package(Cairo REQUIRED)
  if(CAIRO_FOUND)
    target_compile_definitions(DGtal PUBLIC  -DDGTAL_WITH_CAIRO)
    target_include_directories(DGtal PUBLIC ${CAIRO_INCLUDE_DIRS} ${cairo_INCLUDE_DIRS})
    target_link_libraries(DGtal PUBLIC ${CAIRO_LIBRARIES} ${cairo_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${CAIRO_LIBRARIES} ${cairo_LIBRARIES})
    
    target_compile_definitions(DGTAL_LibBoard PUBLIC  -DDGTAL_WITH_CAIRO)
    target_include_directories(DGTAL_LibBoard PUBLIC ${CAIRO_INCLUDE_DIRS} ${cairo_INCLUDE_DIRS})
    target_link_libraries(DGTAL_LibBoard PUBLIC ${CAIRO_LIBRARIES} ${cairo_LIBRARIES})

    message(STATUS "cairo found")


    set(CAIRO_FOUND_DGTAL 1)
  else()
    message(FATAL_ERROR "cairo not found. Check the cmake variables associated to this package or disable it." )
  endif()
else()
  unset(CAIRO_INCLUDE_DIRS)
  unset(CAIRO_LIBRARIES)
endif()

# -----------------------------------------------------------------------------
# Look for HDF5 (data model and file format for storing and managing data)
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(HDF5_FOUND_DGTAL 0)
if (DGTAL_WITH_HDF5)
  find_package (HDF5 REQUIRED HL C)
  if(HDF5_FOUND)
    target_compile_definitions(DGtal PUBLIC -DDGTAL_WITH_HDF5)
    target_include_directories(DGtal PUBLIC ${HDF5_INCLUDE_DIRS})
    target_link_libraries(DGtal PUBLIC ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES})
    message(STATUS "HDF5 found")
    set(HDF5_FOUND_DGTAL 1)
  else()
    message(FATAL_ERROR "HDF5 not found. Check the cmake variables associated to this package or disable it." )
  endif()
else()
  unset(HDF5_INCLUDE_DIRS)
  unset(HDF5_LIBRARIES)
endif()


# -----------------------------------------------------------------------------
# Look for Qt (needed by libqglviewer visualization).
# -----------------------------------------------------------------------------
set(QT5_FOUND_DGTAL 0)
if (WITH_QGLVIEWER)
    find_package(Qt5 COMPONENTS Widgets OpenGL Xml REQUIRED)
    if (Qt5Widgets_FOUND AND Qt5OpenGL_FOUND AND Qt5Xml_FOUND)
      set(QT5_FOUND_DGTAL 1)
      message(STATUS "Qt5 (Widgets, OpenGL and Xml modules) found (needed by QGLViewer compiled with Qt5).")

      target_compile_definitions(DGtal PUBLIC -DWITH_QT5)
      target_link_libraries(DGtal PUBLIC
        ${Qt5Widgets_LIBRARIES}
        ${Qt5OpenGL_LIBRARIES}
        ${Qt5Xml_LIBRARIES})
      target_include_directories(DGtal PUBLIC
        ${Qt5Widgets_INCLUDES_DIRS}
        ${Qt5OpenGL_INCLUDES_DIR}
        ${Qt5Xml_INCLUDES_DIR})
    else()
      message(STATUS "One of Qt5's modules was not found (needed by QGLViewer).")
    endif()
endif()

# -----------------------------------------------------------------------------
# Look for QGLViewer for 3D display.
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(QGLVIEWER_FOUND_DGTAL 0)
set(WITH_VISU3D 0)
if (WITH_QGLVIEWER)
  find_package(QGLVIEWER REQUIRED)
  if (QGLVIEWER_FOUND)
    find_package(OpenGL REQUIRED)
    message(STATUS  "libQGLViewer found.")
    if (OPENGL_GLU_FOUND)
      message(STATUS  "  (OpenGL-GLU ok) " ${OPENGL_INCLUDE_DIR})
    else (OPENGL_GLU_FOUND)
      message(FATAL_ERROR  "  libQGLViewer found but your system does not have OpenGL/GLU modules." )
    endif()

    target_include_directories(DGtal PUBLIC ${QGLVIEWER_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
    set(WITH_VISU3D_QGLVIEWER 1)
    set(QGLVIEWER_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_VISU3D_QGLVIEWER)
    target_link_libraries(DGtal PUBLIC ${QGLVIEWER_LIBRARIES} ${OPENGL_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${QGLVIEWER_LIBRARIES} ${OPENGL_LIBRARIES}  )
    set(WITH_VISU3D 1)
  else()
    message(FATAL_ERROR  "libQGLViewer not found.  Check the cmake variables associated to this package or disable it." )
  endif()
endif()

# -----------------------------------------------------------------------------
# Look for OpenMP
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(OPENMP_FOUND_DGTAL 0)
if(DGTAL_WITH_OPENMP)
  include(openmp)
  set(DGtalLibDependencies ${DGtalLibDependencies} OpenMP::OpenMP_CXX)
  set(OPENMP_FOUND_DGTAL 1)
  set(DGTAL_WITH_OPENMP 1)
endif()

# -----------------------------------------------------------------------------
# Look for CGAL
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(CGAL_FOUND_DGTAL 0)
if (DGTAL_WITH_CGAL)
  if(DGTAL_WITH_GMP)
    message(STATUS "GMP and Eigen3 detected for CGAL.")
  else()
    message(FATAL_ERROR "CGAL needs GMP. You must activate the DGTAL_WITH_GMP flag and have the associated package installed.")
  endif()

  find_package(CGAL COMPONENTS Core)
  if(CGAL_FOUND)
    set(CGAL_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DCGAL_EIGEN3_ENABLED)
    target_compile_definitions(DGtal PUBLIC -DDGTAL_WITH_CGAL)
    target_compile_definitions(DGtal PUBLIC -DWITH_LAPACK)
    message(STATUS  "CGAL found, version ${CGAL_VERSION}")
    if (CGAL_VERSION VERSION_LESS "5.0")
      message(FATAL_ERROR "CGAL version 5.0 or higher is required.")
    else()
      if (CGAL_VERSION VERSION_LESS "6.0")
        message(STATUS  "CGAL  using ${CGAL_USE_FILE}")
        include( ${CGAL_USE_FILE} )
        target_link_libraries(DGtal PUBLIC ${CGAL_LIBRARIES} ${CGAL_3D_PARTY-LIBRARIES})
        set(DGtalLibDependencies ${DGtalLibDependencies} ${CGAL_LIBRARIES} ${CGAL_3D_PARTY-LIBRARIES})
        ## Making sure that CGAL got the Eigen3 flag
      else()
        target_link_libraries(DGtal PUBLIC CGAL::CGAL)
        set(DGtalLibDependencies ${DGtalLibDependencies} CGAL::CGAL)
      endif()
    endif()
  endif()
endif()

# -----------------------------------------------------------------------------
# Look for Patate
# http://patate.gforge.inria.fr/html/index.html
# (they Are not compulsory).
# -----------------------------------------------------------------------------
set(PATATE_FOUND_DGTAL 0)
if(WITH_PATATE)
  find_package(Patate)
  if(PATATE_FOUND)
    target_include_directories(DGtal PUBLIC ${PATATE_INCLUDE_DIR})
    set(PATATE_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_PATATE)
    message(STATUS "PATATE found. ${PATATE_INCLUDE_DIR} ")
 else()
   message(FATAL_ERROR "Patate headers not found.")
 endif()
endif()

# -----------------------------------------------------------------------------
# Look for FFTW3.
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(FFTW3_FOUND_DGTAL 0)
if(DGTAL_WITH_FFTW3)
  find_package(FFTW3 REQUIRED)
  if(FFTW3_FOUND)
    set(FFTW3_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DDGTAL_WITH_FFTW3)
    target_include_directories(DGtal PUBLIC ${FFTW3_INCLUDES} ${FFTW3_INCLUDE_DIRS})
    target_link_libraries(DGtal PUBLIC ${FFTW3_LIBRARIES} ${FFTW3_DEP_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${FFTW3_LIBRARIES} ${FFTW3_DEP_LIBRARIES})
    message(STATUS "FFTW3 is found : ${FFTW3_LIBRARIES}.")
  else()
    message(FATAL_ERROR "FFTW3 is not found.")
  endif()

  if(FFTW3_FLOAT_FOUND)
    set(FFTW3_FLOAT_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DDGTAL_WITH_FFTW3_FLOAT)
  endif()

  if(FFTW3_DOUBLE_FOUND)
    set(FFTW3_DOUBLE_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DDGTAL_WITH_FFTW3_DOUBLE)
  endif()

  if(FFTW3_LONG_FOUND)
    set(FFTW3_LONG_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DDGTAL_WITH_FFTW3_LONG)
  endif()

endif()

# -----------------------------------------------------------------------------
# Look for libigl.
# (They are not compulsory).
# -----------------------------------------------------------------------------
if(DGTAL_WITH_LIBIGL)
  if  (DGTAL_WITH_CGAL)
    message(STATUS "DGtal/CGAL enabled.")
  else()
    message(FATAL_ERROR "LIBIGL requires CGAL. Please if the `WITH_CGAL=true` cmake flag.")
  endif()
  include(cmake/deps/libigl.cmake)
  target_compile_definitions(DGtal PUBLIC  -DDGTAL_WITH_LIBIGL)
  set(DGtalLibDependencies ${DGtalLibDependencies} igl::core)
  set(LIBIGL_FOUND_DGTAL 1)

endif()

message(STATUS "-------------------------------------------------------------------------------")
