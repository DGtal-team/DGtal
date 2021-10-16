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

option(WITH_OPENMP "With OpenMP (compiler multithread programming) features." OFF)
option(WITH_GMP "With Gnu Multiprecision Library (GMP)." OFF)
option(WITH_EIGEN "With Eigen3 Linear Algebra Library." OFF)
option(WITH_CGAL "With CGAL." OFF)
option(WITH_MAGICK "With GraphicsMagick++." OFF)
option(WITH_ITK "With Insight Toolkit ITK." OFF)
option(WITH_CAIRO "With CairoGraphics." OFF)
option(WITH_HDF5 "With HDF5." OFF)
option(WITH_QGLVIEWER "With LibQGLViewer for 3D visualization (Qt5 required, for Qt4 only disable WITH_QT5)." OFF)
option(WITH_PATATE "With Patate library for geometry OFF (Eigen required)." processing)
option(WITH_QT5 "Using Qt5." ON)
option(WITH_BENCHMARK "With Google Benchmark." OFF)
option(WITH_FFTW3 "With FFTW3 discrete Fourier Transform library." OFF)

#----------------------------------
# Removing -frounding-math compile flag for clang
#----------------------------------
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    message( STATUS "Removing -frounding-math flag when compiling with Clang" )
    string( REPLACE "-frounding-math" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
    message( STATUS " " )
endif()
#---------------------------------

if(WITH_OPENMP)
  set(LIST_OPTION ${LIST_OPTION} [OpenMP]\ )
  message(STATUS "      WITH_OPENMP        true    (OpenMP multithread features)")
else()
  message(STATUS "      WITH_OPENMP        false   (OpenMP multithread features)")
endif()

if(WITH_GMP)
  set(LIST_OPTION ${LIST_OPTION} [GMP]\ )
  message(STATUS "      WITH_GMP           true    (Gnu Multiprecision Library)")
else()
  message(STATUS "      WITH_GMP           false   (Gnu Multiprecision Library)")
endif()

if(WITH_EIGEN)
  set(LIST_OPTION ${LIST_OPTION} [EIGEN]\ )
  message(STATUS "      WITH_EIGEN         true    (Eigen3)")
else()
  message(STATUS "      WITH_EIGEN         false   (Eigen3)")
endif()

if(WITH_CGAL)
  set(LIST_OPTION ${LIST_OPTION} [CGAL]\ )
  message(STATUS "      WITH_CGAL          true    (cgal)")
else()
  message(STATUS "      WITH_CGAL          false   (cgal)")
endif()

if(WITH_PATATE)
  set(LIST_OPTION ${LIST_OPTION} [PATATE]\ )
  message(STATUS "      WITH_PATATE        true    (Patate geometry library)")
else()
  message(STATUS "      WITH_PATATE        false   (Patate geometry library)")
endif()


if(WITH_ITK)
  set(LIST_OPTION ${LIST_OPTION} [ITK]\ )
  message(STATUS "      WITH_ITK           true    (Insight Toolkit ITK image wrapper)")
else()
  message(STATUS "      WITH_ITK           false   (Insight Toolkit ITK image wrapper)")
endif()

if(WITH_CAIRO)
  set(LIST_OPTION ${LIST_OPTION} [CAIRO]\ )
  message(STATUS "      WITH_CAIRO         true    (CairoGraphics drawing features)")
else()
  message(STATUS "      WITH_CAIRO         false   (CairoGraphics drawing features)")
endif()

if(WITH_HDF5)
  set(LIST_OPTION ${LIST_OPTION} [HDF5]\ )
  message(STATUS "      WITH_HDF5          true    (HDF5 image i/o)")
else()
  message(STATUS "      WITH_HDF5          false   (HDF5 image i/o)")
endif()

if(WITH_MAGICK)
  set(LIST_OPTION ${LIST_OPTION} [MAGICK]\ )
  message(STATUS "      WITH_MAGICK        true    (GraphicsMagick based 2D image i/o)")
else()
  message(STATUS "      WITH_MAGICK        false   (GraphicsMagick based 2D image i/o)")
endif()

if(WITH_QGLVIEWER)
  set(LIST_OPTION ${LIST_OPTION} [QGLVIEWER]\ )
  message(STATUS "      WITH_QGLVIEWER     true    (QGLViewer based 3D Viewer -- Qt5 by default)")
else()
  message(STATUS "      WITH_QGLVIEWER     false   (QGLViewer based 3D Viewer -- Qt5 by default)")
endif()

if (WITH_QT5)
  set(LIST_OPTION ${LIST_OPTION} [QT5]\ )
  message(STATUS "      WITH_QT5           true    (Using of Qt5 instead of Qt4)")
else (WITH_QT5)
  message(STATUS "      WITH_QT5           false   (Using of Qt5 instead of Qt4)")
endif()

if (WITH_FFTW3)
  set(LIST_OPTION ${LIST_OPTION} [FFTW3]\ )
  message(STATUS "      WITH_FFTW3         true    (FFTW3 discrete Fourier transform library)")
else (WITH_FFTW3)
  message(STATUS "      WITH_FFTW3         false   (FFTW3 discrete Fourier transform library)")
endif()

message(STATUS "")
message(STATUS "For Developpers:")
if(WITH_BENCHMARK)
  set(LIST_OPTION ${LIST_OPTION} [GoogleBenchmark]\ )
  message(STATUS "      WITH_BENCHMARK     true    (Google Benchmark)")
else()
  message(STATUS "      WITH_BENCHMARK     false   (Google Benchmark)")
endif()
message(STATUS "")
message(STATUS "Checking the dependencies: ")



# -----------------------------------------------------------------------------
# Look for GMP (The GNU Multiple Precision Arithmetic Library)
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(GMP_FOUND_DGTAL 0)
if(WITH_GMP)
  find_package(GMP REQUIRED)
  if(GMP_FOUND)
    target_include_directories(DGtal PUBLIC ${GMP_INCLUDE_DIR})
    set(GMP_FOUND_DGTAL 1)
    target_link_libraries(DGtal PUBLIC ${GMPXX_LIBRARIES} ${GMP_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${GMPXX_LIBRARIES} ${GMP_LIBRARIES})
    message(STATUS "GMP and GMPXX found." )
    target_compile_definitions(DGtal PUBLIC -DWITH_GMP)
    target_include_directories(DGtal PUBLIC ${GMP_INCLUDE_DIR})
    set(DGtalLibInc ${DGtalLibInc} ${GMP_INCLUDE_DIR})
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
# Look for GraphicsMagic
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(MAGICK++_FOUND_DGTAL 0)
if(WITH_MAGICK)
  find_package(Magick REQUIRED)
  if(MAGICK++_FOUND)
    set(MAGICK++_FOUND_DGTAL 1)
    target_include_directories(DGtal PUBLIC ${MAGICK++_INCLUDE_DIR})
    message(STATUS "GraphicsMagick++ found." )
    target_compile_definitions(DGtal PUBLIC -DWITH_MAGICK)
    target_include_directories(DGtal PUBLIC ${MAGICK++_INCLUDE_DIR})
    set(DGtalLibInc ${DGtalLibInc} ${MAGICK++_INCLUDE_DIR})
    target_link_libraries(DGtal PUBLIC ${MAGICK++_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${MAGICK++_LIBRARIES})
  else()
    message(FATAL_ERROR "GraphicsMagick++ not found. Check the cmake variables associated to this package or disable it." )
  endif()
else()
  unset(MAGICK++_INCLUDE_DIR)
  unset(MAGICK++_LIBRARIES)
endif()

# -----------------------------------------------------------------------------
# Look for ITK
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(ITK_FOUND_DGTAL 0)
if(WITH_ITK)
  find_package(ITK REQUIRED)
  set (CMAKE_CXX_STANDARD 11)
  if(ITK_FOUND)
    set(ITK_FOUND_DGTAL 1)
    include(${ITK_USE_FILE})
    message(STATUS "ITK found ${ITK_USE_FILE}.")

    target_link_libraries(DGtal PUBLIC ${ITK_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${ITK_LIBRARIES})
    target_compile_definitions(DGtal PUBLIC -DWITH_ITK)
    target_include_directories(DGtal PUBLIC ${ITK_INCLUDE_DIRS})
    set(DGtalLibInc ${DGtalLibInc} ${ITK_INCLUDE_DIRS})

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
if(WITH_CAIRO)
  find_package(Cairo REQUIRED)
  if(CAIRO_FOUND)
    target_compile_definitions(DGtal PUBLIC -DWITH_CAIRO)
    target_include_directories(DGtal PUBLIC ${CAIRO_INCLUDE_DIRS})
    target_link_libraries(DGtal PUBLIC ${CAIRO_LIBRAIRIES})
    set(DGtalLibInc ${DGtalLibInc} ${CAIRO_INCLUDE_DIRS})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${CAIRO_LIBRAIRIES})
    message(STATUS "cairo found")
    set(CAIRO_FOUND_DGTAL 1)
  else()
    message(FATAL_ERROR "cairo not found. Check the cmake variables associated to this package or disable it." )
  endif()
else()
  unset(CAIRO_INCLUDE_DIRS)
  unset(CAIRO_LIBRAIRIES)
endif()

# -----------------------------------------------------------------------------
# Look for HDF5 (data model and file format for storing and managing data)
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(HDF5_FOUND_DGTAL 0)
if(WITH_HDF5)
  find_package (HDF5 REQUIRED HL C)
  if(HDF5_FOUND)
    target_compile_definitions(DGtal PUBLIC -DWITH_HDF5)
    target_include_directories(DGtal PUBLIC ${HDF5_INCLUDE_DIRS})
    target_link_libraries(DGtal PUBLIC ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES})
    set(DGtalLibInc ${DGtalLibInc} ${HDF5_INCLUDE_DIRS})
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
set(QT4_FOUND_DGTAL 0)
set(QT5_FOUND_DGTAL 0)
if (WITH_QGLVIEWER)
  if (WITH_QT5)
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

  else()
    if (APPLE)
      message(STATUS "Warning: on recent MacOs Sierra, Qt4 is no longer supported.")
      message(STATUS "         Please consider switching to Qt5 and define WITH_QT5.")
      message(STATUS "         Otherwise you may have cmake errors while generating the project.")
    endif()

    find_package(Qt4 COMPONENTS QtCore QtGUI QtXml QtOpenGL REQUIRED)

    if (QT4_FOUND)
      set(QT4_FOUND_DGTAL 1)
      message(STATUS  "Qt4 found (needed by QGLVIEWER).")
      set(QT_USE_QTXML 1)
      target_compile_definitions(DGtal PUBLIC -DWITH_QT4)
      include(${QT_USE_FILE})
      target_link_libraries(DGtal PUBLIC ${QT_LIBRARIES})
      target_include_directories(DGtal PUBLIC ${QT_INCLUDE_DIR})
    else ()
      message(FATAL_ERROR "Qt4 not found (needed by QGLVIEWER). Check the cmake variables associated to this package or disable it.")
    endif()
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
if(WITH_OPENMP)
  find_package(OpenMP REQUIRED)
  if(OPENMP_FOUND)
    set(OPENMP_FOUND_DGTAL 1)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    target_link_libraries(DGtal PUBLIC ${OpenMP_CXX_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${OpenMP_CXX_LIBRARIES})
    target_compile_definitions(DGtal PUBLIC -DWITH_OPENMP)
    message(STATUS "OpenMP found. Libs: ${OpenMP_CXX_LIBRARIES}")
  else()
    message(FATAL_ERROR "OpenMP support not available.")
  endif()
endif()

# -----------------------------------------------------------------------------
# Look for Eigen3
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(EIGEN_FOUND_DGTAL 0)
if(WITH_EIGEN)
  set(Eigen3_dgtal_min_version 3.3) # 3.3 is the first version where Eigen3Config.cmake is generated.
  find_package(Eigen3 ${Eigen3_dgtal_min_version} REQUIRED CONFIG)
  set(EIGEN_FOUND_DGTAL 1)
  target_compile_definitions(DGtal PUBLIC -DWITH_EIGEN)
  target_link_libraries(DGtal INTERFACE Eigen3::Eigen)
  message(STATUS "Eigen3 (version ${Eigen3_VERSION}) found.")
endif()

# -----------------------------------------------------------------------------
# Look for CGAL
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(CGAL_FOUND_DGTAL 0)
if(WITH_CGAL)
  if(WITH_GMP AND  WITH_EIGEN)
    message(STATUS "GMP and Eigen3 detected for CGAL.")
  else()
    message(FATAL_ERROR "CGAL needs GMP and Eigen3. You must activate  WITH_GMP and WITH_EIGEN flags and have the associated package installed.")
  endif()

  find_package(CGAL COMPONENTS Core)
  if(CGAL_FOUND)
    include( ${CGAL_USE_FILE} )
    set(CGAL_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DCGAL_EIGEN3_ENABLED)
    target_compile_definitions(DGtal PUBLIC -DWITH_CGAL)
    target_link_libraries(DGtal PUBLIC ${CGAL_LIBRARIES} ${CGAL_3D_PARTY-LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${CGAL_LIBRARIES} ${CGAL_3D_PARTY-LIBRARIES})
    ## Making sure that CGAL got the Eigen3 flag
    target_compile_definitions(DGtal PUBLIC -DWITH_Eigen3 -DWITH_LAPACK)
    message(STATUS "CGAL found.")
  endif()
endif()

# -----------------------------------------------------------------------------
# Look for Patate
# http://patate.gforge.inria.fr/html/index.html
# (they Are not compulsory).
# -----------------------------------------------------------------------------
set(PATATE_FOUND_DGTAL 0)
if(WITH_PATATE)
  if(WITH_EIGEN)
    message(STATUS "Eigen3 detected for PATATE.")
  else()
    message(FATAL_ERROR "PATATE needs  Eigen3. You must activate  WITH_EIGEN flags and have the associated package installed.")
  endif()
  find_package(Patate)
  if(PATATE_FOUND)
    target_include_directories(DGtal PUBLIC ${PATATE_INCLUDE_DIR})
    set(DGtalLibInc ${DGtalLibInc} ${PATATE_INCLUDE_DIR})
    set(PATATE_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_PATATE)
    target_compile_definitions(DGtal PUBLIC -DWITH_Eigen3)
    message(STATUS "PATATE found. ${PATATE_INCLUDE_DIR} ")
 else()
   message(FATAL_ERROR "Patate headers not found.")
 endif()
endif()

# -----------------------------------------------------------------------------
# Look for Google Benchmark
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(BENCHMARK_FOUND_DGTAL 0)
if(WITH_BENCHMARK)
  find_package(Benchmark REQUIRED)
  if(BENCHMARK_FOUND)
    set(BENCHMARK_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_BENCHMARK)
    target_include_directories(DGtal PUBLIC ${BENCHMARK_INCLUDE_DIR})
    target_link_libraries(DGtal PUBLIC ${BENCHMARK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${BENCHMARK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    message(STATUS "Google Benchmark found.   ${BENCHMARK_LIBRARIES}")
  else()
   message(FATAL_ERROR "Google benchmark not installed. Please disable WITH_BENCHMARK or install it.")
 endif()
endif()

# -----------------------------------------------------------------------------
# Look for FFTW3.
# (They are not compulsory).
# -----------------------------------------------------------------------------
set(FFTW3_FOUND_DGTAL 0)
if(WITH_FFTW3)
  find_package(FFTW3 REQUIRED)
  if(FFTW3_FOUND)
    set(FFTW3_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_FFTW3)
    target_include_directories(DGtal PUBLIC ${FFTW3_INCLUDES})
    target_link_libraries(DGtal PUBLIC ${FFTW3_LIBRARIES} ${FFTW3_DEP_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${FFTW3_LIBRARIES} ${FFTW3_DEP_LIBRARIES})
    message(STATUS "FFTW3 is found : ${FFTW3_LIBRARIES}.")
  else()
    message(FATAL_ERROR "FFTW3 is not found.")
  endif()

  if(FFTW3_FLOAT_FOUND)
    set(FFTW3_FLOAT_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_FFTW3_FLOAT)
  endif()

  if(FFTW3_DOUBLE_FOUND)
    set(FFTW3_DOUBLE_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_FFTW3_DOUBLE)
  endif()

  if(FFTW3_LONG_FOUND)
    set(FFTW3_LONG_FOUND_DGTAL 1)
    target_compile_definitions(DGtal PUBLIC -DWITH_FFTW3_LONG)
  endif()

endif()

message(STATUS "-------------------------------------------------------------------------------")
