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
message(STATUS "   -e.g. '-DGTAL_WITH_OPENMP:string=true'-, cf documentation)")
message(STATUS "")

option(DGTAL_WITH_OPENMP "With OpenMP (compiler multithread programming) features." OFF)
option(DGTAL_WITH_CGAL "With CGAL." OFF)
option(DGTAL_WITH_ITK "With Insight Toolkit ITK." OFF)
option(DGTAL_WITH_CAIRO "With CairoGraphics." OFF)
option(DGTAL_WITH_HDF5 "With HDF5." OFF)
option(DGTAL_WITH_POLYSCOPE_VIEWER "With polyscope." OFF)
option(DGTAL_WITH_PONCA "With Ponca library for geometry processing." OFF)
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
  message(STATUS "      DGTAL_WITH_OPENMP              true    (OpenMP multithread features)")
else()
  message(STATUS "      DGTAL_WITH_OPENMP              false   (OpenMP multithread features)")
endif()

if (DGTAL_WITH_CGAL)
  set(LIST_OPTION ${LIST_OPTION} [CGAL]\ )
  message(STATUS "      DGTAL_WITH_CGAL                true    (cgal)")
else()
  message(STATUS "      DGTAL_WITH_CGAL                false   (cgal)")
endif()

if(DGTAL_WITH_PONCA)
  set(LIST_OPTION ${LIST_OPTION} [PONCA]\ )
  message(STATUS "      DGTAL_WITH_PONCA               true    (Ponca geometry library)")
else()
  message(STATUS "      DGTAL_WITH_PONCA               false   (Ponca geometry library)")
endif()

if (DGTAL_WITH_ITK)
  set(LIST_OPTION ${LIST_OPTION} [ITK]\ )
  message(STATUS "      DGTAL_WITH_ITK                 true    (Insight Toolkit ITK image wrapper)")
else()
  message(STATUS "      DGTAL_WITH_ITK                 false   (Insight Toolkit ITK image wrapper)")
endif()

if(DGTAL_WITH_CAIRO)
  set(LIST_OPTION ${LIST_OPTION} [CAIRO]\ )
  message(STATUS "      DGTAL_WITH_CAIRO               true    (CairoGraphics drawing features)")
else()
  message(STATUS "      DGTAL_WITH_CAIRO               false   (CairoGraphics drawing features)")
endif()

if (DGTAL_WITH_HDF5)
  set(LIST_OPTION ${LIST_OPTION} [HDF5]\ )
  message(STATUS "      DGTAL_WITH_HDF5                true    (HDF5 image i/o)")
else()
  message(STATUS "      DGTAL_WITH_HDF5                false   (HDF5 image i/o)")
endif()

if(DGTAL_WITH_POLYSCOPE_VIEWER)
  set(LIST_OPTION ${LIST_OPTION} [POLYSCOPE]\ )
  message(STATUS "      DGTAL_WITH_POLYSCOPE_VIEWER    true    (Polyscope based 3D Viewer)")
else()
  message(STATUS "      DGTAL_WITH_POLYSCOPE_VIEWER    false   (Polyscope based 3D Viewer)")
endif()

if (DGTAL_WITH_FFTW3)
  set(LIST_OPTION ${LIST_OPTION} [FFTW3]\ )
  message(STATUS "      DGTAL_WITH_FFTW3               true    (FFTW3 discrete Fourier transform library)")
else (DGTAL_WITH_FFTW3)
  message(STATUS "      DGTAL_WITH_FFTW3               false   (FFTW3 discrete Fourier transform library)")
endif()


if (DGTAL_WITH_LIBIGL)
  set(LIST_OPTION ${LIST_OPTION} [LIBIGL]\ )
  message(STATUS "      DGTAL_WITH_LIBIGL              true    (libIGL)")
else (DGTAL_WITH_LIBIGL)
  message(STATUS "      DGTAL_WITH_LIBIGL              false   (libIGL)")
endif()

message(STATUS "")
message(STATUS "Checking the dependencies: ")

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

    include(eigen)

    # -------------------------------------------------------------------------
    # ITK 5.0 adds "/usr/lib/x86_64-linux-gnu/include" to include path which 
    # does not exists on "new" (for example in Docker containers) systems. 
    # When linking with DGTal, a cmake Error is raised 
    #  "Imported target "DGtal::DGtal" includes non-existent path"
    # 
    # In case the name is not the same accross unix distributions and windows
    # we filter out all directories that do not exist on the system 
    # --------------------------------------------------------------------------
    set(FILTERED_ITK_INCLUDE_DIRS "")
    foreach (includedir ${ITK_INCLUDE_DIRS})
      if(EXISTS ${includedir})
        list(APPEND FILTERED_ITK_INCLUDE_DIRS ${includedir})
      endif()
    endforeach()

    set(DGtalLibDependencies ${DGtalLibDependencies} ${ITK_LIBRARIES})
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_ITK)
    set(DGtalLibIncludeDirs ${DGtalLibIncludeDirs} ${FILTERED_ITK_INCLUDE_DIRS})
    
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
    set(BoardLibDependencies ${BoardLibDependencies} ${CAIRO_LIBRARIES} ${cairo_LIBRARIES})
    set(DGtalLibDependencies ${DGtalLibDependencies} ${CAIRO_LIBRARIES} ${cairo_LIBRARIES})
    set(BoardLibCompileDefs ${BoardLibCompileDefs} -DDGTAL_WITH_CAIRO)
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_CAIRO)
    set(BoardLibIncludeDirs ${BoardLibIncludeDirs} ${CAIRO_INCLUDE_DIRS} ${cairo_INCLUDE_DIRS})
    set(DGtalLibIncludeDirs ${DGtalLibIncludeDirs} ${CAIRO_INCLUDE_DIRS} ${cairo_INCLUDE_DIRS})

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
    set(DGtalLibDependencies ${DGtalLibDependencies} ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES})
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_HDF5)
    set(DGtalLibIncludeDirs ${DGtalLibIncludeDirs} ${HDF5_INCLUDE_DIRS})

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
# Look for Polyscope.
# -----------------------------------------------------------------------------
set(POLYSCOPE_FOUND_DGTAL 0)
if (DGTAL_WITH_POLYSCOPE_VIEWER)
  include(polyscope)

  set(DGtalLibDependencies ${DGtalLibDependencies} polyscope)
  set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_POLYSCOPE_VIEWER)

  set(POLYSCOPE_FOUND_DGTAL 1)
  set(DGTAL_WITH_POLYSCOPE_VIEWER 1)
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
  find_package(CGAL COMPONENTS Core)
  if(CGAL_FOUND)
    set(CGAL_FOUND_DGTAL 1)
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DCGAL_EIGEN3_ENABLED)
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_CGAL)
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DWITH_LAPACK)

    message(STATUS  "CGAL found, version ${CGAL_VERSION}")
    if (CGAL_VERSION VERSION_LESS "5.0")
      message(FATAL_ERROR "CGAL version 5.0 or higher is required.")
    else()
      if (CGAL_VERSION VERSION_LESS "6.0")
        message(STATUS  "CGAL  using ${CGAL_USE_FILE}")
        include( ${CGAL_USE_FILE} )
        set(DGtalLibDependencies ${DGtalLibDependencies} ${CGAL_LIBRARIES} ${CGAL_3D_PARTY-LIBRARIES})
        ## Making sure that CGAL got the Eigen3 flag
      else()
        set(DGtalLibDependencies ${DGtalLibDependencies} CGAL::CGAL)
      endif()
    endif()
  endif()
endif()

# -----------------------------------------------------------------------------
# Look for ponca
# https://poncateam.github.io/ponca/index.html
# (they Are not compulsory).
# -----------------------------------------------------------------------------
set(PONCA_FOUND_DGTAL 0)
if(DGTAL_WITH_PONCA)
  include(ponca)
  
  set(DGtalLibDependencies ${DGtalLibDependencies} Ponca::Ponca)
  set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DCGAL_WITH_PONCA)
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

    set(DGtalLibDependencies ${DGtalLibDependencies} ${FFTW3_LIBRARIES} ${FFTW3_DEP_LIBRARIES})
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_FFTW3)
    set(DGtalLibIncludeDirs ${DGtalLibIncludeDirs} ${FFTW3_INCLUDES} ${FFTW3_INCLUDE_DIRS})

    message(STATUS "FFTW3 is found : ${FFTW3_LIBRARIES}.")
  else()
    message(FATAL_ERROR "FFTW3 is not found.")
  endif()

  if(FFTW3_FLOAT_FOUND)
    set(FFTW3_FLOAT_FOUND_DGTAL 1)
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_FFTW3_FLOAT)
  endif()

  if(FFTW3_DOUBLE_FOUND)
    set(FFTW3_DOUBLE_FOUND_DGTAL 1)
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_FFTW3_DOUBLE)
  endif()

  if(FFTW3_LONG_FOUND)
    set(FFTW3_LONG_FOUND_DGTAL 1)
    set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_FFTW3_LONG)
  endif()

endif()

# -----------------------------------------------------------------------------
# Look for libigl.
# (They are not compulsory).
# -----------------------------------------------------------------------------
if(DGTAL_WITH_LIBIGL)
  if(DGTAL_WITH_CGAL)
    message(STATUS "DGtal/CGAL enabled.")
  else()
    message(FATAL_ERROR "LIBIGL requires CGAL. Please if the `WITH_CGAL=true` cmake flag.")
  endif()
  include(libigl)
  set(DGtalLibCompileDefs ${DGtalLibCompileDefs} -DDGTAL_WITH_LIBIGL)
  set(DGtalLibDependencies ${DGtalLibDependencies} igl_core)
  set(LIBIGL_FOUND_DGTAL 1)
endif()

message(STATUS "-------------------------------------------------------------------------------")

