cmake_minimum_required(VERSION 3.11)
project(dgtal-dependencies)

# Download/build and optionally install 3rd party dependencies of DGTAL
include(ExternalProject)

set(OUTPUT_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE PATH
  "Base folder where builds and source folder will be installed: i.e. OUTPUT_BUILD_DIR/boost-build")

# This will handle also the case for MultiConfig generators
# Issue to propagate config to ExternalProject_Add: https://gitlab.kitware.com/cmake/cmake/-/issues/17645
if(NOT CMAKE_BUILD_TYPE)
  set(DGTAL_CMAKE_BUILD_TYPE "Release")
else()
  set(DGTAL_CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})
endif()

get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if (isMultiConfig)
  set(_propagate_build_type "--config ${DGTAL_CMAKE_BUILD_TYPE}")
else()
  set(_propagate_build_type "-DCMAKE_BUILD_TYPE=${DGTAL_CMAKE_BUILD_TYPE}")
endif()

option(WITH_BOOST "Build Boost library." ON)
option(WITH_ITK "Build ITK library." OFF)
option(CLEAN_BUILD_ARTIFACTS "Create target clean_artifacts to remove artifacts and external_project folders. Needs to call the target: make/ninja clean_artifacts." ON)
set(NUM_CORES 8 CACHE STRING "Number of cores passed to make: -jNUM_CORES")
option(ZLIB_FORCE_STATIC "Remove the dynamic libraries after zlib install" ON)
mark_as_advanced(ZLIB_FORCE_STATIC)
message(STATUS "OUTPUT_BUILD_DIR: ${OUTPUT_BUILD_DIR}")
message(STATUS "NUM_CORES: ${NUM_CORES}")
message(STATUS "WITH_BOOST: ${WITH_BOOST}")
message(STATUS "WITH_ITK: ${WITH_ITK}")

if(APPLE)
  set(_cmake_os_flags
    -DCMAKE_OSX_DEPLOYMENT_TARGET:STRING=${CMAKE_OSX_DEPLOYMENT_TARGET}
    -DCMAKE_OSX_ARCHITECTURES:STRING=${CMAKE_OSX_ARCHITECTURES}
    )
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # Used to compile zlib in windows with msvc
  set(_compiler_is_msvc ON)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # Used for boost in unix when the compiler is clang
  set(_compiler_is_clang ON)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # Used to disable compilation of DGtal with gcc10.1
  set(_compiler_is_gnu ON)
endif()

#################################################
################### BOOST #######################
#################################################
if(WITH_BOOST)
  set(BOOST_VERSION_MAJOR 1)
  set(BOOST_VERSION_MINOR 72)
  set(BOOST_VERSION_PATCH 0)
  set(BOOST_VERSION "${BOOST_VERSION_MAJOR}_${BOOST_VERSION_MINOR}_${BOOST_VERSION_PATCH}")
  set(BOOST_VERSION_DOTS "${BOOST_VERSION_MAJOR}.${BOOST_VERSION_MINOR}.${BOOST_VERSION_PATCH}")
  message(STATUS "BOOST_VERSION: ${BOOST_VERSION_DOTS}")
  set(BOOST_GIT_REPOSITORY "https://github.com/boostorg/boost")
  set(BOOST_GIT_TAG "boost-${BOOST_VERSION_DOTS}")
  set(BOOST_URL "https://sourceforge.net/projects/boost/files/boost/${BOOST_VERSION_DOTS}/boost_${BOOST_VERSION}.tar.gz/download")
  set(BOOST_URL_HASH SHA256=c66e88d5786f2ca4dbebb14e06b566fb642a1a6947ad8cc9091f9f445134143f)

  # DGTAL_CMAKE_BUILD_TYPE always exists (even in MultiConfig)
  string(TOLOWER ${DGTAL_CMAKE_BUILD_TYPE} _boost_variant)
  if(${_boost_variant} STREQUAL "relwithdebinfo")
    set(_boost_variant "release")
  endif()

  if( UNIX )
    set(_boost_bootstrap_command ./bootstrap.sh)
    set(_boost_b2_command ./b2)
    set(_boost_extra_args
       cxxflags=-fPIC
       )
    if(_compiler_is_clang)
      set(_boost_extra_args
        cxxflags=-fPIC
        toolset=clang
        )
    endif()
  endif()
  if( WIN32 )
    set(_boost_bootstrap_command bootstrap.bat)
    set(_boost_b2_command b2.exe)
    set(_boost_extra_args
        toolset=msvc
        )
  endif()
  if ( APPLE )
    if(${CMAKE_CXX_STANDARD})
      set(_boost_extra_args
        cxxflags=-fPIC
        toolset=clang
        cxxflags=-std=c++${CMAKE_CXX_STANDARD}
        cxxflags=-stdlib=libc++
        cxxflags=-mmacosx-version-min=${CMAKE_OSX_DEPLOYMENT_TARGET}
        linkflags=-stdlib=libc++
        )
    endif()
  endif()

  set(BOOST_BUILD_DIR ${OUTPUT_BUILD_DIR}/boost-build)
  ExternalProject_Add(ep_boost
    URL ${BOOST_URL}
    URL_HASH ${BOOST_URL_MD5}
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ${_boost_bootstrap_command} --prefix=${BOOST_BUILD_DIR}
    BUILD_COMMAND  ${_boost_b2_command} install --prefix=${BOOST_BUILD_DIR}
       --without-python
       --without-log
       --without-coroutine
       --without-type_erasure
       --without-stacktrace
       --threading=multi
       link=static
       variant=${_boost_variant}
       ${_boost_extra_args}
       -j${NUM_CORES}
    INSTALL_COMMAND "" # Already installed in BOOST_BUILD_DIR
  )
endif()

##################################################
#################### zlib  #######################
##################################################

if(_compiler_is_msvc)
  set(ZLIB_GIT_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f) # Version 1.2.11
  message(STATUS "ZLIB_VERSION: ${ZLIB_GIT_TAG} : Version 1.2.11")
  set(ZLIB_BUILD_DIR ${OUTPUT_BUILD_DIR}/zlib-build)
  set(ZLIB_INSTALL_DIR ${OUTPUT_BUILD_DIR}/zlib)
  set(ZLIB_SRC_FOLDER_NAME zlib-src)
  set(ZLIB_SRC_DIR ${OUTPUT_BUILD_DIR}/${ZLIB_SRC_FOLDER_NAME})
  set(ZLIB_GIT_REPOSITORY "https://github.com/madler/zlib")

  ExternalProject_Add(ep_zlib
    GIT_REPOSITORY ${ZLIB_GIT_REPOSITORY}
    GIT_TAG ${ZLIB_GIT_TAG}
    # GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    CMAKE_GENERATOR ${CMAKE_GENERATOR}
    SOURCE_DIR ${ZLIB_SRC_DIR}
    BINARY_DIR ${ZLIB_BUILD_DIR}
    CMAKE_ARGS
        -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
        -DCMAKE_BUILD_TYPE=${DGTAL_CMAKE_BUILD_TYPE}
        -DBUILD_SHARED_LIBS:BOOL=OFF
        -DCMAKE_INSTALL_PREFIX=${ZLIB_INSTALL_DIR}
        ${_propagate_build_type}
        ${_cmake_os_flags}
    )
  if(ZLIB_FORCE_STATIC)
    ExternalProject_Add_Step(
      ep_zlib zlib_remove_dll
      COMMENT "Remove zlib.lib and zlib.dll, leaves only zlibstatic.lib"
      DEPENDEES install
      COMMAND ${CMAKE_COMMAND} -E remove -f ${ZLIB_INSTALL_DIR}/lib/zlib.lib ${ZLIB_INSTALL_DIR}/bin/zlib.dll ${ZLIB_INSTALL_DIR}/lib/zlibd.lib ${ZLIB_INSTALL_DIR}/bin/zlibd.dll
      )
  endif()
endif()

#################################################
################### ITK #######################
#################################################
if(WITH_ITK)
  set(ITK_GIT_TAG v5.1.1) # Release 10 Aug 2020
  message(STATUS "ITK_VERSION: ${ITK_GIT_TAG}")
  set(ITK_BUILD_DIR ${OUTPUT_BUILD_DIR}/ITK-build)
  set(ITK_SRC_FOLDER_NAME ITK-src)
  set(ITK_SRC_DIR ${OUTPUT_BUILD_DIR}/${ITK_SRC_FOLDER_NAME})
  set(ITK_GIT_REPOSITORY "https://github.com/InsightSoftwareConsortium/ITK.git")
  if(WITH_VTK OR WITH_MODERN_VTK)
    set(_ITK_depends ep_vtk)
    set(_ITK_optional_cmake_args
      -DModule_ITKVtkGlue:BOOL=ON
      -DVTK_DIR:STRING=${VTK_BUILD_DIR}
    )
  endif()

  ExternalProject_Add(ep_itk
    GIT_REPOSITORY ${ITK_GIT_REPOSITORY}
    GIT_TAG ${ITK_GIT_TAG}
    # GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    CMAKE_GENERATOR ${CMAKE_GENERATOR}
    SOURCE_DIR ${ITK_SRC_DIR}
    BINARY_DIR ${ITK_BUILD_DIR}
    CMAKE_ARGS
    -DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
    -DBUILD_SHARED_LIBS:BOOL=OFF
    -DBUILD_TESTING:BOOL=OFF
    ${_propagate_build_type}
    ${_ITK_optional_cmake_args}
    ${_cmake_os_flags}
    INSTALL_COMMAND ""
    DEPENDS ${_ITK_depends}
    )
endif()

##################################################
#################### CLEAN #######################
##################################################
if(CLEAN_BUILD_ARTIFACTS)
  # Dummy target
  add_custom_target(clean_artifacts
    )
  add_custom_command(TARGET clean_artifacts
    POST_BUILD
    COMMAND
    ${CMAKE_COMMAND}
    -DOUTPUT_BUILD_DIR=${OUTPUT_BUILD_DIR}
    -DITK_SRC_DIR=${ITK_SRC_DIR}
    -DDGtal_SRC_DIR=${DGtal_SRC_DIR}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/clean_build_artifacts.cmake
    )
endif()
