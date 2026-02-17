#
# Copyright 2019 Adobe. All rights reserved.
# This file is licensed to you under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under
# the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS
# OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
#

option(EIGEN_WITH_MKL "Use Eigen with MKL" OFF)
option(EIGEN_DONT_VECTORIZE "Disable Eigen vectorization" OFF)
option(DGTAL_WITH_EIGEN_ITK "Use the EIGEN configuration of ITK (effective only if DGTAL_WITH_ITK=ON)" ON)


if (DGTAL_WITH_ITK AND DGTAL_WITH_EIGEN_ITK)
   message(STATUS "Using EIGEN from ITK project, to avoid any eigen conflict version we strongly recommend to compile ITK using cmake option -DITK_USE_SYSTEM_EIGEN:BOOL=on ")
   return()
endif()

if(TARGET Eigen3::Eigen)
    return()
endif()

if(EIGEN_ROOT)
    message(STATUS "Third-party (external): creating target 'Eigen3::Eigen' for external path: ${EIGEN_ROOT}")
    set(EIGEN_INCLUDE_DIRS ${EIGEN_ROOT})
else()
    message(STATUS "Third-party (external): creating target 'Eigen3::Eigen'")

    include(CPM)
    CPMAddPackage(
        NAME eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        DOWNLOAD_ONLY ON
        SYSTEM TRUE
    )
    set(EIGEN_INCLUDE_DIRS ${eigen_SOURCE_DIR})

    install(DIRECTORY ${EIGEN_INCLUDE_DIRS}/Eigen
        DESTINATION ${DGTAL_INSTALL_DEPS_DESTINATION}
    )
endif()

add_library(Eigen3_Eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)

include(GNUInstallDirs)
target_include_directories(Eigen3_Eigen SYSTEM INTERFACE
    $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${DGTAL_INSTALL_DEPS_DESTINATION}/Eigen>
)
target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_MPL2_ONLY)

if(EIGEN_DONT_VECTORIZE)
    target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_DONT_VECTORIZE)
endif()

if(EIGEN_WITH_MKL)
    # TODO: Checks that, on 64bits systems, `mkl::mkl` is using the LP64 interface
    # (by looking at the compile definition of the target)
    include(mkl)
    target_link_libraries(Eigen3_Eigen INTERFACE mkl::mkl)
    target_compile_definitions(Eigen3_Eigen INTERFACE
        EIGEN_USE_MKL_ALL
        EIGEN_USE_LAPACKE_STRICT
    )
endif()

# On Windows, enable natvis files to improve debugging experience
if(WIN32 AND eigen_SOURCE_DIR)
    target_sources(Eigen3_Eigen INTERFACE $<BUILD_INTERFACE:${eigen_SOURCE_DIR}/debug/msvc/eigen.natvis>)
endif()

# Install rules
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME eigen)
set_target_properties(Eigen3_Eigen PROPERTIES EXPORT_NAME Eigen)

install(TARGETS Eigen3_Eigen EXPORT Eigen_Targets)
install(EXPORT Eigen_Targets FILE EigenConfig.cmake DESTINATION ${DGTAL_INSTALL_CMAKE_DESTINATION} NAMESPACE Eigen3::)
export(TARGETS Eigen3_Eigen FILE EigenConfig.cmake NAMESPACE Eigen3::)
