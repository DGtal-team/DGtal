#
# Copyright 2021 Adobe. All rights reserved.
# This file is licensed to you under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under
# the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS
# OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
#
if(TARGET Boost::headers)
    return()
endif()

message(STATUS "Third-party (external): creating targets 'Boost::<lib>'")

set(OLD_CMAKE_POSITION_INDEPENDENT_CODE ${CMAKE_POSITION_INDEPENDENT_CODE})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Restrict the list of Boost targets to expose in the project (this should be the superset of all
# Boost libraries directly depended on by any subproject).
set(BOOST_INCLUDE_LIBRARIES
    accumulators
    algorithm
    align
    asio
    assign
    atomic
    bimap
    callable_traits
    chrono
    compute
    container
    crc
    date_time
    filesystem
    format
    functional
    graph
    heap
    icl
    interprocess
    iostreams
    lockfree
    log
    math
    numeric/conversion
    numeric/interval
    numeric/ublas
    polygon
    system
    thread
    throw_exception
    timer
    uuid
    vmd
)

if(APPLE)
    if((arm64 IN_LIST CMAKE_OSX_ARCHITECTURES) AND (x86_64 IN_LIST CMAKE_OSX_ARCHITECTURES))
        # Build universal binaries on macOS
        set(BOOST_CONTEXT_ARCHITECTURE "combined" CACHE STRING "Boost.Context architecture")
        set(BOOST_CONTEXT_ABI "sysv" CACHE STRING "Boost.Context ABI")
    elseif(CMAKE_OSX_ARCHITECTURES STREQUAL arm64)
        # Build for arm64 architecture on macOS
        set(BOOST_CONTEXT_ARCHITECTURE "arm64" CACHE STRING "Boost.Context architecture")
        set(BOOST_CONTEXT_ABI "aapcs" CACHE STRING "Boost.Context ABI")
    elseif(CMAKE_OSX_ARCHITECTURES STREQUAL x86_64)
        # Build for x86_64 architecture on macOS
        set(BOOST_CONTEXT_ARCHITECTURE "x86_64" CACHE STRING "Boost.Context architecture")
        set(BOOST_CONTEXT_ABI "sysv" CACHE STRING "Boost.Context ABI")
    endif()
endif()

# Let's limit the external libs pulled by Boost...
option(BOOST_IOSTREAMS_ENABLE_ZLIB "Boost.Iostreams: Enable ZLIB support" OFF)
option(BOOST_IOSTREAMS_ENABLE_BZIP2 "Boost.Iostreams: Enable BZip2 support" OFF)
option(BOOST_IOSTREAMS_ENABLE_LZMA "Boost.Iostreams: Enable LZMA support" OFF)
option(BOOST_IOSTREAMS_ENABLE_ZSTD "Boost.Iostreams: Enable Zstd support" OFF)

if(SKBUILD)
    set(OLD_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS ON)
endif()

# Modern CMake target support was added in Boost 1.82.0
# CMake support for boost::numeric_ublas was added in Boost 1.84.0
include(CPM)
CPMAddPackage(
    NAME Boost
    VERSION 1.84.0
    GITHUB_REPOSITORY "boostorg/boost"
    GIT_TAG "boost-1.84.0"
    EXCLUDE_FROM_ALL ON
)

if(SKBUILD)
    set(BUILD_SHARED_LIBS ${OLD_BUILD_SHARED_LIBS})
endif()

# Due to MKL, we may require the release runtime (/MD) even when compiling in Debug mode.
#
# Boost::random will call <auto_link.hpp> in the following line:
# https://github.com/boostorg/random/blob/3c1f0dbf634ad92fd2a2ffe2a46f5553e5a02de7/src/random_device.cpp#L52
#
# This causes a compilation error with MSVC in Debug mode, at the following line:
# https://github.com/boostorg/config/blob/29c39d45858d40bee86bd3b58ca14499663f08b5/include/boost/config/auto_link.hpp#L122
#
# Since CMake already adds advapi32.lib as part of CMAKE_CXX_STANDARD_LIBRARIES_INIT, we can safely
# spoof the <auto_link.hpp> header with a blank one and remove the problematic error message.
if(TARGET Boost::random)
    set(boost_dummy_autolink_dir "${Boost_BINARY_DIR}/dummy/boost/config/")
    file(WRITE "${boost_dummy_autolink_dir}/auto_link.hpp.in" "")
    configure_file(${boost_dummy_autolink_dir}/auto_link.hpp.in ${boost_dummy_autolink_dir}/auto_link.hpp COPYONLY)
    target_include_directories(boost_random PRIVATE "${Boost_BINARY_DIR}/dummy")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ${OLD_CMAKE_POSITION_INDEPENDENT_CODE})

# Indirect deps pulled by other boost targets
set(Boost_Deps
    assert context core coroutine exception random serialization variant2
)
foreach(name IN ITEMS ${BOOST_INCLUDE_LIBRARIES} ${Boost_Deps})
    if(TARGET boost_${name})
        set_target_properties(boost_${name} PROPERTIES FOLDER third_party/boost)
    endif()
endforeach()

if(NOT TARGET Boost::headers)
    message(FATAL_ERROR "Boost::headers target not found")
endif()

if(NOT TARGET Boost::boost)
    # Forward ALIAS target for Boost::boost
    get_target_property(_aliased Boost::headers ALIASED_TARGET)
    if(_aliased)
        message(STATUS "Creating 'Boost::boost' as a new ALIAS target for '${_aliased}'.")
        add_library(Boost::boost ALIAS ${_aliased})
    else()
        add_library(Boost::boost ALIAS Boost::headers)
    endif()
endif()

# Install boost files when installing library
install(DIRECTORY ${BOOST_INCLUDE_DIRS}/Boost DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DGtal/3rdParties/)
install(TARGETS boost_headers EXPORT boost_headers)
install(EXPORT boost_headers DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/boost NAMESPACE Boost::)

# Export target Boost::headers
export(TARGETS
    boost_headers
    NAMESPACE Boost::
    FILE BoostTargets.cmake
)