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
    algorithm
    align
    any
    array
    assert
    atomic
    bimap
    bind
    chrono
    concept_check
    config
    container
    container_hash
    conversion
    core
    date_time
    describe
    detail
    dynamic_bitset
    endian
    exception
    foreach
    function
    function_types
    functional
    fusion
    graph
    headers
    icl
    integer
    intrusive
    io
    iostreams 
    iterator
    lambda
    lexical_cast
    math
    move
    mp11
    mpl
    multi_index
    multiprecision
    numeric_conversion
    optional
    parameter
    phoenix
    pool
    predef
    preprocessor   
    property_map
    property_tree
    proto
    random
    range
    ratio
    rational
    regex
    serialization
    smart_ptr
    spirit
    static_assert
    system
    thread
    throw_exception
    tokenizer
    tti
    tuple
    type_index
    type_traits
    typeof
    unordered
    utility
    variant
    variant2
    winapi
    xpressive

# Other boost header if needed
    # accumulators
    # asio
    # assign
    # beast
    # callable_traits
    # charconv
    # circular_buffer
    # compat
    # compatibility
    # compute
    # context
    # contract
    # convert
    # coroutine
    # coroutine2
    # crc
    # dll
    # fiber
    # filesystem
    # flyweight
    # format
    # geometry
    # gil
    # hana
    # heap
    # histogram
    # hof
    # interprocess
    # json
    # lambda2
    # leaf
    # locale
    # local_function
    # lockfree
    # log
    # logic
    # metaparse
    # msm
    # multi_array
    # mysql
    # nowide
    # numeric_interval
    # numeric_ublas
    # outcome
    # pfr
    # poly_collection
    # polygon
    # process
    # program_options
    # ptr_container
    # qvm
    # redis
    # safe_numerics
    # scope
    # scope_exit
    # signals2
    # sort
    # statechart
    # static_string
    # stl_interfaces
    # timer
    # type_erasure
    # uuid
    # vmd
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
include(CPM)

CPMAddPackage(
    NAME Boost
    VERSION 1.84.0
    GITHUB_REPOSITORY "boostorg/boost"
    GIT_TAG "boost-1.84.0"
    SYSTEM TRUE
)

# Manually create a library. For some reason boost::headers seems empty
add_library(boost INTERFACE)

add_library(Boost::boost ALIAS boost)
set(boost_export_list )
foreach (name ${BOOST_INCLUDE_LIBRARIES})
    target_link_libraries(boost INTERFACE Boost::${name})
    list(APPEND boost_export_list boost_${name})
endforeach()

# Install boost files when installing library
install(DIRECTORY ${BOOST_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DGtal/3rdParties/)
install(TARGETS boost ${boost_export_list} EXPORT boost)
install(EXPORT boost DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/boost NAMESPACE Boost::)

# Export target Boost::headers
export(TARGETS
    boost
    ${boost_export_list}
    NAMESPACE Boost::
    FILE BoostTargets.cmake
)

