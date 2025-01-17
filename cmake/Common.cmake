#------------------------------------------------------------------------------
# -- Messages
#------------------------------------------------------------------------------
message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "DGtal Version ${PROJECT_VERSION}")
message(STATUS "-------------------------------------------------------------------------------" )
message(STATUS "Source DIR is ${PROJECT_SOURCE_DIR}")
message(STATUS "Binary/Build DIR is ${PROJECT_BINARY_DIR}")
message(STATUS "Build type is ${CMAKE_BUILD_TYPE}")
message(STATUS "Installation prefix directory is " ${CMAKE_INSTALL_PREFIX})
message(STATUS "Host system is " ${CMAKE_HOST_SYSTEM} " with processor " ${CMAKE_HOST_SYSTEM_PROCESSOR})
message(STATUS "Target system is " ${CMAKE_SYSTEM} " with processor " ${CMAKE_SYSTEM_PROCESSOR})

#------------------------------------------------------------------------------
# Hardcode relative paths to CMAKE_INSTALL_PREFIX
#------------------------------------------------------------------------------
set(INSTALL_LIB_DIR_RELATIVE lib CACHE PATH "Installation directory for libraries. Relative to CMAKE_INSTALL_PREFIX.")
set(INSTALL_BIN_DIR_RELATIVE bin CACHE PATH "Installation directory for executables. Relative to CMAKE_INSTALL_PREFIX.")
set(INSTALL_INCLUDE_DIR_RELATIVE include CACHE PATH "Installation directory for headers. Relative to CMAKE_INSTALL_PREFIX.")
set(INSTALL_DATA_DIR_RELATIVE lib/DGtal  CACHE PATH "Installation directory for CMake files. Relative to CMAKE_INSTALL_PREFIX.")
#------------------------------------------------------------------------------
# Set absolute paths in INSTALL_(LIB/BIN/INCLUDE/DATA)_DIR
#------------------------------------------------------------------------------
foreach(p LIB BIN INCLUDE DATA)
  set(INSTALL_${p}_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}_DIR_RELATIVE}")
endforeach()



# -----------------------------------------------------------------------------
# CPP20
# -----------------------------------------------------------------------------
set(DGTAL_CMAKE_CXX_STANDARD_MIN_REQUIRED 20)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${DGTAL_CMAKE_CXX_STANDARD_MIN_REQUIRED})
else()
  # Throw if CMAKE_CXX_STANDARD is 98
  if(${CMAKE_CXX_STANDARD} EQUAL 98)
    message(FATAL_ERROR "CMAKE_CXX_STANDARD is set to ${CMAKE_CXX_STANDARD}, "
      "but DGtal requires at least ${DGTAL_CMAKE_CXX_STANDARD_MIN_REQUIRED}.")
  endif()
endif()
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

message(STATUS "C++ standard set to ${CMAKE_CXX_STANDARD}")

# -----------------------------------------------------------------------------
# Visual Studio : to distinguish between debug and release lib and /bigobj flag
# -----------------------------------------------------------------------------
if (MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
   add_compile_options(/bigobj)
endif()

# -----------------------------------------------------------------------------
# GCC 10.1 Incompatibility
# -----------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 10.1)
  message("CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")
  message(FATAL_ERROR "DGtal cannot be compiled in GCC 10.1 due to a bug in the compiler. "
    "Try llvm, or other version of gcc (solved in gcc 10.2). Issue: https://github.com/DGtal-team/DGtal/issues/1501")
endif()

# -----------------------------------------------------------------------------
# Doxygen targets
# -----------------------------------------------------------------------------
message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "Checking if doxygen/dot is installed:")
message(STATUS " ")
set(INSTALL_DOC_DIR_RELATIVE share/DGtal CACHE PATH "Relative installation directory for DGtal documentation files.")
set(INSTALL_DOC_PATH_CUSTOM "" CACHE PATH "Custom absolute path to the directory to install DGtal documentation files. If empty, CMAKE_INSTALL_PREFIX/INSTALL_DOC_DIR_RELATIVE will be used.")
if("${INSTALL_DOC_PATH_CUSTOM}" STREQUAL "")
  set(INSTALL_DOC_PATH "${CMAKE_INSTALL_PREFIX}/${INSTALL_DOC_DIR_RELATIVE}")
else()
  set(INSTALL_DOC_PATH "${INSTALL_DOC_PATH_ABSOLUTE}")
endif()
include(doxygen)
include(TargetDoxygenDoc OPTIONAL)
include(TargetDoxygenDox OPTIONAL)

# -----------------------------------------------------------------------------
# uninstall target
# -----------------------------------------------------------------------------
option(DGTAL_REMOVE_UNINSTALL "Remove DGtal uninstall target." OFF)
if (NOT DGTAL_REMOVE_UNINSTALL)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/TargetUninstall.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake
  @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/TargetUninstall.cmake")
endif()

# -----------------------------------------------------------------------------
# Parsing cmake options
# -----------------------------------------------------------------------------
if (MSVC)
  option(BUILD_SHARED_LIBS "Build shared libraries." OFF)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries." OFF)
endif()
option(DGTAL_BUILD_TESTS "Build testing." OFF)
option(DEBUG_VERBOSE "Verbose debug messages." OFF)
option(VERBOSE "Verbose messages." OFF)
option(COLOR_WITH_ALPHA_ARITH "Consider alpha channel in color arithmetical operations." OFF)
option(DGTAL_NO_ESCAPED_CHAR_IN_TRACE "Avoid printing special color and font weight terminal escaped char in program output." OFF)
option(DGTAL_CONFIG_HINTS "Provide HINTS to find_dependency in DGtalConfig.cmake. Projects consuming DGtal does not have to provide FOO_DIR to their project, where FOO is a DGtal dependency. Recommended to turn it off when deploying." ON)
mark_as_advanced(DGTAL_CONFIG_HINTS)
option(NO_ADD_STBIMAGE_IMPLEMENT "To avoid duplicated linking errors (like LNK2005 in MSVC)" OFF)
#------------------------------------------------------------------------------
# Some directories and files should also be cleaned when invoking 'make clean'
#------------------------------------------------------------------------------
add_custom_target(distclean
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/TargetDistclean.cmake")

#------------------------------------------------------------------------------
# Configuration of the Config.h
#------------------------------------------------------------------------------
configure_file(${PROJECT_SOURCE_DIR}/src/DGtal/base/Config.h.in
  ${PROJECT_BINARY_DIR}/src/DGtal/base/Config.h)

#------------------------------------------------------------------------------
# Building in the source tree is forbidden
#------------------------------------------------------------------------------
if(PROJECT_BINARY_DIR STREQUAL ${PROJECT_SOURCE_DIR})
  message(STATUS "Building in the source tree is not a good idea ! Remove the file 'CMakeCache.txt' and the folder 'CMakeFiles' an
d build outside the sources (for example 'mkdir build ; cmake <DGTAL_DIR>'.")
endif()
