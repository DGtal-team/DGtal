set(VERBOSE_DGTAL 0)
set(DEBUG_VERBOSE_DGTAL 0)
set(COLOR_WITH_ALPHA_ARITH_DGTAL 0)

if (DEBUG_VERBOSE)
  set(DEBUG_VERBOSE_DGTAL 1)
  target_compile_definitions(DGtal PUBLIC -DDEBUG_VERBOSE)
  message(STATUS "Debug verbose mode activated")
endif()
if (VERBOSE)
  set(VERBOSE_DGTAL 1)
  target_compile_definitions(DGtal PUBLIC -DVERBOSE)
  message(STATUS "Verbose mode activated")
endif()

if(COLOR_WITH_ALPHA_ARITH)
  set(COLOR_WITH_ALPHA_ARITH_DGTAL 1)
  target_compile_definitions(DGtal PUBLIC -DCOLOR_WITH_ALPHA_ARITH)
endif()

# -----------------------------------------------------------------------------
# CPM and CCache specific options
# -----------------------------------------------------------------------------
include(cmake/CPM.cmake)
CPMAddPackage(
  NAME Ccache.cmake
  GITHUB_REPOSITORY TheLartians/Ccache.cmake
  VERSION 1.2
)

# -----------------------------------------------------------------------------
# clang-format in cmake
# -----------------------------------------------------------------------------
CPMAddPackage(
  NAME Format.cmake
  VERSION 1.7.3
  GITHUB_REPOSITORY TheLartians/Format.cmake
  OPTIONS 
      # set to yes skip cmake formatting
      "FORMAT_SKIP_CMAKE YES"
      # path to exclude (optional, supports regular expressions)
      "CMAKE_FORMAT_EXCLUDE cmake/CPM.cmake"
)
# -----------------------------------------------------------------------------
# Debug specific options
# -----------------------------------------------------------------------------
option(WARNING_AS_ERROR "Transform compiler warnings as errors (in Debug build type)." OFF)
if (WARNING_AS_ERROR)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Werror")
  message(STATUS "Warnings as Errors ENABLED.")
endif()
