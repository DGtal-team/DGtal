set(VERBOSE_DGTAL 0)
set(DGTAL_DEBUG_VERBOSE_DGTAL 0)
set(COLOR_WITH_ALPHA_ARITH_DGTAL 0)

if (DGTAL_DEBUG_VERBOSE)
  set(DGTAL_DEBUG_VERBOSE_DGTAL 1)
  target_compile_definitions(DGtal PUBLIC -DDGTAL_DEBUG_VERBOSE)
  message(STATUS "Debug verbose mode activated")
endif()

if (DGTAL_VERBOSE)
  set(VERBOSE_DGTAL 1)
  target_compile_definitions(DGtal PUBLIC -DVERBOSE)
  message(STATUS "Verbose mode activated")
endif()

if(DGTAL_COLOR_WITH_ALPHA_ARITH)
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
  VERSION 1.2.5
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
option(DGTAL_WARNING_AS_ERROR "Transform compiler warnings as errors (in Debug build type)." OFF)
if (DGTAL_WARNING_AS_ERROR)
target_compile_options(DGtal PRIVATE
  $<$<CONFIG:Debug>:
    -Wall
    -Wno-dangling-reference
    -Wno-sign-compare
    -Werror
    -Wno-unknown-pragmas
    -Wshadow
    -Wunused-variable
    -Wunused-parameter
    -Wunused-function
    -Wno-deprecated-copy
    -Werror=type-limits
    -Wno-nonnull
    -Wno-unused-function
    -Wunused
    -Wno-long-long
    -Wno-system-headers
    -Wno-deprecated
    -Woverloaded-virtual
    -Wwrite-strings
  >
)
  message(STATUS "Warnings as Errors ENABLED.")
endif()
