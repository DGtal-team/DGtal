# Enable floating point exception when DGtal library is loaded. Only works for gcc.
if (UNIX AND NOT APPLE)
  # Only used in Common.cpp
  option(DGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS "Enable feenableexcept when DGtal library is loaded." OFF)
  mark_as_advanced(DGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS)
  if(DGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS)
    target_compile_definitions(DGtal PRIVATE -DDGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS)
  endif()
endif()

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
# Debug specific options
# -----------------------------------------------------------------------------
option(WARNING_AS_ERROR "Transform compiler warnings as errors (in Debug build type)." OFF)
if (WARNING_AS_ERROR)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Werror")
  message(STATUS "Warnings as Errors ENABLED.")
endif()
