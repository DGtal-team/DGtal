#------------------------------------------------------------------------------
# -- Environement variables
#------------------------------------------------------------------------------
if (UNIX)
  target_compile_definitions(DGtal PUBLIC -DUNIX)
endif()
if (WIN32)
  target_compile_definitions(DGtal PUBLIC -DWIN32)
endif()
if (APPLE)
  target_compile_definitions(DGtal PUBLIC -DAPPLE)
endif()

#------------------------------------------------------------------------------
# -- Removing some strange warnings when compiling with VS Express
#------------------------------------------------------------------------------
if(MSVC)
 if(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    set(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
  endif()
endif()

# Misc
if( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
# Add -fPIC
set_target_properties(DGtal PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

#------------------------------------------------------------------------------
# Remove some MS Visual c++ flags
#------------------------------------------------------------------------------
if(MSVC)
  target_compile_definitions(DGtal PRIVATE -D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)
  #------------------------------------------------------------------------------
  # for GMP / MPIR (MT)
  #------------------------------------------------------------------------------
  target_link_options(DGtal PRIVATE "/NODEFAULTLIB:\"libcmtd.lib;libcmt.lib\"")
endif()


#------------------------------------------------------------------------------
# Specific compiler options
#------------------------------------------------------------------------------
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_compile_options(DGtal PRIVATE -Qunused-arguments)
  message(STATUS "Clang compiler detected")
  if ( ${CMAKE_BUILD_TYPE} MATCHES "Debug" )
    target_compile_options(DGtal PRIVATE -Wdocumentation)
  endif()
endif()


# Enable floating point exception when DGtal library is loaded. Only works for gcc.
if (UNIX AND NOT APPLE)
  # Only used in Common.cpp
  option(DGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS "Enable feenableexcept when DGtal library is loaded." OFF)
  mark_as_advanced(DGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS)
  if(DGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS)
    target_compile_definitions(DGtal PRIVATE -DDGTAL_ENABLE_FLOATING_POINT_EXCEPTIONS)
  endif()
endif()
