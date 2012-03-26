#------------------------------------------------------------------------------
# -- Environement variables
#------------------------------------------------------------------------------
if (UNIX)
  add_definitions(-DUNIX)
endif (UNIX)
if (WIN32)
  add_definitions(-DWIN32)
endif (WIN32)
if (APPLE)
  add_definitions(-DAPPLE)
endif (APPLE)

#------------------------------------------------------------------------------
# -- Removing some strange warnings when compiling with VS Express
#------------------------------------------------------------------------------
IF(MSVC)
 IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    SET(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
  ENDIF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
ENDIF(MSVC)

#------------------------------------------------------------------------------
# Remove some MS Visual c++ flags
#------------------------------------------------------------------------------
IF(MSVC)
  ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)
  
#------------------------------------------------------------------------------
# GMP / MPIR (test for static runtime)
#------------------------------------------------------------------------------
#	IF (NOT BUILD_SHARED_LIBS)
#		foreach(flag_var
#		      CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
#		      CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
#		 if(${flag_var} MATCHES "/MD")
#		    string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
#		 endif(${flag_var} MATCHES "/MD")
#		endforeach(flag_var)
#	ENDIF(NOT BUILD_SHARED_LIBS)
ENDIF(MSVC)


