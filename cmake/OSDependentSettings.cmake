
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
ENDIF(MSVC)


