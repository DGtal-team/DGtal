#----------------------------------------------------------------
# Generated CMake target import file for configuration "".
#----------------------------------------------------------------

# Commands may need to know the format version.
SET(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "DGtal" for configuration ""
SET_PROPERTY(TARGET DGtal APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
SET_TARGET_PROPERTIES(DGtal PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "/usr/lib/libboost_program_options-mt.a"
  IMPORTED_LOCATION_NOCONFIG "/usr/local/lib/libDGtal.so"
  IMPORTED_SONAME_NOCONFIG "libDGtal.so"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS DGtal )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_DGtal "/usr/local/lib/libDGtal.so" )

# Import target "DGtalIO" for configuration ""
SET_PROPERTY(TARGET DGtalIO APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
SET_TARGET_PROPERTIES(DGtalIO PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "DGtal;/usr/lib/libboost_program_options-mt.a"
  IMPORTED_LOCATION_NOCONFIG "/usr/local/lib/libDGtalIO.so"
  IMPORTED_SONAME_NOCONFIG "libDGtalIO.so"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS DGtalIO )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_DGtalIO "/usr/local/lib/libDGtalIO.so" )

# Loop over all imported files and verify that they actually exist
FOREACH(target ${_IMPORT_CHECK_TARGETS} )
  FOREACH(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    IF(NOT EXISTS "${file}" )
      MESSAGE(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    ENDIF()
  ENDFOREACH()
  UNSET(_IMPORT_CHECK_FILES_FOR_${target})
ENDFOREACH()
UNSET(_IMPORT_CHECK_TARGETS)

# Commands beyond this point should not need to know the version.
SET(CMAKE_IMPORT_FILE_VERSION)
