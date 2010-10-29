#-*-cmake-*-
#
# Test for DGtal libraries.
#
# Once loaded this will define
#  DGtal_FOUND        - system has GraphicsMagick
#  DGtal_INCLUDE_DIR  - include directory for GraphicsMagick
#  DGtal_LIBRARY_DIR  - library directory for GraphicsMagick
#  DGtal_LIBRARIES    - libraries you need to link to
#


SET(DGtal_FOUND   "NO" )

FIND_PATH( DGtal_INCLUDE_DIR SpaceND.h
  "$ENV{DGtal_LOCATION}/"
  "$ENV{DGtal_LOCATION}/include"
  "$ENV{DGtal_HOME}/include/"
  /usr/include/DGtal
  /usr/include/
  /opt/local/include/
  )


FIND_LIBRARY( DGtal DGtal
  PATHS 
  "$ENV{DGtal_LOCATION}/.libs"
  "$ENV{DGtal_LOCATION}/lib"
  "$ENV{DGtal_HOME}/lib"
  /usr/local/lib
  /usr/lob
  /opt/local/lib
  DOC   "DGtal library"
)

SET(DGtal_LIBRARIES ${DGtal} )

IF (DGtal_INCLUDE_DIR)
  IF(DGtal_LIBRARIES)
    SET(DGtal_FOUND "YES")
    GET_FILENAME_COMPONENT(DGtal_LIBRARY_DIR ${DGtal}   PATH)
  ENDIF(DGtal_LIBRARIES)
ENDIF(DGtal_INCLUDE_DIR)


IF(NOT DGtal_FOUND)
  # make FIND_PACKAGE friendly
  IF(NOT DGtal_FIND_QUIETLY)
    IF(DGtal_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR
              "DGtal required, please specify it's location with DGtal_HOME or DGtal_LOCATION")
    ELSE(DGtal_FIND_REQUIRED)
      MESSAGE(STATUS "DGtal was not found.")
    ENDIF(DGtal_FIND_REQUIRED)
  ENDIF(NOT DGtal_FIND_QUIETLY)
ENDIF(NOT DGtal_FOUND)


#####

