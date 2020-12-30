#-*-cmake-*-
#
# Test for GraphicsMagick libraries, unlike CMake's FindGraphicsMagick.cmake which
# tests for GraphicsMagick's binary utilities
#
# Once loaded this will define
#  MAGICK_FOUND        - system has GraphicsMagick
#  MAGICK_INCLUDE_DIR  - include directory for GraphicsMagick
#  MAGICK_LIBRARY_DIR  - library directory for GraphicsMagick
#  MAGICK_LIBRARIES    - libraries you need to link to
#
#  MAGICK++_FOUND        - system has GraphicsMagick
#  MAGICK++_INCLUDE_DIR  - include directory for GraphicsMagick
#  MAGICK++_LIBRARY_DIR  - library directory for GraphicsMagick
#  MAGICK++_LIBRARIES    - libraries you need to link to
#

set(MAGICK_FOUND   0 )
set(MAGICK++_FOUND 0 )

FIND_PATH( MAGICK_INCLUDE_DIR magick.h
  "$ENV{MAGICK_LOCATION}/magick"
  "$ENV{MAGICK_LOCATION}/include/GraphicsMagick"
  "$ENV{MAGICK_LOCATION}/include/magick"
  "$ENV{MAGICK_HOME}/include/magick"
  /usr/include/magick
  /usr/include/
  /usr/include/GraphicsMagick/
  /usr/include/GraphicsMagick/magick/
  /usr/local/include
  /usr/local/include/GraphicsMagick/magick
  /usr/local/include/GraphicsMagick/
  /opt/local/include/GraphicsMagick/magick
  /opt/local/include/GraphicsMagick
  )

FIND_PATH( MAGICK++_INCLUDE_DIR Magick++.h
  "$ENV{MAGICK++_LOCATION}/Magick++"
  "$ENV{MAGICK++_LOCATION}/include/"
  "$ENV{MAGICK_LOCATION}/Magick++"
  "$ENV{MAGICK_LOCATION}/include/Magick++"
  "$ENV{MAGICK_LOCATION}/include/GraphicsMagick"
  "$ENV{MAGICK_LOCATION}/include/"
  "$ENV{MAGICK_HOME}/include/"
  /usr/include/Magick++
  /usr/include/
  /usr/include/GraphicsMagick/
  /usr/local/include
  /usr/local/include/GraphicsMagick
  /opt/local/include/GraphicsMagick/Magick++
  /opt/local/include/GraphicsMagick
  )

if (MSVC)
  # Under Windows, the name of the lib file is CORE_RL_magick
  FIND_LIBRARY( MAGICK_LIBRARIES NAMES Magick GraphicsMagick CORE_RL_magick
    PATHS 
    "$ENV{MAGICK_LOCATION}/magick/.libs"
    "$ENV{MAGICK_LOCATION}/lib"
    "$ENV{MAGICK_HOME}/lib"
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC   "GraphicsMagick magic library"
  )

  # Under Windows, the name of the lib file is CORE_RL_Magick++
  FIND_LIBRARY( MAGICK++_LIBRARIES NAMES Magick++ GraphicsMagick++ CORE_RL_Magick++
    PATHS 
    "$ENV{MAGICK++_LOCATION}/.libs"
    "$ENV{MAGICK_LOCATION}/.libs"
    "$ENV{MAGICK++_LOCATION}/lib"
    "$ENV{MAGICK_LOCATION}/lib"
    "$ENV{MAGICK_HOME}/lib"
    /opt/local/lib
    /usr/lib
    /usr/local/lib
     DOC   "GraphicsMagick Magick++ library"
  )
else()
  FIND_LIBRARY( Magick GraphicsMagick
    PATHS 
    "$ENV{MAGICK_LOCATION}/magick/.libs"
    "$ENV{MAGICK_LOCATION}/lib"
    "$ENV{MAGICK_HOME}/lib"
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    DOC   "GraphicsMagick magic library"
  )


  FIND_LIBRARY( Magick++ GraphicsMagick++
    PATHS 
    "$ENV{MAGICK++_LOCATION}/.libs"
    "$ENV{MAGICK_LOCATION}/.libs"
    "$ENV{MAGICK++_LOCATION}/lib"
    "$ENV{MAGICK_LOCATION}/lib"
    "$ENV{MAGICK_HOME}/lib"
    /opt/local/lib
    /usr/lib
    /usr/local/lib
     DOC   "GraphicsMagick Magick++ library"
  )
endif()


set(MAGICK_LIBRARIES ${Magick} )
set(MAGICK++_LIBRARIES ${Magick++} )


if (MAGICK_INCLUDE_DIR)
  if(MAGICK_LIBRARIES)
    set(MAGICK_FOUND 1)
    GET_FILENAME_COMPONENT(MAGICK_LIBRARY_DIR ${Magick}   PATH)
  endif()
endif()

if (MAGICK++_INCLUDE_DIR)
  if(MAGICK++_LIBRARIES)
    set(MAGICK++_FOUND 1)
    GET_FILENAME_COMPONENT(MAGICK++_LIBRARY_DIR ${Magick++} PATH)
  endif()
endif()


if(NOT MAGICK_FOUND)
  # make FIND_PACKAGE friendly
  if(NOT Magick_FIND_QUIETLY)
    if(Magick_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR
              "GraphicsMagick required, please specify it's location with MAGICK_HOME, MAGICK_LOCATION or MAGICK++_LOCATION")
    else()
      MESSAGE(STATUS "GraphicsMagick was not found.")
    endif()
  endif()
endif()


#####

