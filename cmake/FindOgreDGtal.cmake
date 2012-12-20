###################################
# Specific Ogre discovery in DGtal.
###################################

if(UNIX AND NOT APPLE)

  if(NOT DEFINED OGRE_PLUGINS)
    if(EXISTS "/usr/local/lib/OGRE/") #Ogre built from source (default install path)
      set(OGRE_PLUGINS "/usr/local/lib/OGRE/")
    elseif(EXISTS "/usr/lib/OGRE/") #Old ubuntu install
      set(OGRE_PLUGINS "/usr/lib/OGRE/")
    elseif (EXISTS "/usr/lib/i386-linux-gnu/OGRE/") #Ubuntu12.04-32bits-default
      set(OGRE_PLUGINS "/usr/lib/i386-linux-gnu/OGRE/")
    elseif (EXISTS "/usr/lib/x86_64-linux-gnu/OGRE-1.7.4/") #Ubuntu12.04-64bits-default
      set(OGRE_PLUGINS  "/usr/lib/x86_64-linux-gnu/OGRE-1.7.4/")
    else()
      message(SEND_ERROR "Unable to find Ogre plugins, please set OGRE_PLUGINS variable")
    endif(EXISTS "/usr/local/lib/OGRE/")
  endif()

  if(EXISTS "/usr/local/lib/OGRE/cmake")
    set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
  elseif(EXISTS "/usr/share/OGRE/cmake")
    set(CMAKE_MODULE_PATH "/usr/share/OGRE/cmake/modules;${CMAKE_MODULE_PATH}")
  else ()
    message(SEND_ERROR "Failed to find module path, please set OGRE_DIR variable to the folder containing FindOgre.cmake")
  endif(EXISTS "/usr/local/lib/OGRE/cmake")
endif(UNIX AND NOT APPLE)

if(APPLE)
  
  FIND_LIBRARY(CARBON_LIBRARY Carbon)
  FIND_LIBRARY(COCOA_LIBRARY Cocoa)
  FIND_LIBRARY(IOKIT_LIBRARY IOKit )
  FIND_LIBRARY(FOUNDATION_LIBRARY Foundation )
  MARK_AS_ADVANCED (CARBON_LIBRARY COCOA_LIBRARY)
  SET(EXTRA_LIBS ${CARBON_LIBRARY} ${COCOA_LIBRARY}  ${IOKIT_LIBRARY} ${FOUNDATION_LIBRARY})

  set( OGRE_HOME $ENV{OGRE_HOME})
  if(NOT DEFINED OGRE_HOME)
    message("The variable OGRE_HOME is unset, please set it, this cmake lists has very low chances to compile")
    set(CMAKE_MODULE_PATH "${OGRE_DIR}/cmake/;${CMAKE_MODULE_PATH}")
    set(OGRE_PLUGINS "${OGRE_DIR}/lib/")
  else()
    set(OGRE_HOME "$ENV{OGRE_HOME}")
    set(OGRE_DIR OGRE_HOME)
    set(CMAKE_MODULE_PATH "${OGRE_HOME}/cmake/;${CMAKE_MODULE_PATH}")
    set(OGRE_PLUGINS "${OGRE_HOME}/lib/")
  endif()
  ADD_DEFINITIONS("-x objective-c++")
  set(OGRE_DIR "${OGRE_HOME}" )
endif(APPLE)

find_package(OGRE REQUIRED)
if(OGRE_FOUND)
  find_package(OIS REQUIRED)
  message(STATUS "OGRE_PLUGINS: "${OGRE_PLUGINS})
  message(STATUS  "libOgre found. ")
else ( OGRE_FOUND )
  message(FATAL_ERROR  "Ogre not found.  Check the cmake variables associated to this package or disable it." )
endif ( OGRE_FOUND )
