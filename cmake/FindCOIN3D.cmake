# Try to find Coin3D
# Once done this will define
#
# COIN3D_FOUND        - system has Coin3D - Open Inventor
# COIN3D_INCLUDE_DIR  - where the Inventor include directory can be found
# COIN3D_LIBRARY      - Link this to use Coin3D
#
 

if (WIN32)
  if (CYGWIN)

    FIND_PATH(COIN3D_INCLUDE_DIR Inventor/So.h)

    FIND_LIBRARY(COIN3D_LIBRARY Coin)

  else()

    FIND_PATH(COIN3D_INCLUDE_DIR Inventor/So.h
      "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\Coin3D\\2;Installation Path]/include"
    )

    FIND_LIBRARY(COIN3D_LIBRARY_DEBUG coin2d
      "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\Coin3D\\2;Installation Path]/lib"
    )

    FIND_LIBRARY(COIN3D_LIBRARY_RELEASE coin2
      "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\Coin3D\\2;Installation Path]/lib"
    )

    if (COIN3D_LIBRARY_DEBUG AND COIN3D_LIBRARY_RELEASE)
      set(COIN3D_LIBRARY optimized ${COIN3D_LIBRARY_RELEASE}
                         debug ${COIN3D_LIBRARY_DEBUG})
    else()
      if (COIN3D_LIBRARY_DEBUG)
        set (COIN3D_LIBRARY ${COIN3D_LIBRARY_DEBUG})
      endif()
      if (COIN3D_LIBRARY_RELEASE)
        set (COIN3D_LIBRARY ${COIN3D_LIBRARY_RELEASE})
      endif()
    endif()

    if (COIN3D_LIBRARY)
      ADD_DEFINITIONS ( -DCOIN_NOT_DLL )
    endif()

  endif()

else()
  if(APPLE)
    FIND_PATH(COIN3D_INCLUDE_DIR Inventor/So.h
     /Library/Frameworks/Inventor.framework/Headers 
    )
    FIND_LIBRARY(COIN3D_LIBRARY Coin
      /Library/Frameworks/Inventor.framework/Libraries
    )   
    set(COIN3D_LIBRARY "-framework Coin3d" CACHE STRING "Coin3D library for OSX")
   else()

  FIND_PATH(COIN3D_INCLUDE_DIR Inventor/So.h
	  /usr/include/
	  /usr/local/include/
	  /home/lachaud/local/include/
	 )

  FIND_LIBRARY(COIN3D_LIBRARY Coin
	  /usr/lib/
	  /usr/local/lib/
	  /home/lachaud/local/lib/
    )   
  endif()

endif()

# handle the QUIETLY and REQUIRED arguments and set COIN3D_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Coin3D DEFAULT_MSG COIN3D_LIBRARY COIN3D_INCLUDE_DIR)

MARK_AS_ADVANCED(COIN3D_INCLUDE_DIR COIN3D_LIBRARY )


