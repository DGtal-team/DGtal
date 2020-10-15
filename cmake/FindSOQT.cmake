# Try to find SoQt
# Once done this will define
#
# SOQT_FOUND        - system has SOQT - needs Coin3D - Open Inventor
# SOQT_INCLUDE_DIR  - where the SoQt include directory can be found
# SOQT_LIBRARY      - Link this to use SoQt
#
 

if (WIN32)
  if (CYGWIN)

    FIND_PATH(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h)

    FIND_LIBRARY(SOQT_LIBRARY SoQt)

  else()
    message("[xx] Unchecked system." )
    FIND_PATH(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h
      "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\SoQt\\2;Installation Path]/include"
    )

    FIND_LIBRARY(SOQT_LIBRARY_DEBUG soqtd
      "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\SoQt\\2;Installation Path]/lib"
    )

    FIND_LIBRARY(SOQT_LIBRARY_RELEASE soqt
      "[HKEY_LOCAL_MACHINE\\SOFTWARE\\SIM\\SoQt\\2;Installation Path]/lib"
    )

    if (SOQT_LIBRARY_DEBUG AND SOQT_LIBRARY_RELEASE)
      set(SOQT_LIBRARY optimized ${SOQT_LIBRARY_RELEASE}
                         debug ${SOQT_LIBRARY_DEBUG})
    else()
      if (SOQT_LIBRARY_DEBUG)
        set (SOQT_LIBRARY ${SOQT_LIBRARY_DEBUG})
      endif()
      if (SOQT_LIBRARY_RELEASE)
        set (SOQT_LIBRARY ${SOQT_LIBRARY_RELEASE})
      endif()
    endif()

    if (SOQT_LIBRARY)
      ADD_DEFINITIONS ( -DSOQT_NOT_DLL )
    endif()

  endif()

else()
  if(APPLE)
    FIND_PATH(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h
     /Library/Frameworks/Inventor.framework/Headers 
    )
    FIND_LIBRARY(SOQT_LIBRARY SoQt
      /Library/Frameworks/Inventor.framework/Libraries
    )   
    set(SOQT_LIBRARY "-framework SoQt" CACHE STRING "SoQt library for OSX")
   else()
       FIND_PATH(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h
	/home/lachaud/local/include)
       FIND_LIBRARY(SOQT_LIBRARY SoQt
	/home/lachaud/local/lib)   
  endif()

endif()

# handle the QUIETLY and REQUIRED arguments and set SOQT_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SoQt DEFAULT_MSG SOQT_LIBRARY SOQT_INCLUDE_DIR)

MARK_AS_ADVANCED(SOQT_INCLUDE_DIR SOQT_LIBRARY )


