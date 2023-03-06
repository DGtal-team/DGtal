
#------------------------------------------------------------------------------
# CPack Configurations
#------------------------------------------------------------------------------
set(CPACK_PACKAGE_VERSION_MAJOR 	 ${DGtal_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR 	 ${DGtal_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH 	 ${DGtal_VERSION_PATCH})
# for other generator set specific backend information
INCLUDE(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "DGtal Project")
set(CPACK_PACKAGE_VENDOR ".")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_CONTACT "contacts@dgtal.org" )
set(CPACK_NSIS_CONTACT "contacts@dgtal.org")

if(WIN32 AND NOT UNIX)
  set(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} DGtal")
  set(CPACK_NSIS_HELP_LINK "http:\\\\\\\\dgtal.org")
  set(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\dgtal.org")
  set(CPACK_NSIS_MODifY_PATH ON)
else()
  ##set(CPACK_STRIP_FILES "bin/main") ??
  set(CPACK_SOURCE_STRIP_FILES "")
endif()

set(CPACK_SOURCE_IGNORE_FILES CVS;[.]svn;[.]git;[.]AppleDouble;[.]\#.*;\#.*;.*[.]kdev4;.*~;build.*;doc/html/*;doc/latex/*;doc/html-Board/*)

if(WIN32)
  set(CPACK_GENERATOR "NSIS;")
else()
  set(CPACK_GENERATOR "TGZ;DEB")
endif()

INCLUDE(CPack)

# -----------------------------------------------------------------------------
# CTest/Debug options
# -----------------------------------------------------------------------------
if (BUILD_TESTING)
  message(STATUS "Build test files ENABLED")
  ENABLE_TESTING()
  include(CTest)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"  OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra\
       -pedantic -Wno-unknown-pragmas -W -Wshadow -Wunused-variable  \
       -Wunused-parameter -Wunused-function        -Wunused  -Wno-long-long\
        -Wno-system-headers -Wno-deprecated -Wno-dtor-name -Woverloaded-virtual -Wwrite-strings")

    else()
      MESSAGE(STATUS "**************************** Not clang ${CMAKE_CXX_COMPILER_ID}")
    endif()
  if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra\
         -pedantic -Wno-unknown-pragmas -W -Wshadow -Wunused-variable  \
         -Wunused-parameter -Wunused-function        -Wunused  -Wno-long-long\
          -Wno-system-headers -Wno-deprecated -Wno-dtor-name -Woverloaded-virtual -Wwrite-strings")
  endif()
  if (CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -Wall -Wextra -W -Wno-unknown-pragmas\
        -Wno-long-long -pedantic")
  endif()
  if (CMAKE_C_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -Wall -Wextra -W -Wno-unknown-pragmas\
      -Wno-long-long -pedantic")
  endif()
  add_subdirectory (${PROJECT_SOURCE_DIR}/tests)
else()
  message(STATUS "Build test files DISABLED (you can activate unit tests with '-DBUILD_TESTING=ON' cmake option)")
endif()
message(STATUS "-------------------------------------------------------------------------------")
