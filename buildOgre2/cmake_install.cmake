# Install script for directory: /home/anisbenyoub/Libraries/DGtal

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/doc/DGtal/html")
FILE(INSTALL DESTINATION "/usr/local/doc/DGtal" TYPE DIRECTORY FILES "/home/anisbenyoub/Libraries/DGtal/doc/html")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include//")
FILE(INSTALL DESTINATION "/usr/local/include/" TYPE DIRECTORY FILES "/home/anisbenyoub/Libraries/DGtal/src/" FILES_MATCHING REGEX "/[^/]*\\.[^/]*h$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/DGtal//")
FILE(INSTALL DESTINATION "/usr/local/include/DGtal/" TYPE DIRECTORY FILES "/home/anisbenyoub/Libraries/DGtal/buildOgre2/src/DGtal/" FILES_MATCHING REGEX "/[^/]*\\.[^/]*h$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  IF(EXISTS "$ENV{DESTDIR}/usr/local/share/DGtal/CMake/DGtalLibraryDepends.cmake")
    FILE(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}/usr/local/share/DGtal/CMake/DGtalLibraryDepends.cmake"
         "/home/anisbenyoub/Libraries/DGtal/buildOgre2/CMakeFiles/Export/_usr/local/share/DGtal/CMake/DGtalLibraryDepends.cmake")
    IF(EXPORT_FILE_CHANGED)
      FILE(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}/usr/local/share/DGtal/CMake/DGtalLibraryDepends-*.cmake")
      IF(OLD_CONFIG_FILES)
        MESSAGE(STATUS "Old export file \"$ENV{DESTDIR}/usr/local/share/DGtal/CMake/DGtalLibraryDepends.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        FILE(REMOVE ${OLD_CONFIG_FILES})
      ENDIF(OLD_CONFIG_FILES)
    ENDIF(EXPORT_FILE_CHANGED)
  ENDIF()
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/share/DGtal/CMake/DGtalLibraryDepends.cmake")
FILE(INSTALL DESTINATION "/usr/local/share/DGtal/CMake" TYPE FILE FILES "/home/anisbenyoub/Libraries/DGtal/buildOgre2/CMakeFiles/Export/_usr/local/share/DGtal/CMake/DGtalLibraryDepends.cmake")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
     "/usr/local/share/DGtal/CMake/DGtalLibraryDepends-noconfig.cmake")
FILE(INSTALL DESTINATION "/usr/local/share/DGtal/CMake" TYPE FILE FILES "/home/anisbenyoub/Libraries/DGtal/buildOgre2/CMakeFiles/Export/_usr/local/share/DGtal/CMake/DGtalLibraryDepends-noconfig.cmake")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/share/DGtal/CMake/DGtalConfig.cmake;/usr/local/share/DGtal/CMake/DGtalConfigVersion.cmake")
FILE(INSTALL DESTINATION "/usr/local/share/DGtal/CMake" TYPE FILE FILES
    "/home/anisbenyoub/Libraries/DGtal/buildOgre2/InstallFiles/DGtalConfig.cmake"
    "/home/anisbenyoub/Libraries/DGtal/buildOgre2/InstallFiles/DGtalConfigVersion.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "dev")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/anisbenyoub/Libraries/DGtal/buildOgre2/tests/cmake_install.cmake")
  INCLUDE("/home/anisbenyoub/Libraries/DGtal/buildOgre2/src/cmake_install.cmake")
  INCLUDE("/home/anisbenyoub/Libraries/DGtal/buildOgre2/examples/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/anisbenyoub/Libraries/DGtal/buildOgre2/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/anisbenyoub/Libraries/DGtal/buildOgre2/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
