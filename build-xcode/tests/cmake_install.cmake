# Install script for directory: /Users/davidcoeurjolly/Sources/DGtal/tests

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
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
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

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/base/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/kernel/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/math/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/arithmetic/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/geometry/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/graph/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/topology/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/io/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/images/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/helpers/cmake_install.cmake")
  INCLUDE("/Users/davidcoeurjolly/Sources/DGtal/build-xcode/tests/shapes/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

