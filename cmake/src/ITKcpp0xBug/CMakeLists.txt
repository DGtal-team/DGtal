#David Coeurjolly - it03/2012
PROJECT(ITKCPP0XBUG)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})
include_directories(${ITK_INCLUDE_DIRS})
add_executable(itkcpp0x itkcpp0x.cpp)
target_link_libraries(itkcpp0x  ${ITK_LIBRARIES})
