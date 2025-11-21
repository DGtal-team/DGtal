project(SystemWideCompile)
cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)

find_package(DGtal REQUIRED PATHS /home/pattoune/install/DGtal)

include(CMakePrintHelpers)
cmake_print_properties(
  TARGETS Boost::Boost
  PROPERTIES
    INCLUDE_DIRECTORIES
    INTERFACE_INCLUDE_DIRECTORIES
)

add_executable(SystemWideCompile main.cpp)
target_link_libraries(SystemWideCompile DGtal::DGtal)
