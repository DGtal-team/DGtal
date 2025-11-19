project(SystemWideCompile)
cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)

find_package(DGtal REQUIRED)
add_executable(SystemWideCompile main.cpp)
target_link_libraries(SystemWideCompile DGtal::DGtal)
