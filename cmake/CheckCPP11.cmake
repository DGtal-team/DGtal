# -----------------------------------------------------------------------------
# Test some c++11 functionalities
# -----------------------------------------------------------------------------

try_compile( CPP11_COMPATIBLE 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/cpp11_check.cpp
  COMPILE_DEFINITIONS "-std=c++11"
  OUTPUT_VARIABLE OUTPUT
  )

