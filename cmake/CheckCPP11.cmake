# -----------------------------------------------------------------------------
# Test some c++11 functionalities
# -----------------------------------------------------------------------------

try_compile( CPP11_COMPATIBLE_BY_DEFAULT
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/cpp11_check.cpp
  OUTPUT_VARIABLE OUTPUT
  )

IF (NOT CPP11_COMPATIBLE_BY_DEFAULT)
  try_compile( CPP11_COMPATIBLE_AFTER_FLAG
    ${CMAKE_BINARY_DIR}/CMakeTmp
    ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/cpp11_check.cpp
    COMPILE_DEFINITIONS "-std=c++11"
    OUTPUT_VARIABLE OUTPUT
    )
  SET(CPP11_COMPATIBLE_AFTER_FLAG CPP11_COMPATIBLE_AFTER_FLAG PARENT_SCOPE)
ENDIF()

SET(CPP11_COMPATIBLE CPP11_COMPATIBLE_BY_DEFAULT OR CPP11_COMPATIBLE_AFTER_FLAG)

