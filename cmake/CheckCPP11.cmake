# -----------------------------------------------------------------------------
# Test some c++11 functionalities
# -----------------------------------------------------------------------------

try_compile( CPP11_INITIALIZER_LIST 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/initializer_list.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_INITIALIZER_LIST )
  add_definitions("-DCPP11_INITIALIZER_LIST")
endif ( CPP11_INITIALIZER_LIST )


try_compile( CPP11_AUTO 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/auto.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_AUTO )
  add_definitions("-DCPP11_AUTO")
endif ( CPP11_AUTO )


try_compile( CPP11_FORWARD_LIST
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/forward_list.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_FORWARD_LIST )
  add_definitions("-DCPP11_FORWARD_LIST") 
endif ( CPP11_FORWARD_LIST )

try_compile( CPP11_ARRAY
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/array.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_ARRAY )
  add_definitions("-DCPP11_ARRAY")
endif ( CPP11_ARRAY)

try_compile( CPP11_RREF_MOVE
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/rref-move.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_RREF_MOVE )
  add_definitions("-DCPP11_RREF_MOVE")
endif ( CPP11_RREF_MOVE )

