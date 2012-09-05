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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP11_INITIALIZER_LIST")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP11_INITIALIZER_LIST )


try_compile( CPP11_AUTO 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/auto.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_AUTO )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP11_AUTO")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP11_AUTO )


try_compile( CPP11_FORWARD_LIST
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/forward_list.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_FORWARD_LIST )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP11_FORWARD_LIST")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP11_FORWARD_LIST )

try_compile( CPP11_ARRAY
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp11/array.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP11_ARRAY )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP11_ARRAY")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( CPP11_ARRAY)

