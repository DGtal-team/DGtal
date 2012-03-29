# -----------------------------------------------------------------------------
# Test some c++0x functionalities
# -----------------------------------------------------------------------------
try_compile( CPP0X_INITIALIZER_LIST 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp0x/initializer_list.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP0X_INITIALIZER_LIST )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP0X_INITIALIZER_LIST")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  message(STATUS "[c++0x] with initializer_list" )
else ( CPP0X_INITIALIZER_LIST )
  message(STATUS "[c++0x] initializer_list not found" )
endif ( CPP0X_INITIALIZER_LIST )

try_compile( CPP0X_AUTO 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/cpp0x/auto.cpp
  COMPILE_DEFINITIONS "-std=c++0x"
  OUTPUT_VARIABLE OUTPUT
  )
if ( CPP0X_AUTO )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCPP0X_AUTO")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  

  message(STATUS "[c++0x] with auto" )
else ( CPP0X_AUTO )
  message(STATUS "[c++0x] auto not found" )
endif ( CPP0X_AUTO )


