# -----------------------------------------------------------------------------
# Test some objc functionalities
# -----------------------------------------------------------------------------


#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -x objective-c++")

try_compile( OBJC_SUPPORT 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/objc/objcsupport.cpp
  COMPILE_DEFINITIONS "-x objective-c++"
  OUTPUT_VARIABLE OUTPUT
  )

if ( OBJC_SUPPORT )
  set(CMAKE_FLAGS "${CMAKE_FLAGS} -DOBJC_SUPPORT ")
  message(STATUS "[objc] is supported" )
else ( OBJC_SUPPORT )
  message(STATUS "[objc] is not supported" )
endif ( OBJC_SUPPORT )


