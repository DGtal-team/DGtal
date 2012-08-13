# -----------------------------------------------------------------------------
# Test some objc functionalities
# -----------------------------------------------------------------------------


try_compile( OBJC_AUTO 
  ${CMAKE_BINARY_DIR}/CMakeTmp
  ${CMAKE_SOURCE_DIR}/cmake/src/objc/auto.mm
  OUTPUT_VARIABLE OUTPUT
  )
if ( OBJC_AUTO )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOBJC_AUTO")
  message(STATUS "[objc] with auto" )
else ( OBJC_AUTO )
  message(STATUS "[objc] auto not found" )
endif ( OBJC_AUTO )


