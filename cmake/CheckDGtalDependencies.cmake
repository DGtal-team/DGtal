# -----------------------------------------------------------------------------
# Check Mandatory Dependencies
# -----------------------------------------------------------------------------

message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "DGtal required dependencies: ")

# -----------------------------------------------------------------------------
# Looking for boost
# -----------------------------------------------------------------------------
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_FOUND FALSE)
FIND_PACKAGE(Boost 1.50.0 REQUIRED)
if ( Boost_FOUND )
  # SYSTEM to avoid warnings from boost.
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS} )
  SET(DGtalLibInc ${DGtalLibInc} ${Boost_INCLUDE_DIRS})
endif( Boost_FOUND )

# -----------------------------------------------------------------------------
# Looking for zlib
# -----------------------------------------------------------------------------

set(ZLIB_FOUND FALSE)
FIND_PACKAGE(ZLIB REQUIRED)
if ( ZLIB_FOUND )
  include_directories(SYSTEM ${ZLIB_INCLUDE_DIRS} )
  SET(DGtalLibInc ${DGtalLibInc} ${ZLIB_INCLUDE_DIRS})
  SET(DGtalLibDependencies ${DGtalLibDependencies} ${ZLIB_LIBRARIES})
endif( ZLIB_FOUND )


# -----------------------------------------------------------------------------
# Check some CPP11 features in the compiler
# -----------------------------------------------------------------------------
MESSAGE(STATUS "Checking C++11 compatibility:")
INCLUDE(CheckCPP11)
IF (CPP11_COMPATIBLE)
  IF (NOT CPP11_COMPATIBLE_FLAG_SET_BY_USER)
    IF (NOT MSVC)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
      MESSAGE(STATUS "  -std=c++11 added to CMAKE_CXX_FLAGS. ")
    ENDIF()
  ENDIF()
  MESSAGE(STATUS "OK.")
ELSE()
  MESSAGE(FATAL_ERROR "Your compiler does not support C++11. Please specify another C++ compiler.")
ENDIF()


# -----------------------------------------------------------------------------
# Fixing Catch issue for C++11 and old GCC
# -----------------------------------------------------------------------------
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # require at least gcc 4.7
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.7)
    add_definitions("-DCATCH_CONFIG_CPP11_NO_IS_ENUM")
    message(STATUS "Patching Catch for gcc < 4.7")
  endif()
endif()


# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT(APPLE))
  SET(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
endif()
