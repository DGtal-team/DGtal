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
FIND_PACKAGE(Boost 1.46.0 REQUIRED)
if ( Boost_FOUND )
  message(STATUS "Boost found.")
  include_directories( ${Boost_INCLUDE_DIRS} )
  SET(DGtalLibInc ${DGtalLibInc} ${Boost_INCLUDE_DIRS})
endif( Boost_FOUND )

# -----------------------------------------------------------------------------
# Check some CPP11 features in the compiler
# -----------------------------------------------------------------------------
SET(C11_FOUND_DGTAL 0)
SET(C11_AUTO_DGTAL 0)
SET(C11_FORWARD_DGTAL 0)
SET(C11_INITIALIZER_DGTAL 0)
SET(C11_ARRAY 0)
INCLUDE(CheckCPP11)
IF (CPP11_INITIALIZER_LIST OR CPP11_AUTO OR CP11_FORWARD_LIST)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x ")
  SET(C11_FOUND_DGTAL 1)
  IF (CPP11_AUTO)
    SET(C11_AUTO_DGTAL 1)
    SET(C11_FEATURES "${C11_FEATURES} auto")
  ENDIF()
  IF (CPP11_INITIALIZER_LIST)
    SET(C11_INITIALIZER_DGTAL 1)
    SET(C11_FEATURES "${C11_FEATURES} initializer-list")
  ENDIF()
  IF (CPP11_FORWARD_LIST)
    SET(C11_FORWARD_DGTAL 1)
    SET(C11_FEATURES "${C11_FEATURES} std::forward-list")
  ENDIF()
  IF (CPP11_ARRAY)
    SET(C11_ARRAY 1)
    SET(C11_FEATURES "${C11_FEATURES} std::array")
  ENDIF()
  IF (CPP11_RREF_MOVE)
    SET(C11_RREF_MOVE 1)
    SET(C11_FEATURES "${C11_FEATURES} std::move rvalue-reference(&&)")
  ENDIF()
  MESSAGE(STATUS "Supported c++11 features: [${C11_FEATURES} ]")
  ADD_DEFINITIONS("-DWITH_C11 ")
ELSE()
  MESSAGE(FATAL_ERROR "Your compiler does not support any c++11 feature. Please specify another C++ compiler of disable this WITH_C11 option.")
ENDIF()


# -----------------------------------------------------------------------------
# Fixing Catch issue for C++11 and old GCC
# -----------------------------------------------------------------------------
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # require at least gcc 4.7
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.7)
    add_defitinions("-DCATCH_CONFIG_CPP11_NO_IS_ENUM")
    message(STATUS "Patching Catch for gcc < 4.7")
  endif()
endif()


# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT(APPLE))
  SET(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
endif()
