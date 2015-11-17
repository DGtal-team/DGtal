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
  include_directories( ${Boost_INCLUDE_DIRS} )
  SET(DGtalLibInc ${DGtalLibInc} ${Boost_INCLUDE_DIRS})

  ## Checking boost/random ( <1.47)
  STRING (COMPARE LESS "${Boost_MINOR_VERSION}" 47 BOOST_RANDOM_OLD)
  IF (BOOST_RANDOM_OLD)
    message(STATUS "   Old boost::random found, I define BOOST_RANDOM_OLD")
    ADD_DEFINITIONS("-DBOOST_RANDOM_OLD")
  ELSE()
    message(STATUS "   boost::random ok")
  ENDIF()
endif( Boost_FOUND )

# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT(APPLE))
  SET(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
endif()
