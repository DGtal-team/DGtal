# -----------------------------------------------------------------------------
# Check Mandatory Dependencies
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Look for boost 
# -----------------------------------------------------------------------------

set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_FOUND FALSE)
FIND_PACKAGE(Boost 1.42.0 REQUIRED COMPONENTS program_options)
if ( Boost_FOUND )
  message(STATUS "Boost and boost_program_options found.")
  include_directories( ${Boost_INCLUDE_DIRS} )
  SET(DGtalLibDependencies ${DGtalLibDependencies} 
     ${Boost_LIBRAIRIES}  
     ${Boost_PROGRAM_OPTIONS_LIBRARY})
   SET(DGtalLibInc ${Boost_INCLUDE_DIRS})
endif( Boost_FOUND )

# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT(APPLE))
  SET(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
endif()
