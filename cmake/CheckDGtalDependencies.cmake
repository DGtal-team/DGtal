# -----------------------------------------------------------------------------
# Check Mandatory Dependencies
# -----------------------------------------------------------------------------

message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "DGtal required dependencies: ")

# -----------------------------------------------------------------------------
# Downloading external deps
# -----------------------------------------------------------------------------
message(STATUS "Downloading external projects")
include(FetchExternalDeps)
message(STATUS "Done.")

# -----------------------------------------------------------------------------
# Looking for boost
# -----------------------------------------------------------------------------
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_FOUND FALSE)
find_package(Boost 1.50.0 REQUIRED)
if ( Boost_FOUND )
  target_compile_definitions(DGtal PUBLIC ${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
  # SYSTEM to avoid warnings from boost.
  target_include_directories(DGtal SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )
  set(DGtalLibInc ${DGtalLibInc} ${Boost_INCLUDE_DIRS})
endif()

# -----------------------------------------------------------------------------
# Looking for zlib
# -----------------------------------------------------------------------------
set(ZLIB_FOUND FALSE)
find_package(ZLIB REQUIRED)
if ( ZLIB_FOUND )
  target_include_directories(DGtal SYSTEM PUBLIC ${ZLIB_INCLUDE_DIRS} )
  set(DGtalLibInc ${DGtalLibInc} ${ZLIB_INCLUDE_DIRS})
  target_link_libraries(DGtal PUBLIC ${ZLIB_LIBRARIES})
  set(DGtalLibDependencies ${DGtalLibDependencies} ${ZLIB_LIBRARIES})
endif()

# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT APPLE)
  target_link_libraries(DGtal PUBLIC rt)
  set(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
endif()
