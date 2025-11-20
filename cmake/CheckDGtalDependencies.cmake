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
include(boost)

set(DGtalLibDependencies ${DGtalLibDependencies} Boost::boost )

# -----------------------------------------------------------------------------
# Looking for zlib
# -----------------------------------------------------------------------------
find_package(ZLIB REQUIRED)
set(DGtalLibDependencies ${DGtalLibDependencies} ZLIB::ZLIB)

# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT APPLE)
  set(DGtalLibDependencies ${DGtalLibDependencies} rt)
endif()

# -----------------------------------------------------------------------------
# Fetching Catch2 and googlebenchmark
# (only if the DGTAL_BUILD_TESTS variable has been set to true)
# -----------------------------------------------------------------------------
if (DGTAL_BUILD_TESTS OR DGTAL_BUILD_BENCHMARKS)

  message(STATUS "    Catch2 (v2.13.7)")
  include(catch2)
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
  include(CTest)

  message(STATUS "    Google benchmark (v1.6.1)")
  include(googlebenchmark)
endif()

# -----------------------------------------------------------------------------
# Fetching Eigen3
# -----------------------------------------------------------------------------
include(eigen)
set(DGtalLibDependencies ${DGtalLibDependencies} DGtalEigen3)
