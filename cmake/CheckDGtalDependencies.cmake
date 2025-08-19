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
set(DGtalLibDependencies ${DGtalLibDependencies} Boost::headers )

target_compile_definitions(DGTAL_BoostAddons PUBLIC ${BOOST_DEFINITIONS})
#target_include_directories(DGTAL_BoostAddons SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )

target_compile_definitions(DGTAL_LibBoard PUBLIC ${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
target_include_directories(DGTAL_LibBoard SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )

# -----------------------------------------------------------------------------
# Looking for zlib
# -----------------------------------------------------------------------------
find_package(ZLIB REQUIRED)
target_link_libraries(DGtal PUBLIC ZLIB::ZLIB)
target_link_libraries(DGTAL_BoostAddons PUBLIC ZLIB::ZLIB Boost::headers)
set(DGtalLibDependencies ${DGtalLibDependencies} ${ZLIB_LIBRARIES})

# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT APPLE)
  target_link_libraries(DGtal PUBLIC rt)
  set(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
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
set(DGtalLibDependencies ${DGtalLibDependencies} Eigen3::Eigen)
target_compile_definitions(DGtal PUBLIC "-DDGTAL_WITH_EIGEN=true")