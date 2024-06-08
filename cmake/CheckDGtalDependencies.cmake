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
find_package(Boost 1.50.0 REQUIRED)
target_compile_definitions(DGtal PUBLIC ${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
# SYSTEM to avoid warnings from boost.
target_include_directories(DGtal SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )

# -----------------------------------------------------------------------------
# Looking for zlib
# -----------------------------------------------------------------------------
find_package(ZLIB REQUIRED)
target_link_libraries(DGtal PUBLIC ZLIB::ZLIB)
set(DGtalLibDependencies ${DGtalLibDependencies} ${ZLIB_LIBRARIES})

# -----------------------------------------------------------------------------
# Setting librt dependency on Linux
# -----------------------------------------------------------------------------
if (UNIX AND NOT APPLE)
  target_link_libraries(DGtal PUBLIC rt)
  set(DGtalLibDependencies ${DGtalLibDependencies} -lrt)
endif()

# -----------------------------------------------------------------------------
# Eigen (already fetched)
# -----------------------------------------------------------------------------
set(WITH_EIGEN ON)
set(EIGEN_FOUND_DGTAL 1)
target_compile_definitions(DGtal PUBLIC "-DWITH_EIGEN=true")
set(DGtalLibDependencies ${DGtalLibDependencies} Eigen3::Eigen)
