# -----------------------------------------------------------------------------
# Fetching Catch2 and googlebenchmark
# (only if the DGTAL_BUILD_TESTS variable has been set to true)
# -----------------------------------------------------------------------------
if (DGTAL_BUILD_TESTS)

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
