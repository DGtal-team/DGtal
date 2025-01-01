# -----------------------------------------------------------------------------
# Fetching google benchmark (only if the DGTAL_BUILD_TESTS variable has been set to true)
# -----------------------------------------------------------------------------

if (TARGET benchmark)
  return()
endif()

include(cmake/CPM.cmake)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Enabling googletest in benchmark")

CPMAddPackage(
  NAME benchmark
  VERSION 1.6.1
  GITHUB_REPOSITORY google/benchmark
)
