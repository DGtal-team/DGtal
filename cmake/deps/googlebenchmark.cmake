# -----------------------------------------------------------------------------
# Fetching google benchmark (only if the BUILD_TESTING variable has been set to true)
# -----------------------------------------------------------------------------

if (TARGET benchmark)
  return()
endif()

include(FetchContent)

FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.6.1
    GIT_SHALLOW    TRUE
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Enabling googletest in benchmark")
FetchContent_MakeAvailable(benchmark)
