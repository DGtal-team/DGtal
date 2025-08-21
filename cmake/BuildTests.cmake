option(DGTAL_BUILD_TESTS "Build testing." OFF)
option(DGTAL_BUILD_BENCHMARKS "Build benchmarks." OFF)

if (DGTAL_BUILD_TESTS)
  message(STATUS "Build tests ENABLED")
else()
  message(STATUS "Build test files DISABLED (you can activate the examples with '-DDGTAL_BUILD_TESTS=ON' cmake option)")
endif()
message(STATUS "-------------------------------------------------------------------------------")

if (DGTAL_BUILD_BENCHMARKS)
  message(STATUS "Build benchmarks ENABLED")
else()
  message(STATUS "Build benchmarks DISABLED (you can activate the examples with '-DDGTAL_BUILD_BENCHMARK=ON' cmake option)")
endif()

message(STATUS "-------------------------------------------------------------------------------")

# -----------------------------------------------------------------------------
# Randomized build 1/2.
# -----------------------------------------------------------------------------
SET(DGTAL_RANDOMIZED_TESTING_THRESHOLD "100" CACHE INTERNAL "Threshold for the random selection of unit tests to build.")
SET(DGTAL_RANDOMIZED_TESTING_WHITELIST "" CACHE INTERNAL "List of whitelisted unit-test/examples to build.")

# -----------------------------------------------------------------------------
# Randomized build 2/2.
# -----------------------------------------------------------------------------
if (DGTAL_BUILD_EXAMPLES OR DGTAL_BUILD_TESTS)
 message(STATUS "Randomized examples/testing")
 if (DGTAL_RANDOMIZED_TESTING_THRESHOLD EQUAL "100")
  message(STATUS "All examples or unit-tests will be compiled (cmake variable DGTAL_RANDOMIZED_TESTING_THRESHOLD)")
 else()
  message(STATUS "Only ~${DGTAL_RANDOMIZED_TESTING_THRESHOLD}% of the (randomly selected) examples/tests will be  compiled/run")
  message(STATUS "  (you can adjust this with the cmake variable DGTAL_RANDOMIZED_TESTING_THRESHOLD)")
  message(STATUS " Whitelist: ${DGTAL_RANDOMIZED_TESTING_WHITELIST}")
endif()
message(STATUS "-------------------------------------------------------------------------------")
endif()

# -----------------------------------------------------------------------------
# CTest/Debug options
# -----------------------------------------------------------------------------
if (DGTAL_BUILD_TESTS OR DGTAL_BUILD_BENCHMARKS)
  message(STATUS "Build test/benchmarks files ENABLED")
  ENABLE_TESTING()
  include(CTest)
  
  add_subdirectory (${PROJECT_SOURCE_DIR}/tests)
endif()
