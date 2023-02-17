
# -----------------------------------------------------------------------------
# CTest/Debug options
# -----------------------------------------------------------------------------
if (BUILD_TESTING)
  message(STATUS "Build test files ENABLED")
  ENABLE_TESTING()
  include(CTest)
  if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra -pedantic -W -Wshadow -Wunused-variable  -Wunused-parameter -Wunused-function        -Wunused  -Wno-long-long -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -fprofile-arcs -ftest-coverage")
  endif()
  if (CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -Wall -Wextra -W -Wno-long-long -pedantic -fprofile-arcs -ftest-coverage")
  endif()
  add_subdirectory (${PROJECT_SOURCE_DIR}/tests)
else()
  message(STATUS "Build test files DISABLED (you can activate unit tests with '-DBUILD_TESTING=ON' cmake option)")
endif()
message(STATUS "-------------------------------------------------------------------------------")
