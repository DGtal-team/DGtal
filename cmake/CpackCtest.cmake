
#------------------------------------------------------------------------------
# CPack Configurations
#------------------------------------------------------------------------------
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(${CMAKE_MODULE_PATH}/DGtalCPackConfig.cmake)

# -----------------------------------------------------------------------------
# CTest options
# -----------------------------------------------------------------------------
ENABLE_TESTING()
include(CTest)
if (BUILD_TESTING)
  if (CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -pedantic -W -Wshadow -Wunused-variable  -Wunused-parameter -Wunused-function        -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -fprofile-arcs -ftest-coverage")
  endif (CMAKE_COMPILER_IS_GNUCXX)
  if (CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -Wall -W -pedantic -fprofile-arcs -ftest-coverage")
  endif (CMAKE_COMPILER_IS_GNUCC)
  add_subdirectory (tests)
endif (BUILD_TESTING)
