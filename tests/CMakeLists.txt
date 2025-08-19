#CMakeLists associated to the tests subdir
#David Coeurjolly <david.coeurjolly@liris.cnrs.fr>

# # Make sure the compiler can find include files.
# include_directories (${PROJECT_SOURCE_DIR}/tests/)

# ConfigTest.h instanciation.
configure_file(${PROJECT_SOURCE_DIR}/tests/ConfigTest.h.in
  ${PROJECT_BINARY_DIR}/tests/ConfigTest.h)

# To speedup compilation time
# See https://github.com/catchorg/Catch2/blob/devel/docs/slow-compiles.md
# and https://stackoverflow.com/a/1388969
add_library(DGtalCatch STATIC DGtalCatch.cpp)
target_link_libraries(DGtalCatch PRIVATE Catch2::Catch2)


#------TESTS subdirectories ------
if (DGTAL_BUILD_TESTS)
  # For DGtal_add_test function
  include(TestFunctions.cmake)

  add_subdirectory(base)
  add_subdirectory(kernel)
  add_subdirectory(math)
  add_subdirectory(arithmetic)
  add_subdirectory(geometry)
  add_subdirectory(graph)
  add_subdirectory(topology)
  add_subdirectory(io)
  add_subdirectory(images)
  add_subdirectory(helpers)
  add_subdirectory(shapes)
  add_subdirectory(dec)
endif()

#------BENCHMARKS subdirectory ------
if (DGTAL_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()