#CMakeLists associated to the tests subdir
#David Coeurjolly <david.coeurjolly@liris.cnrs.fr>


# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/include)
# Make sure the linker can find the Hello library once it is built.
link_directories (${PROJECT_BINARY_DIR})

SET(DGTAL_TESTS_SRC
   test_clock
   test_trace
   test_measure
   test_PointVector
   test_Space)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH()


