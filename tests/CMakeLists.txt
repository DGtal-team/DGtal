#CMakeLists associated to the tests subdir
#David Coeurjolly <david.coeurjolly@liris.cnrs.fr>

add_subdirectory(board-examples)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/src/)
# Make sure the linker can find the Hello library once it is built.
link_directories (${PROJECT_BINARY_DIR})

SET(DGTAL_TESTS_SRC
   utils/testClock
   utils/testTrace
   kernel/testInteger
   math/testMeasure
   kernel/testDigitalSet
   kernel/testDomainSpanIterator
   kernel/testHyperRectDomain
   kernel/testHyperRectDomain-snippet
   kernel/testImage
   kernel/testImageSpanIterators
   kernel/testPointVector
   kernel/testSpaceND
   geometry/2d/testArithDSS)


FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
  add_test(${FILE} ${FILE})
ENDFOREACH(FILE)


