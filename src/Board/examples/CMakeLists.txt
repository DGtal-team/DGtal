
# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/src/)

# Make sure the linker can find the library once it is built.
link_directories (${PROJECT_BINARY_DIR})

SET(BOARD_TESTS_SRC
arithmetic
clipping
example1
example3
flag
koch
ruler
arrows
ellipse
example2
example4
graph
logo)

FOREACH(FILE ${BOARD_TESTS_SRC})
add_executable(${FILE} ${FILE})
target_link_libraries (${FILE} Board)
add_test(${FILE} ${FILE})
ENDFOREACH()

