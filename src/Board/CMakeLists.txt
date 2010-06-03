# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/src/Board)

SET(BOARD_SRC ${BOARD_SRC}
  Board
  Color
  Path
  PSFonts
  Rect
  ShapeList
  Shapes
  Tools
  Transforms)

# Create a library called "Board".
add_library (${LIBBOARD_NAME}  ${BOARD_SRC})

message(STATUS "BOARD_SRCs: "${BOARD_SRC})

install(TARGETS ${LIBBOARD_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

#add_subdirectory(examples)

SET(BOARD_TESTS_SRC
    koch.cpp
    logo.cpp)

FOREACH(FILE ${BOARD_TESTS_SRC})
  add_executable(${FILE} examples/${FILE})
  target_link_libraries (${FILE} ${LIBBOARD_NAME})
#  add_test(${FILE} ${FILE})
ENDFOREACH()
