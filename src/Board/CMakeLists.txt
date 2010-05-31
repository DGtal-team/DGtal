
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
add_library ( Board  ${BOARD_SRC})

add_subdirectory(examples)