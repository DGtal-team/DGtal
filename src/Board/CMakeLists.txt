
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


add_subdirectory(examples)