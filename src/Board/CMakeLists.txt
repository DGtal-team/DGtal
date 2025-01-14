set(BOARD_SRC
  		Board.cpp
  		Path.cpp
  		PSFonts.cpp
  		Rect.cpp
  		ShapeList.cpp
  		Shapes.cpp
  		Tools.cpp
  		Transforms.cpp)

# @TODO : proper parent directory
add_library(DGTAL_LibBoard STATIC ${BOARD_SRC})
target_include_directories(DGTAL_LibBoard PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")