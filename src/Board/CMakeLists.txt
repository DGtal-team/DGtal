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

target_include_directories(DGTAL_LibBoard PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
  # INSTALL_INTERFACE must not contain absolute paths
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR_RELATIVE}>
)
