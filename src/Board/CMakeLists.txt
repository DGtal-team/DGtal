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

target_link_libraries(DGTAL_LibBoard PRIVATE Boost::boost)
target_compile_definitions(DGTAL_LibBoard PUBLIC ${BOOST_DEFINITIONS} -DBOOST_ALL_NO_LIB)
target_include_directories(DGTAL_LibBoard SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )

target_link_libraries(DGTAL_LibBoard PRIVATE ${BoardLibDependencies})
target_compile_options(DGTAL_LibBoard PRIVATE ${BoardLibCompileDefs})
target_include_directories(DGTAL_LibBoard PRIVATE ${BoardLibIncludeDirs})

target_include_directories(DGTAL_LibBoard PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
  $<INSTALL_INTERFACE:${DGTAL_INSTALL_DEPS_DESTINATION}>
)
