if (BUILD_SHARED_LIBS)
  add_library(DGtal SHARED helpers/StdDefs.cpp)
else()
  add_library(DGtal STATIC helpers/StdDefs.cpp)
endif()

if( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
  # Add -fPIC
  set_target_properties(DGtal PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()


target_include_directories(DGtal PUBLIC
  # src/DGtal
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  # build/src/DGtal
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
  # INSTALL_INTERFACE must not contain absolute paths
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR_RELATIVE}>
)

target_link_libraries(DGtal PUBLIC DGtal_STB DGTAL_LibBoard DGTAL_BoostAddons)