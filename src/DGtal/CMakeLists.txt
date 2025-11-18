if (DGTAL_BUILD_SHARED_LIBS)
  add_library(DGtal SHARED helpers/StdDefs.cpp)
else()
  add_library(DGtal STATIC helpers/StdDefs.cpp)
endif()
add_library(DGtal::DGtal ALIAS DGtal)

if( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
  # Add -fPIC
  set_target_properties(DGtal PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_compile_definitions(DGtal PUBLIC -DBOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(DGtal PUBLIC
  # src/DGtal
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  # build/src/DGtal
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
  # INSTALL_INTERFACE must not contain absolute paths
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(DGtal PUBLIC DGtal_STB DGTAL_LibBoard DGTAL_BoostAddons Boost::boost Eigen3_Eigen)
