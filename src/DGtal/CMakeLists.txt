if (DGTAL_BUILD_SHARED_LIBS)
  add_library(DGtal SHARED helpers/StdDefs.cpp)
else()
  add_library(DGtal STATIC helpers/StdDefs.cpp)
endif()

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
  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR_RELATIVE}>
)

target_link_libraries(DGtal PUBLIC DGtal_STB DGTAL_LibBoard DGTAL_BoostAddons)

#Bugfix warnings in boost headers https://stackoverflow.com/questions/52135983/cmake-target-link-libraries-include-as-system-to-suppress-compiler-warnings
get_target_property(lib_include_dirs Boost::boost INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(DGtal SYSTEM PUBLIC ${lib_include_dirs})
target_link_libraries(DGtal Bosot::boost)
