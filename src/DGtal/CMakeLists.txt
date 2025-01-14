add_library(DGtal SHARED helpers/StdDefs.cpp)

target_link_libraries(DGtal PUBLIC DGtal_STB DGTAL_LibBoard DGTAL_BoostAddons)
target_include_directories(DGtal PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")