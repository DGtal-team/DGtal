add_executable(dgtalCalculus dgtalCalculus.cpp)
target_include_directories(dgtalCalculus PRIVATE polyscope)
target_link_libraries(dgtalCalculus DGtal ${DGTAL_LIBRARIES} polyscope)
