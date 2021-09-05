

# -----------------------------------------------------------------------------
# Fetching polyscope (only if the WITH_POLYSCOPE_EXAMPLES Variable has been set to true)
# -----------------------------------------------------------------------------
if (BUILD_POLYSCOPE_EXAMPLES)
  message(STATUS "    polyscope (v1.2.0)")
  include(polyscope)
endif()


add_executable(dgtalCalculus dgtalCalculus.cpp)
target_link_libraries(dgtalCalculus DGtal ${DGTAL_LIBRARIES} polyscope)
