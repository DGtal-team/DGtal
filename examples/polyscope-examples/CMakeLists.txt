# -----------------------------------------------------------------------------
# Fetching polyscope (only if the WITH_POLYSCOPE_EXAMPLES Variable has been set to true)
# -----------------------------------------------------------------------------
if (BUILD_POLYSCOPE_EXAMPLES)
  message(STATUS "    polyscope (v1.2.0)")
  include(polyscope)
endif()

if(BUILD_SHARED_LIBS)
  message(FATAL_ERROR "Polyscope examples require static build of the DGtal. Please set BUILD_SHARED_LIBS variable to OFF")
endif()
if(NOT WITH_EIGEN)
  message(FATAL_ERROR "Polyscope Polygonal Calculus examples require Eigen (WITH_EIGEN=true)")
endif()

include_directories (${PROJECT_BINARY_DIR}/examples/)

set(POLYSCOPE_SRCS
  dgtalCalculus
  dgtalCalculus-single
  dgtalCalculus-poisson
  dgtalCalculus-bunny
  dgtalCalculus-halfsphere
  dgtalCalculus-geodesic
  tangency-explorer
)

foreach(FILE ${POLYSCOPE_SRCS})
  #We should be using `DGtal_add_example(${FILE} polyscope)` to
  #enable the randomized testing but it does not work (polyscope headers are
  #not found properly).
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries(${FILE} DGtal ${DGTAL_LIBRARIES} polyscope)
endforeach()
