

# -----------------------------------------------------------------------------
# Fetching polyscope (only if the WITH_POLYSCOPE_EXAMPLES Variable has been set to true)
# -----------------------------------------------------------------------------
if (BUILD_POLYSCOPE_EXAMPLES)
  message(STATUS "    polyscope (v1.2.0)")
  include(polyscope)
endif()

set(POLYSCOPE_SRCS
  dgtalCalculus
  dgtalCalculus-single
  dgtalCalculus-poisson
)

foreach(FILE ${POLYSCOPE_SRCS})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries(${FILE} DGtal ${DGTAL_LIBRARIES} polyscope)
endforeach()
