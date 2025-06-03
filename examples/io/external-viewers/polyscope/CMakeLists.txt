if (NOT DGTAL_BUILD_POLYSCOPE_EXAMPLES)
  return()
endif()

if (NOT TARGET polyscope)
  # Fetch library
  include(polyscope)
endif()

set(DGTAL_EXAMPLES_POLYSCOPE_SRC
  dgtalCalculus
  dgtalCalculus-single
  dgtalCalculus-poisson
  dgtalCalculus-bunny
  dgtalCalculus-halfsphere
  dgtalCalculus-geodesic
  exampleVectorHeatMethod
  exampleHarmonicParametrization
  exampleBunnyHead
  tangency-explorer
  tangency-reconstruction
)

foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
    DGtal_add_example(${FILE})
    target_link_libraries(${FILE} PUBLIC polyscope)
endforeach()


