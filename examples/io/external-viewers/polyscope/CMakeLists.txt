if (NOT DGTAL_WITH_POLYSCOPE_VIEWER)
  return()
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
  exampleGenericLatticeConvexHull3D
  exampleGenericLatticeConvexHull4D
  tangency-explorer
  tangency-reconstruction
)

foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
    DGtal_add_example(${FILE})
endforeach()


