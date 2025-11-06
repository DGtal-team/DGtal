
set(DGTAL_EXAMPLES_SRC
	exampleIntegralInvariantCurvature2D
	exampleEstimatorFromSurfelFunctors
    dvcm-2d-curvature
    examplePlaneProbingTetrahedronEstimator
    examplePlaneProbingParallelepipedEstimator
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()


### with polyscope viewer
if (  DGTAL_WITH_POLYSCOPE_VIEWER )
   set(DGTAL_EXAMPLES_POLYSCOPE_SRC
	greedy-plane-segmentation
	greedy-plane-segmentation-ex2
	greedy-plane-segmentation-ex3
	exampleIntegralInvariantCurvature3D
    dvcm-3d
    examplePlaneProbingSurfaceLocalEstimator
    exampleMaximalSegmentSliceEstimation
   )
  foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
      DGtal_add_example(${FILE})
 endforeach()

endif()
