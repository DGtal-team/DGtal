
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


### with QGLViewer
if (  DGTAL_WITH_POLYSCOPE )
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
