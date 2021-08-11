
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
if (  WITH_VISU3D_QGLVIEWER )
   set(DGTAL_EXAMPLES_QGL_SRC
	greedy-plane-segmentation
	greedy-plane-segmentation-ex2
	greedy-plane-segmentation-ex3
	exampleIntegralInvariantCurvature3D
    dvcm-3d
    examplePlaneProbingSurfaceLocalEstimator
    exampleMaximalSegmentSliceEstimation
   )
  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC})
      DGtal_add_example(${FILE})
 endforeach()

endif()
