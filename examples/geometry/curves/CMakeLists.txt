add_subdirectory(estimation)

set(DGTAL_EXAMPLES_SRC
  greedy-dss-decomposition
  greedyAlphaThickDecomposition
  convex-and-concave-parts
  exampleAlphaThickSegment
  exampleAlphaThickSegmentNoisy
  exampleAlphaThickSegmentTgtCover
  exampleGridCurve2d
  exampleStabbingLineComputer
  exampleStabbingCircleComputer
  exampleFrechetShortcut
  exampleArithmeticalDSS
  exampleArithmeticalDSSComputer
  exampleDSLSubsegment
  exampleArithmeticalDSL
  exampleNaive3DDSSComputer
  exampleArithDSSIterator
  exampleDigitalConvexity
  exampleRationalConvexity
  freemanChainDisplay
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

### Test with Polyscope viewer
if (  DGTAL_WITH_POLYSCOPE_VIEWER)
   set(DGTAL_EXAMPLES_POLYSCOPE_SRC
     exampleArithDSS3d
     exampleGridCurve3d
     exampleGridCurve3d-2
     exampleParamCurve3dDigitization
     exampleParamCurve3dDigitizationTransformationDecorator
     exampleTrofoliKnot
     )

  foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC} )
      DGtal_add_example(${FILE})
  endforeach()

endif()

