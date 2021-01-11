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
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal)
endforeach()



if(MAGICK++_FOUND)
  set(DGTALE_EXAMPLES_IO_Magick
    freemanChainDisplay
    )
  foreach(FILE ${DGTALE_EXAMPLES_IO_Magick})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal)
  endforeach()
endif()


### Test with QGLViewer
if (  WITH_VISU3D_QGLVIEWER )
   set(DGTAL_EXAMPLES_QGL_SRC
     exampleArithDSS3d
     exampleGridCurve3d
     exampleGridCurve3d-2
     exampleParamCurve3dDigitization
     exampleParamCurve3dDigitizationTransformationDecorator
     exampleTrofoliKnot
     )

  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC} )
   add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal)
  endforeach()

endif()

