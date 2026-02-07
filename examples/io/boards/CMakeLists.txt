set(DGTAL_TESTS_SRC
   dgtalBoard2D-1-points
   dgtalBoard2D-2-sets
   dgtalBoard2D-3-custom-classes
   dgtalBoard2D-3-custom-points
   dgtalBoard2D-4-colormaps
   logoDGtal
   exampleBezierCurve
   )

FOREACH(FILE ${DGTAL_TESTS_SRC})
     DGtal_add_example(${FILE})
endforeach()
