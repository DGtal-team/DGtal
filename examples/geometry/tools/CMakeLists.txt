add_subdirectory(determinant)

set(DGTAL_EXAMPLES_SRC
  examplePreimage
  exampleConvexHull2D
  exampleAlphaShape
  exampleLatticeBallQuickHull3D
  exampleRationalBallQuickHull3D
  exampleLatticeBallDelaunay2D
  exampleQuickHull3D
  exampleLatticeBallDelaunay3D
  exampleRationalBallDelaunay3D
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
  DGtal_add_example(${FILE})
endforeach()

if ( DGTAL_WITH_GMP )
  set(DGTAL_EXAMPLES_GMP_SRC
    checkLatticeBallQuickHull
  )
  foreach(FILE ${DGTAL_EXAMPLES_GMP_SRC})
    DGtal_add_example(${FILE})
  endforeach()
endif()

