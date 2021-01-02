add_subdirectory(determinant)

set(DGTAL_EXAMPLES_SRC
  examplePreimage
  exampleConvexHull2D
  exampleAlphaShape
  exampleLatticeBallQuickHull3D
  exampleRationalBallQuickHull3D
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal  )
endforeach()



