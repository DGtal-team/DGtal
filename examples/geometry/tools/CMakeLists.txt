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
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE} DGtal  )
endforeach()

if ( WITH_GMP )
  set(DGTAL_EXAMPLES_GMP_SRC
    checkLatticeBallQuickHull
  )
  foreach(FILE ${DGTAL_EXAMPLES_GMP_SRC})
    add_executable(${FILE} ${FILE}.cpp)
    target_link_libraries (${FILE} DGtal)
  endforeach()
endif()

