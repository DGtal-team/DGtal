if (  DGTAL_WITH_POLYSCOPE_VIEWER )
  set(DGTAL_EXAMPLES_POLYSCOPE_SRC
    digitalSetFromVol
    digitalSetFromPointList
    #    meshFromOFF
    viewDualSurface
    )
  foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
     DGtal_add_example(${FILE})
  endforeach()

  add_subdirectory(viewers)
endif()

add_subdirectory(boards)
add_subdirectory(external-viewers)
