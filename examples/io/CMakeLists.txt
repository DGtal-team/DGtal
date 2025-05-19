if (  WITH_VISU3D_QGLVIEWER )
  set(DGTAL_EXAMPLES_QGL_SRC
    digitalSetFromVol
    digitalSetFromPointList
    #    meshFromOFF
    viewDualSurface
    )
  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC})
     DGtal_add_example(${FILE})
  endforeach()

  add_subdirectory(viewers)
endif()

add_subdirectory(boards)

