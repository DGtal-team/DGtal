set(DGTAL_EXAMPLES_IO_SRC
	display3DToOFF
)

foreach(FILE ${DGTAL_EXAMPLES_IO_SRC})
   DGtal_add_example(${FILE})
endforeach()


if (  WITH_VISU3D_QGLVIEWER )
  set(DGTAL_EXAMPLES_QGL_SRC
    digitalSetFromVol
    digitalSetFromPointList
    meshFromOFF
    viewDualSurface
    )
  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC})
     DGtal_add_example(${FILE})
  endforeach()

endif()

add_subdirectory(viewers)
add_subdirectory(boards)

