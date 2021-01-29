set(DGTAL_EXAMPLES_IO_SRC
	display3DToOFF
)

foreach(FILE ${DGTAL_EXAMPLES_IO_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal)
endforeach()


if (  WITH_VISU3D_QGLVIEWER )
  set(DGTAL_EXAMPLES_QGL_SRC
    digitalSetFromVol
    digitalSetFromPointList
    meshFromOFF
    viewDualSurface
    )
  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal)
  endforeach()

  add_subdirectory(viewers)
endif()

add_subdirectory(boards)

