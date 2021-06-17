add_subdirectory(distance)

set(DGTAL_EXAMPLES_SRC
        dvcm-2d
        simpleness-convexity-2d
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE} DGtal  )
endforeach()

if (  WITH_VISU3D_QGLVIEWER )
   set(DGTAL_EXAMPLES_QGL_SRC
      geometricThinning3D
   )
  foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC})
    add_executable(${FILE} ${FILE}.cpp)
    target_link_libraries (${FILE} DGtal)
  endforeach()

endif()
