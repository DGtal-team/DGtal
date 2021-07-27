set(DGTAL_EXAMPLES_TOPO_SRC
  graphTraversal
)

foreach(FILE ${DGTAL_EXAMPLES_TOPO_SRC})
     DGtal_add_example(${FILE})
endforeach()


if ( WITH_VISU3D_QGLVIEWER )
   set(DGTAL_EXAMPLES_QGL_SRC
     volDistanceTraversal
   )
   foreach(FILE ${DGTAL_EXAMPLES_QGL_SRC})
     DGtal_add_example(${FILE})
   endforeach()
endif()

