set(DGTAL_EXAMPLES_TOPO_SRC
  graphTraversal
)

foreach(FILE ${DGTAL_EXAMPLES_TOPO_SRC})
     DGtal_add_example(${FILE})
endforeach()


if ( DGTAL_WITH_POLYSCOPE_VIEWER )
   set(DGTAL_EXAMPLES_POLYSCOPE_SRC
     volDistanceTraversal
   )
   foreach(FILE ${DGTAL_EXAMPLES_POLYSCOPE_SRC})
     DGtal_add_example(${FILE})
   endforeach()
endif()
