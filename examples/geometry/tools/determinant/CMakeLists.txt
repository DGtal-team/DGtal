set(DGTAL_EXAMPLES_SRC
   exampleInHalfPlane
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()
