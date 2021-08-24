set(DGTAL_EXAMPLES_SRC
  exampleIteratorCompletion
  exampleFunctorHolder
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()
