set(DGTAL_EXAMPLES_SRC
  exampleIteratorCompletion
  exampleFunctorHolder
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal  )
endforeach()
