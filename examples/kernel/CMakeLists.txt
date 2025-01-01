set(DGTAL_EXAMPLES_SRC
  examplePointFunctorHolder
)

if(DGTAL_WITH_OPENMP)
  set(DGTAL_EXAMPLES_SRC ${DGTAL_EXAMPLES_SRC}
    exampleHyperRectDomainParallelScan
  )
endif()

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()
