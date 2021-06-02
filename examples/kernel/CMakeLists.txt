set(DGTAL_EXAMPLES_SRC
  examplePointFunctorHolder
)

if(WITH_OPENMP)
  set(DGTAL_EXAMPLES_SRC ${DGTAL_EXAMPLES_SRC}
    exampleHyperRectDomainParallelScan
  )
endif()

foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE} DGtal  )
endforeach()
