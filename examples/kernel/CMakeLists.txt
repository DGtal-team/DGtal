SET(DGTAL_EXAMPLES_SRC
  examplePointFunctorHolder
)

IF(WITH_OPENMP)
  SET(DGTAL_EXAMPLES_SRC ${DGTAL_EXAMPLES_SRC}
    exampleHyperRectDomainParallelScan
  )
ENDIF(WITH_OPENMP)

FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal  )
ENDFOREACH(FILE)
