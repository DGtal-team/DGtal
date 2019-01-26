SET(DGTAL_EXAMPLES_SRC
  examplePointFunctorHolder
)

FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal  )
ENDFOREACH(FILE)
