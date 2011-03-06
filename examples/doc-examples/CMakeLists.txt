SET(DGTAL_DOC_EX_SRC
  kernelDomain
  )

FOREACH(FILE ${DGTAL_DOC_EX_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)

