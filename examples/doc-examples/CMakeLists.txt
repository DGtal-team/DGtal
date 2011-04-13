SET(DGTAL_DOC_EX_SRC
  demo-kernel-1
  kernelDomain
  range
  ctopo-1
  )

FOREACH(FILE ${DGTAL_DOC_EX_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)

