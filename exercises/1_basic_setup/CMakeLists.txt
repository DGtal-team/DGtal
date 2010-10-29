SET(DGTAL_EXO_SRC
  helloworld
  pointvectors)

FOREACH(FILE ${DGTAL_EXO_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${DGtal_LIBRARIES})
ENDFOREACH(FILE)
