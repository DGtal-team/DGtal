SET(DGTAL_EXO_SRC
  a_imageinit
  b_simpleDT)

FOREACH(FILE ${DGTAL_EXO_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${DGtal_LIBRARIES})
ENDFOREACH(FILE)
