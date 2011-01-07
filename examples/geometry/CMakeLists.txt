SET(DGTAL_EXAMPLES_SRC
  greedy-dss-decomposition
  distancetransform2D
  DSS
	freemanChainDisplay
 )

FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME} )
ENDFOREACH(FILE)





IF(MAGICK++_FOUND)
  SET(DGTALE_EXAMPLES_IO_Magick
    freemanChainDisplay
    )
  FOREACH(FILE ${DGTALE_EXAMPLES_IO_Magick})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} ${LIBDGTAL_NAME} ${LIBBOARD_NAME} ${MAGICK++_LIBRARIES})
  ENDFOREACH(FILE)
ENDIF(MAGICK++_FOUND)
