SET(DGTAL_EXAMPLES_SRC
  greedy-dss-decomposition
  ArithmeticalDSS
  convex-and-concave-parts
  freemanChainFromImage
  exampleGridCurve2d
  exampleGeometricalDSS
  exampleGeometricalDCA
)


FOREACH(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} DGtal DGtalIO )
ENDFOREACH(FILE)



IF(MAGICK++_FOUND)
  SET(DGTALE_EXAMPLES_IO_Magick
    freemanChainDisplay
    )
  FOREACH(FILE ${DGTALE_EXAMPLES_IO_Magick})
    add_executable(${FILE} ${FILE})
    target_link_libraries (${FILE} DGtal DGtalIO)
  ENDFOREACH(FILE)
ENDIF(MAGICK++_FOUND)




