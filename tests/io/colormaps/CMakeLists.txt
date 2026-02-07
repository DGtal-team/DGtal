set(DGTAL_TESTS_SRC_COLORMAP
   testTickedColorMap
   testColorMaps
   )


foreach(FILE ${DGTAL_TESTS_SRC_COLORMAP})
  DGtal_add_test(${FILE})
endforeach()
