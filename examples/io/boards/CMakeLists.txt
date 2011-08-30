SET(DGTAL_TESTS_SRC
   dgtalboard-1-points
   dgtalboard-2-sets
   dgtalboard-3-custom-classes
   dgtalboard-3-custom-points
   dgtalboard-4-colormaps
   )


if( WITH_CAIRO )
  SET(DGTAL_TESTS_SRC
    ${DGTAL_TESTS_SRC}
    dgtalCairo-0-demo-kernel-2
    dgtalCairo-1-points
    dgtalCairo-2-sets
    dgtalCairo-2bis-sets
    dgtalCairo-3-objects
    dgtalCairo-4-modes
    dgtalCairo-5-custom
    dgtalCairo-6
)
endif ( WITH_CAIRO )


FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)


