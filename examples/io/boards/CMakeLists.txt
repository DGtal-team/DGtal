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
    dgtalBoard3DTo2D-0-demo-kernel-2
    dgtalBoard3DTo2D-1-points
    dgtalBoard3DTo2D-2-sets
    dgtalBoard3DTo2D-2bis-sets
    dgtalBoard3DTo2D-3-objects
    dgtalBoard3DTo2D-4-modes
    dgtalBoard3DTo2D-5-custom
    dgtalBoard3DTo2D-6
)
endif ( WITH_CAIRO )


FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)


