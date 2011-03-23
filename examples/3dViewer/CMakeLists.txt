SET(DGTAL_TESTS_SRC
   dgtalQGLviewer-1-points
   dgtalQGLviewer-2-sets
   dgtalQGLviewer-3-objects
   dgtalQGLviewer-5-custom
   dgtalQGLviewer-6-clipping
)

FOREACH(FILE ${DGTAL_TESTS_SRC})
  add_executable(${FILE} ${FILE})
  target_link_libraries (${FILE} ${LIBDGTALVISU3D_NAME} ${LIBDGTAL_NAME})
ENDFOREACH(FILE)

