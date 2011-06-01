SET(DGTAL_TESTS_SRC
	testArithDSS3d
	)

FOREACH(FILE ${DGTAL_TESTS_SRC})
	add_executable(${FILE} ${FILE})
	target_link_libraries (${FILE} ${LIBDGTAL_NAME})
	add_test(${FILE} ${FILE})
ENDFOREACH(FILE)
