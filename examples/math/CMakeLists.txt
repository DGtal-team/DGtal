set(DGTAL_EXAMPLES_MATH_SRC
        polynomial-derivative
        polynomial2-derivative
	polynomial-read
)

foreach(FILE ${DGTAL_EXAMPLES_MATH_SRC})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE} DGtal)
endforeach()
