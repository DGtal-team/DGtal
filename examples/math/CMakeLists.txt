set(DGTAL_EXAMPLES_MATH_SRC
        polynomial-derivative
        polynomial2-derivative
	polynomial-read
)

foreach(FILE ${DGTAL_EXAMPLES_MATH_SRC})
     DGtal_add_example(${FILE})
endforeach()
