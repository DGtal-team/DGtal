set(DGTAL_EXAMPLES_SRC
        approximation
        convergents
        fraction
        pattern
        lower-integer-convex-hull
        convergents-biginteger
        extended-euclid
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()
