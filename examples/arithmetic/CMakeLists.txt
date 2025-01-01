set(DGTAL_EXAMPLES_SRC
        approximation
        convergents
        fraction
        pattern
        lower-integer-convex-hull
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

if ( DGTAL_WITH_GMP )
  set(DGTAL_EXAMPLES_GMP_SRC
      convergents-biginteger
      extended-euclid
  )
  foreach(FILE ${DGTAL_EXAMPLES_GMP_SRC})
      DGtal_add_example(${FILE})
 endforeach()

endif()
