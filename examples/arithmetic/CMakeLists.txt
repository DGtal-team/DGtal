set(DGTAL_EXAMPLES_SRC
        approximation
        convergents
        fraction
        pattern
        lower-integer-convex-hull
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries ( ${FILE} DGtal)
endforeach()

if ( WITH_GMP )
  set(DGTAL_EXAMPLES_GMP_SRC
      convergents-biginteger
      extended-euclid
  )
  foreach(FILE ${DGTAL_EXAMPLES_GMP_SRC})
    add_executable(${FILE} ${FILE}.cpp)
    target_link_libraries (${FILE} DGtal)
  endforeach()

endif()
