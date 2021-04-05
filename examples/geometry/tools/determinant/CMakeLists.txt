set(DGTAL_EXAMPLES_SRC
   exampleInHalfPlane
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE} DGtal  )
endforeach()



