set(DGTAL_EXAMPLES_SRC
  exampleCurvature
  exampleLMST2D
  exampleLMST3D
  exampleLMST3DBy2D
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries (${FILE} DGtal  )
endforeach()
