set(DGTAL_EXAMPLES_SRC
  exampleCurvature
  exampleLMST2D
  exampleLMST3D
  exampleLMST3DBy2D
)


foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()
