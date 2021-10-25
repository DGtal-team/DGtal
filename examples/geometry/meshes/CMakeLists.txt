set(DGTAL_EXAMPLES_SRC
        curvature-measures-icnc-3d
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

