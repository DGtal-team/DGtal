add_subdirectory(distance)

set(DGTAL_EXAMPLES_SRC
        dvcm-2d
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

