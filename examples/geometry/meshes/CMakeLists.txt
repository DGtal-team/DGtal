set(DGTAL_EXAMPLES_SRC
        curvature-measures-icnc-3d
        curvature-measures-icnc-XY-3d
        curvature-measures-nc-3d
        curvature-measures-nc-XY-3d
        obj-curvature-measures-icnc-3d
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()

