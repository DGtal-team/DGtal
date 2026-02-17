set(DGTAL_EXAMPLES_SRC
        curvature-comparator-ii-cnc-3d
        curvature-measures-icnc-3d
        curvature-measures-icnc-XY-3d
        curvature-measures-nc-3d
        curvature-measures-nc-XY-3d
        obj-curvature-measures-icnc-3d
        obj-curvature-measures-icnc-XY-3d
        vol-curvature-measures-icnc-3d
        vol-curvature-measures-icnc-XY-3d
        digpoly-curvature-measures-cnc-3d
        digpoly-curvature-measures-cnc-XY-3d
)

foreach(FILE ${DGTAL_EXAMPLES_SRC})
     DGtal_add_example(${FILE})
endforeach()
