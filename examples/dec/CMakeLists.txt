if (WITH_CAIRO AND WITH_EIGEN AND WITH_QGLVIEWER)

    set(DGTAL_EXAMPLES_SRC_DEC
        exampleDECSurface
        exampleDiscreteExteriorCalculusUsage
        exampleDiscreteExteriorCalculusSolve
        exampleDiscreteExteriorCalculusChladni
        exampleHeatLaplace
        )

    foreach(FILE ${DGTAL_EXAMPLES_SRC_DEC})
      DGtal_add_example(${FILE})
    endforeach()

endif()

if (WITH_EIGEN)
    set(DGTAL_EXAMPLES_SRC2_DEC
        examplePropagation
        exampleSurfaceATNormals
        )

    foreach(FILE ${DGTAL_EXAMPLES_SRC2_DEC})
     DGtal_add_example(${FILE})
    endforeach()
endif()
