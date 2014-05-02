if (WITH_CAIRO AND WITH_EIGEN AND WITH_QGLVIEWER)

    set(DGTAL_EXAMPLES_SRC_DEC
        exampleDiscreteExteriorCalculusUsage
        exampleDiscreteExteriorCalculusSolve
        exampleDiscreteExteriorCalculusChladni
        )

    foreach(FILE ${DGTAL_EXAMPLES_SRC_DEC})
        add_executable(${FILE} ${FILE})
        target_link_libraries (${FILE} DGtal DGtalIO)
    endforeach(FILE)

endif(WITH_CAIRO AND WITH_EIGEN AND WITH_QGLVIEWER)
