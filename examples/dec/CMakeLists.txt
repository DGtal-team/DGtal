if (WITH_VISU3D_QGLVIEWER AND WITH_SUITESPARSE AND WITH_ITK)

    set(DGTAL_EXAMPLES_SRC_DEC
        exampleUsage
        )

    foreach(FILE ${DGTAL_EXAMPLES_SRC_DEC})
        add_executable(${FILE} ${FILE})
        target_link_libraries (${FILE} DGtal DGtalIO)
    endforeach(FILE)

endif(WITH_VISU3D_QGLVIEWER AND WITH_SUITESPARSE AND WITH_ITK)
