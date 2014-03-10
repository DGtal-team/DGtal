
if(WITH_SUITESPARSE)

    set(DGTAL_TESTS_SRC_DEC
        testDiscreteExteriorCalculus
        )

    foreach(FILE ${DGTAL_TESTS_SRC_DEC})
        add_executable(${FILE} ${FILE})
        target_link_libraries (${FILE} DGtal)
        add_test(${FILE} ${FILE})
    endforeach(FILE)

endif(WITH_SUITESPARSE)

