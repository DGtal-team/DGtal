
if(WITH_EIGEN)

    add_executable(testDiscreteExteriorCalculus testDiscreteExteriorCalculus)
    target_link_libraries(testDiscreteExteriorCalculus DGtal)
    add_test(testDiscreteExteriorCalculus testDiscreteExteriorCalculus)

    add_executable(testDiscreteExteriorCalculusExtended testDiscreteExteriorCalculusExtended)
    target_link_libraries(testDiscreteExteriorCalculusExtended DGtal)
    #add_test(testDiscreteExteriorCalculusExtended testDiscreteExteriorCalculusExtended)

endif(WITH_EIGEN)

