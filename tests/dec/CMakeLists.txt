set(DGTAL_TESTS_SRC
    testDiscreteExteriorCalculus
    testLinearStructure
    testEmbedding
    testHeatLaplace
    testPolygonalCalculus
    testGeodesicsInHeat
    testVectorsInHeat
  )

# add_test is disabled for the following sources
set(DGTAL_TESTS_SRC_NOTEST
    testDiscreteExteriorCalculusExtended
    )

if(WITH_EIGEN)
  foreach(FILE ${DGTAL_TESTS_SRC})
      DGtal_add_test(${FILE})
  endforeach()

  foreach(FILE ${DGTAL_TESTS_SRC_NOTEST})
      DGtal_add_test(${FILE} ONLY_ADD_EXECUTABLE)
  endforeach()
endif()

