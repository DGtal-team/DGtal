add_subdirectory(curves)
add_subdirectory(volumes)
add_subdirectory(surfaces)
add_subdirectory(tools)
add_subdirectory(meshes)

set(DGTAL_TESTS_SRC
  testContourHelper)

foreach(FILE ${DGTAL_TESTS_SRC})
  DGtal_add_test(${FILE})
endforeach()
