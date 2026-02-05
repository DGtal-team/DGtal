# ConfigExamples.h instanciation.
configure_file(${PROJECT_SOURCE_DIR}/examples/ConfigExamples.h.in
  ${PROJECT_BINARY_DIR}/examples/ConfigExamples.h)

# For DGtal_add_example function
include(ExampleFunctions.cmake)


#------Examples subdirectories ------

add_subdirectory(images)
add_subdirectory(io)
add_subdirectory(geometry)
add_subdirectory(doc-examples)
add_subdirectory(tutorial-examples)
add_subdirectory(topology)
add_subdirectory(math)
add_subdirectory(arithmetic)
add_subdirectory(shapes)
add_subdirectory(graph)
add_subdirectory(dec)
add_subdirectory(base)
add_subdirectory(kernel)
