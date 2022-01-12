option(BUILD_EXAMPLES "Build examples." OFF)
if (BUILD_EXAMPLES)
  message(STATUS "Build examples ENABLED")
  add_subdirectory (${PROJECT_SOURCE_DIR}/examples)
else()
  message(STATUS "Build examples DISABLED (you can activate the examples with '-DBUILD_EXAMPLES=ON' cmake option)")
endif()
message(STATUS "-------------------------------------------------------------------------------")



# -----------------------------------------------------------------------------
# polyscope examples
# -----------------------------------------------------------------------------
option(BUILD_POLYSCOPE_EXAMPLES "Build polyscope examples." OFF)
if (BUILD_POLYSCOPE_EXAMPLES )
  message(STATUS "Build polyscope examples ENABLED (BUILD_POLYSCOPE_EXAMPLES)")
  add_subdirectory (${PROJECT_SOURCE_DIR}/examples/polyscope-examples)
else()
  message(STATUS "Build polyscope examples DISABLED (BUILD_POLYSCOPE_EXAMPLES)")
endif()

