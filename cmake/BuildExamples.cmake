option(DGTAL_BUILD_EXAMPLES "Build examples." OFF)
if (DGTAL_BUILD_EXAMPLES)
  message(STATUS "Build examples ENABLED")
  add_subdirectory (${PROJECT_SOURCE_DIR}/examples)
else()
  message(STATUS "Build examples DISABLED (you can activate the examples with '-DDGTAL_BUILD_EXAMPLES=ON' cmake option)")
endif()
message(STATUS "-------------------------------------------------------------------------------")

# -----------------------------------------------------------------------------
# Specific examples
# -----------------------------------------------------------------------------
if (DGTAL_BUILD_EXAMPLES)
  # -----------------------------------------------------------------------------
  # polyscope examples
  # -----------------------------------------------------------------------------
  if (DGTAL_WITH_POLYSCOPE_VIEWER)
    message(STATUS "Build polyscope examples ENABLED (DGTAL_WITH_POLYSCOPE_VIEWER)")
  else()
    message(STATUS "Build polyscope examples DISABLED (DGTAL_WITH_POLYSCOPE_VIEWER)")
  endif()
endif()
message(STATUS "-------------------------------------------------------------------------------")
