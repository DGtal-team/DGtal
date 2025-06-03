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
  option(DGTAL_BUILD_POLYSCOPE_EXAMPLES "Build polyscope examples." OFF)
  if (DGTAL_BUILD_POLYSCOPE_EXAMPLES )
    message(STATUS "Build polyscope examples ENABLED (DGTAL_BUILD_POLYSCOPE_EXAMPLES)")
  else()
    message(STATUS "Build polyscope examples DISABLED (DGTAL_BUILD_POLYSCOPE_EXAMPLES)")
  endif()
endif()
