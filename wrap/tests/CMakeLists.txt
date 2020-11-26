set(pytest_options
  "-rA" # verbose, show all output with ctest -V
  "-v"  # assert shows full diff when there is an error
  )

add_subdirectory(kernel)
add_subdirectory(topology)
add_subdirectory(images)
