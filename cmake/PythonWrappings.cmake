option(DGTAL_WRAP_PYTHON "Compile python wrappings" OFF)
if(DGTAL_WRAP_PYTHON)
  # Fetch pybind11
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v2.9
  )
  FetchContent_GetProperties(pybind11)
  if(NOT pybind11_POPULATED)
    FetchContent_Populate(pybind11)
    add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
  endif()

  option(DGTAL_BUILD_TESTS_PYTHON "Enable python testing" ON)
  if(NOT DGTAL_BUILD_TESTS AND DGTAL_BUILD_TESTS_PYTHON)
    enable_testing()
    include(CTest)
    message(STATUS "Tests for Python-only ENABLED")
  endif()

  add_subdirectory(wrap)
endif()
