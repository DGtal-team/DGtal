option(DGTAL_WRAP_PYTHON "Compile python wrappings" OFF)
if(DGTAL_WRAP_PYTHON)
  # Fetch nanobind
  include(FetchContent)
  FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind
    GIT_TAG v2.2.0
  )
  FetchContent_GetProperties(nanobind)
  if(NOT nanobind_POPULATED)
    FetchContent_Populate(nanobind)
    add_subdirectory(${nanobind_SOURCE_DIR} ${nanobind_BINARY_DIR})
  endif()

  option(DGTAL_BUILD_TESTS_PYTHON "Enable python testing" ON)
  if(NOT DGTAL_BUILD_TESTS AND DGTAL_BUILD_TESTS_PYTHON)
    enable_testing()
    include(CTest)
    message(STATUS "Tests for Python-only ENABLED")
  endif()

  add_subdirectory(wrap)
endif()
