option(DGTAL_WRAP_PYTHON "Compile python wrappings" OFF)
if(DGTAL_WRAP_PYTHON)
  # Find Python first (required by nanobind)
  find_package(Python COMPONENTS Interpreter Development.Module Development.SABIModule REQUIRED)
  
  # Fetch nanobind
  include(FetchContent)
  FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind
    GIT_TAG v2.2.0
  )
  FetchContent_MakeAvailable(nanobind)

  option(DGTAL_BUILD_TESTS_PYTHON "Enable python testing" ON)
  if(NOT DGTAL_BUILD_TESTS AND DGTAL_BUILD_TESTS_PYTHON)
    enable_testing()
    include(CTest)
    message(STATUS "Tests for Python-only ENABLED")
  endif()

  add_subdirectory(wrap)
endif()
