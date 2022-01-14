if (TARGET polyscope)
  return()
endif()

include(FetchContent)

FetchContent_Declare(
    polyscope
    GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
    GIT_TAG        v1.2.0
    GIT_SHALLOW    TRUE
    )
FetchContent_MakeAvailable(polyscope)
