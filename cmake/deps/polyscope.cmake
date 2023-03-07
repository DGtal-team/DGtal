if (TARGET polyscope)
  return()
endif()

include(FetchContent)

set(CMAKE_CXX_FLAGS_DEBUG_OLD "${CMAKE_CXX_FLAGS_DEBUG}")


FetchContent_Declare(
    polyscope
    GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
    GIT_TAG        v1.2.0
    GIT_SHALLOW    TRUE
    )

set(CMAKE_CXX_FLAGS_DEBUG "-w")

FetchContent_MakeAvailable(polyscope)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG_OLD}")
