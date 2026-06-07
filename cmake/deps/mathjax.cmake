if (TARGET mathjax)
  return()
endif()

include(CPM)

set(CMAKE_CXX_FLAGS_DEBUG_OLD "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG "-w")

CPMAddPackage(
  NAME mathjax
  GIT_TAG 3.2.2
  DOWNLOAD_ONLY TRUE
  GITHUB_REPOSITORY "mathjax/MathJax"
)
