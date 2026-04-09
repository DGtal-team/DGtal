if (TARGET mathjax)
  return()
endif()

set(CMAKE_CXX_FLAGS_DEBUG_OLD "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG "-w")

CPMAddPackage(
  NAME mathjax
  GIT_TAG 4.1.1
  DOWNLOAD_ONLY TRUE
  GITHUB_REPOSITORY "mathjax/MathJax"
)
