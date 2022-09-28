Include(FetchContent)

## https://cliutils.gitlab.io/modern-cmake/chapters/projects/fetch.html
if(${CMAKE_VERSION} VERSION_LESS 3.14)
  macro(FetchContent_MakeAvailable NAME)
        string(TOLOWER ${NAME} lcNAME)
        FetchContent_GetProperties(${NAME})
        if(NOT ${lcNAME}_POPULATED)
            FetchContent_Populate(${NAME})
            add_subdirectory(${${lcNAME}_SOURCE_DIR} ${${lcNAME}_BINARY_DIR})
        endif()
    endmacro()
  endif()


# -----------------------------------------------------------------------------
# Fetching Catch2 and googlebenchmark
# (only if the BUILD_TESTING variable has been set to true)
# -----------------------------------------------------------------------------
if (BUILD_TESTING)

  message(STATUS "    Catch2 (v2.13.7)")
  include(catch2)
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
  include(CTest)
  include(Catch)

  message(STATUS "    Google benchmark (v1.6.1)")
  include(googlebenchmark)
endif()

# -----------------------------------------------------------------------------
# Fetching Eigen3
# -----------------------------------------------------------------------------
include(eigen)
