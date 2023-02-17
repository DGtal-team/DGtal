if (TARGET ZLIB)
  return()
endif()

message(STATUS "    ZLIB (v1.2.13) creating target 'ZLIB'")

include(FetchContent)

FetchContent_Declare(
    ZLIB
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG        v1.2.13
    GIT_SHALLOW    TRUE
    )
FetchContent_MakeAvailable(ZLIB)
