# DGTAL_RANDOMIZED_TESTING_THRESHOLD must be a global variable between 0 and 99

function(DGtal_add_test test_file) #optional_avoid_add_test
  string(RANDOM LENGTH 2 ALPHABET "0123456789" _random)
  cmake_parse_arguments(
    OPTS
    "ONLY_ADD_EXECUTABLE;NO_CATCH"
    ""
    ""
    ${ARGN}
  )

  if(${_random} LESS ${DGTAL_RANDOMIZED_TESTING_THRESHOLD} OR ${test_file} IN_LIST DGTAL_RANDOMIZED_TESTING_WHITELIST)
    add_executable(${test_file} ${test_file}.cpp)
    target_link_libraries(${test_file} PRIVATE DGtal ${DGtalLibDependencies} benchmark::benchmark )
    if (NOT OPTS_NO_CATCH)
      target_link_libraries(${test_file} PRIVATE Catch2::Catch2 benchmark::benchmark)
    endif()
    target_include_directories(${test_file} PRIVATE ${PROJECT_SOURCE_DIR}/tests/)
    target_include_directories(${test_file} PRIVATE ${PROJECT_BINARY_DIR}/tests/)
    if(DGTAL_WARNING_AS_ERROR)
      target_compile_options(${test_file} PRIVATE
        $<$<CONFIG:Debug>:
        -Wall
        -Wno-sign-compare
        -Werror
        -Wno-unknown-pragmas
        -Wshadow
        -Wunused-variable
        -Wunused-parameter
        -Wunused-function
        -Wno-deprecated-copy
        -Werror=type-limits
        -Wno-nonnull
        -Wno-unused-function
        -Wunused
        -Wno-long-long
        -Wno-system-headers
        -Wno-deprecated
        -Woverloaded-virtual
        -Wwrite-strings
        >
      )
    endif()
    if(ONLY_ADD_EXECUTABLE)
      add_test(NAME ${test_file} COMMAND ${test_file})
    endif()
  endif()
endfunction()
