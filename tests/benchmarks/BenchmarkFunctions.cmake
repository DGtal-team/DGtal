function(DGtal_add_benchmark test_file)
    add_executable(${test_file} ${test_file}.cpp)
    target_link_libraries (${test_file} PRIVATE DGtal benchmark::benchmark DGtalCatch Catch2::Catch2)

    target_include_directories(${test_file} PRIVATE ${PROJECT_SOURCE_DIR}/tests/)
    target_include_directories(${test_file} PRIVATE ${PROJECT_BINARY_DIR}/tests/)

    list(APPEND DGTAL_benchmark_exe_list ${test_file})
    set(DGTAL_benchmark_exe_list ${DGtal_benchmark_exe_list} CACHE STRING "" FORCE)

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
endfunction()
