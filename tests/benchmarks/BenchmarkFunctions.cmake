function(DGtal_add_benchmark test_file)
    add_executable(${test_file} ${test_file}.cpp)
    target_link_libraries (${test_file} PRIVATE DGtal benchmark::benchmark DGtalCatch Catch2::Catch2)

    target_include_directories(${test_file} PRIVATE ${PROJECT_SOURCE_DIR}/tests/)
    target_include_directories(${test_file} PRIVATE ${PROJECT_BINARY_DIR}/tests/)

    list(APPEND DGtal_benchmark_exe_list ${test_file})
    set(DGtal_benchmark_exe_list ${DGtal_benchmark_exe_list} CACHE STRING "" FORCE)
endfunction()
