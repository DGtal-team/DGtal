# DGTAL_RANDOMIZED_TESTING_THRESHOLD must be a global variable between 0 and 99

function(DGtal_add_example test_file)
  string(RANDOM LENGTH 2 ALPHABET "0123456789" _random)
  if (${_random} LESS ${DGTAL_RANDOMIZED_TESTING_THRESHOLD} OR ${test_file} IN_LIST DGTAL_RANDOMIZED_TESTING_WHITELIST)
    add_executable(${test_file} ${test_file}.cpp)
    target_link_libraries (${test_file} PRIVATE DGtal ${DGtalLibDependencies})
    target_include_directories(${test_file} PRIVATE ${PROJECT_SOURCE_DIR}/examples/)
    target_include_directories(${test_file} PRIVATE ${PROJECT_BINARY_DIR}/examples/)
  endif()
endfunction()
