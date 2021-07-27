# THRESHOLD_RANDOM_TESTING must be a global variable between 0 and 99

function(DGtal_add_example test_file) #optional_avoid_add_test
  string(RANDOM LENGTH 2 ALPHABET "0123456789" _random)
  if (${_random} LESS ${THRESHOLD_RANDOM_TESTING})
    add_executable(${test_file} ${test_file}.cpp)
    target_link_libraries (${test_file} PRIVATE DGtal)
    target_include_directories(${test_file} PRIVATE ${PROJECT_SOURCE_DIR}/examples/)
    target_include_directories(${test_file} PRIVATE ${PROJECT_BINARY_DIR}/examples/)
  endif()
endfunction()
