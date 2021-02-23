function(DGtal_add_test test_file) #optional_avoid_add_test
  add_executable(${test_file} ${test_file})
  target_link_libraries (${test_file} PRIVATE DGtal Catch2::Catch2 DGtalCatch)
  target_include_directories(${test_file} PRIVATE ${PROJECT_SOURCE_DIR}/tests/)
  target_include_directories(${test_file} PRIVATE ${PROJECT_BINARY_DIR}/tests/)
  if(NOT ${ARGC} GREATER 1)
    add_test(${test_file} ${test_file})
  endif()
endfunction()
