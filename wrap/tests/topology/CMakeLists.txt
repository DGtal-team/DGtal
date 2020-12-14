set(python_tests_
  test_KhalimskyPreSpaceND.py
  test_KhalimskySpaceND.py
  test_MetricAdjacency.py
  test_DigitalTopology.py
  test_Object.py
  test_CubicalComplex.py
  )

get_filename_component(module_name_ ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(test_folder "${CMAKE_CURRENT_SOURCE_DIR}")
# test files should start with "test_"
# unittest functions (in .py) should start with "test_" for discover to work
foreach(python_test ${python_tests_})
  set(python_test_name_ python||${module_name_}||${python_test})
  add_test(NAME ${python_test_name_}
    COMMAND
    ${PYTHON_EXECUTABLE}
    -m pytest
    # Verbose, show all output with ctest -V
    ${pytest_options}
    ${test_folder}/${python_test}
    # Execute the tests from the right directory to allow `ìmport dgtal` to work
    WORKING_DIRECTORY "${CMAKE_BUILD_PYTHONLIBDIR}/.."
    )
endforeach()
