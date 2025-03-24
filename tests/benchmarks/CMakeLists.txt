#CMakeLists associated to the benchmarks subdir
#Bastien Doignies <bastien.doignies@liris.cnrs.fr>

# List of commands to run
set(DGTAL_benchmark_exe_list "" CACHE STRING "List of benchmarks target")
include(BenchmarkFunctions.cmake)


#------TESTS subdirectories ------
add_subdirectory(base)
add_subdirectory(kernel)
# add_subdirectory(math)
add_subdirectory(arithmetic)
    # add_subdirectory(geometry)
add_subdirectory(graph)
add_subdirectory(topology)
# add_subdirectory(io)
add_subdirectory(images)
# add_subdirectory(helpers)
# add_subdirectory(shapes)
# add_subdirectory(dec)

# Run all benchmarks. Can not be called 'benchmarks' 
# because it conflicts with the library
add_custom_target(dgtal_benchmark
    COMMAND ${DGTAL_benchmark_exe_list}
    DEPENDS ${DGTAL_benchmark_exe_list}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)