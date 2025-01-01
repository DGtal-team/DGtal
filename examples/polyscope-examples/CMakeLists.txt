
# -----------------------------------------------------------------------------
# Fetching polyscope (only if the WITH_POLYSCOPE_EXAMPLES Variable has been set to true)
# -----------------------------------------------------------------------------
if (DGTAL_BUILD_POLYSCOPE_EXAMPLES)
  message(STATUS "    polyscope (v2.3.0)")
  include(polyscope)
endif()

if(BUILD_SHARED_LIBS)
  message(FATAL_ERROR "Polyscope examples require static build of the DGtal. Please set BUILD_SHARED_LIBS variable to OFF")
endif()

# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/src/)
include_directories (${PROJECT_BINARY_DIR}/src)

# Make sure the compiler can find include files.
include_directories (${PROJECT_SOURCE_DIR}/examples/)
include_directories (${PROJECT_BINARY_DIR}/examples/)


# Make sure the linker can find the Hello library once it is built.
link_directories (${PROJECT_BINARY_DIR})

# ConfigExamples.h instanciation.
configure_file(${PROJECT_SOURCE_DIR}/examples/ConfigExamples.h.in
  ${PROJECT_BINARY_DIR}/examples/ConfigExamples.h)

set(POLYSCOPE_SRCS
  dgtalCalculus
  dgtalCalculus-single
  dgtalCalculus-poisson
  dgtalCalculus-bunny
  dgtalCalculus-halfsphere
  dgtalCalculus-geodesic
  exampleVectorHeatMethod
  exampleHarmonicParametrization
  exampleBunnyHead
  tangency-explorer
  tangency-reconstruction
)

if(DGTAL_WITH_LIBIGL)
 set(POLYSCOPE_SRCS  
      ${POLYSCOPE_SRCS} 
      windingNumbersShape
 )
endif()


if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"  OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(MY_DEBUG_OPTIONS "-w")
endif()

foreach(FILE ${POLYSCOPE_SRCS})
  #We should be using `DGtal_add_example(${FILE} polyscope)` to
  #enable the randomized testing but it does not work (polyscope headers are
  #not found properly).
  add_executable(${FILE} ${FILE}.cpp)
  target_link_libraries(${FILE} DGtal ${DGtalLibDependencies} polyscope Eigen3::Eigen)
  target_compile_options(${FILE} PUBLIC "$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")
endforeach()
