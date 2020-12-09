# Useful to trim down the size of the dependencies build folder
# Used in docker images.
if(NOT OUTPUT_BUILD_DIR)
  message(FATAL_ERROR "Provide -DOUTPUT_BUILD_DIR to this script pointing to the BINARY_DIR")
endif()
# ITK
file(REMOVE_RECURSE ${OUTPUT_BUILD_DIR}/ep_itk-prefix)
file(REMOVE_RECURSE ${ITK_SRC_DIR}/.git)
# Boost
file(REMOVE_RECURSE ${OUTPUT_BUILD_DIR}/ep_boost-prefix)

file(GLOB compilation_objects ${OUTPUT_BUILD_DIR}/*.o ${OUTPUT_BUILD_DIR}/*.obj)
if(compilation_objects)
  file(REMOVE ${compilation_objects})
  message(STATUS "compilation_objects removed")
endif()

