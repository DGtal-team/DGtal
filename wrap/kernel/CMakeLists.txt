set(module_name_ kernel)
set(module_path_ ${CMAKE_CURRENT_SOURCE_DIR})
set(current_sources_
  ${module_name_}_init.cpp
  PointVector_py.cpp
  )
list(TRANSFORM current_sources_ PREPEND "${module_path_}/")

set(all_modules_python_sources
  ${all_modules_python_sources}
  ${current_sources_}
  PARENT_SCOPE)
