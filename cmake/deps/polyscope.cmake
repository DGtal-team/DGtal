if (TARGET polyscope)
  return()
endif()

set(CMAKE_CXX_FLAGS_DEBUG_OLD "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG "-w")

CPMAddPackage(
  NAME polyscope
  VERSION 2.4.0
  SYSTEM TRUE
  GITHUB_REPOSITORY "nmwsharp/polyscope"
)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG_OLD}")

# Polyscope uses header-only targets for which INTERFACE_INCLUDE_DIRECTORIES
# points to build dir. This is forbidden by CMake (in install/export).
# This function cleans INTERFACE_INCLUDE_DIRECTORIES to use generator expression
# instead. It also provide the necessary install and exports.
function(cleanup_target target ho)
  get_property(target_include_dir TARGET ${target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  
  set_target_properties(${target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
  target_include_directories(${target} 
    INTERFACE
    $<BUILD_INTERFACE:${target_include_dir}>
    $<INSTALL_INTERFACE:${DGTAL_INSTALL_DEPS_DESTINATION}/${target}> 
  )

  if (NOT DEFINED ho)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${target_include_dir}>
        $<INSTALL_INTERFACE:${DGTAL_INSTALL_DEPS_DESTINATION}/${target}> 
    )
  endif()

  install(TARGETS ${target} EXPORT ${target}Targets)
  export(TARGETS ${target}
    FILE ${target}Targets.cmake
  )
  install(EXPORT ${target}Targets
      FILE ${target}Config.cmake
      DESTINATION ${DGTAL_INSTALL_CMAKE_DESTINATION}
  )
  
  get_property(includes TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
  set(existring_include_dirs)
  foreach(file in ${includes})
    if (EXISTS ${file})
      list(APPEND existing_include_dirs ${file})
    endif()
  endforeach()
  message(STATUS "${target}: ${existing_include_dirs}")
  install(DIRECTORY ${existing_include_dirs} DESTINATION ${DGTAL_INSTALL_DEPS_DESTINATION}/${target})
endfunction()

# glm does not have an include_directories by default when pulled through polyscope
# we instead copy source dir wich is the correct location
get_property(glm_dir TARGET glm PROPERTY SOURCE_DIR)
set_target_properties(glm PROPERTIES INCLUDE_DIRECTORIES ${glm_dir})

# Polyscope dependencies
cleanup_target(imgui OFF)
cleanup_target(glfw OFF)
cleanup_target(glad OFF)
cleanup_target(stb OFF)
cleanup_target(glm OFF)
cleanup_target(glm-header-only ON)
cleanup_target(nlohmann_json OFF)
cleanup_target(MarchingCube OFF)

cleanup_target(polyscope OFF)
