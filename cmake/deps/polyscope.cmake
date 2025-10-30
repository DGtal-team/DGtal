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
      $<INSTALL_INTERFACE:include/${target}> 
      $<INSTALL_INTERFACE:include> 
  )

  if (NOT DEFINED ho)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${target_include_dir}>
        $<INSTALL_INTERFACE:include/${target}> 
        $<INSTALL_INTERFACE:include> 
    )
  endif()

  install(TARGETS ${target} EXPORT ${target}Targets)
  export(TARGETS ${target}
    FILE ${target}Targets.cmake
  )
  install(EXPORT ${target}Targets
      FILE ${target}Config.cmake
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake
  )

  get_property(includes TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
  install(DIRECTORY ${includes} DESTINATION include/${target})
endfunction()

# glm does not have an include_directories by default when pulled through polyscope
# we instead copy source dir wich is the correct location
get_property(glm_dir TARGET glm PROPERTY SOURCE_DIR)
set_target_properties(glm PROPERTIES INCLUDE_DIRECTORIES ${glm_dir})
set_target_properties(glm-header-only PROPERTIES INCLUDE_DIRECTORIES ${glm_dir})

# Polyscope dependencies
cleanup_target(imgui OFF)
cleanup_target(glfw OFF)
cleanup_target(glad OFF)
cleanup_target(stb OFF)
cleanup_target(glm OFF)
cleanup_target(glm-header-only ON)
cleanup_target(nlohmann_json OFF)
cleanup_target(MarchingCube OFF)

# GLM has a special setup fix it here !

cleanup_target(polyscope OFF)
