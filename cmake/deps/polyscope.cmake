if (TARGET polyscope)
  return()
endif()

set(CMAKE_CXX_FLAGS_DEBUG_OLD "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG "-w")

CPMAddPackage(
  NAME polyscope
  VERSION 2.4.0
  GITHUB_REPOSITORY "nmwsharp/polyscope"
  SYSTEM TRUE
)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG_OLD}")

# Polyscope uses header-only targets for which INTERFACE_INCLUDE_DIRECTORIES
# points to build dir. This is forbidden by CMake (in install/export).
# This function cleans INTERFACE_INCLUDE_DIRECTORIES to use generator expression
# instead. It also provide the necessary install and exports  
function(cleanup_target target)
  get_property(target_include_dir TARGET ${target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  
  set_target_properties(${target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
  target_include_directories(${target} INTERFACE
      $<BUILD_INTERFACE:${target_include_dir}> 
      $<INSTALL_INTERFACE:include> 
  )
  get_property(test TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)

  install(TARGETS ${target} EXPORT ${target}Targets)
  install(EXPORT ${target}Targets
      FILE ${target}Config.cmake
      DESTINATION lib/cmake/${target}
  )
  export(TARGETS ${target}
      FILE ${target}Config.cmake
  )
endfunction()

# Polyscope dependencies
cleanup_target(imgui)
cleanup_target(glfw)
cleanup_target(glad)
cleanup_target(stb)
cleanup_target(glm)
cleanup_target(glm-header-only)
cleanup_target(nlohmann_json)
cleanup_target(MarchingCube)

cleanup_target(polyscope)
