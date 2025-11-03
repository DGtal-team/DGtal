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

include(CMakePrintHelpers)
function(cleanup_target target include_paths)
  get_property(target_include_dir TARGET ${target} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(${target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
  target_include_directories(${target} 
    INTERFACE
    $<BUILD_INTERFACE:${target_include_dir}>
    $<INSTALL_INTERFACE:${DGTAL_INSTALL_DEPS_DESTINATION}/${target}> 
  )
  
  if (NOT ${include_paths})
    foreach(path ${include_paths}) 
      target_include_directories(${target}
        INTERFACE
          $<INSTALL_INTERFACE:${DGTAL_INSTALL_DEPS_DESTINATION}/${target}/${path}> 
      )
    endforeach()
  endif()

  install(TARGETS ${target} EXPORT ${target}Targets)
  export(TARGETS ${target}
    FILE ${target}Targets.cmake
  )
  install(EXPORT ${target}Targets
      FILE ${target}Config.cmake
      DESTINATION ${DGTAL_INSTALL_CMAKE_DESTINATION}
  )
  
  get_property(target_dir TARGET ${target} PROPERTY SOURCE_DIR)
  install(DIRECTORY ${target_dir} DESTINATION ${DGTAL_INSTALL_DEPS_DESTINATION}/${target})
endfunction()

# Polyscope dependencies
# Only imgui have a different structure...
cleanup_target(imgui "imgui/imgui")
cleanup_target(glfw "")
cleanup_target(glad "")
cleanup_target(stb "")
cleanup_target(glm "")
cleanup_target(glm-header-only "")
cleanup_target(nlohmann_json "")
cleanup_target(MarchingCube "")

cleanup_target(polyscope "")
