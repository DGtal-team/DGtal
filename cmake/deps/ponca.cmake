if (TARGET ponca)
  return()
endif()

include(CPM)
CPMAddPackage(
  NAME ponca
  VERSION 1.4
  GITHUB_REPOSITORY "poncateam/ponca"
  SYSTEM TRUE
  OPTIONS
    "PONCA_CONFIGURE_TESTS OFF"
)

# DGtal links Eigen itself; keep Ponca's Eigen dependency out of build-tree exports.
foreach(_ponca_target Fitting SpatialPartitioning)
  if(TARGET "${_ponca_target}")
    get_target_property(_ponca_link_libraries "${_ponca_target}" INTERFACE_LINK_LIBRARIES)
    if(_ponca_link_libraries)
      list(REMOVE_ITEM _ponca_link_libraries Eigen3::Eigen Eigen3_Eigen)
      set_target_properties("${_ponca_target}" PROPERTIES
        INTERFACE_LINK_LIBRARIES "${_ponca_link_libraries}"
      )
    endif()
  endif()
endforeach()

# Create a custom target because Fitting collides with boost::Fitting...
add_library(Ponca INTERFACE)
target_link_libraries(Ponca INTERFACE Ponca::Fitting)
add_library(Ponca::Ponca ALIAS Ponca)

# Install / export ponca targets
install(DIRECTORY ${PONCA_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DGtal/3rdParties/)
install(TARGETS Ponca Fitting EXPORT PoncaFitting)
install(EXPORT PoncaFitting DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/boost NAMESPACE Ponca::)

export(TARGETS
    Ponca
    Fitting
    NAMESPACE Ponca::
    FILE PoncaTargets.cmake
)
