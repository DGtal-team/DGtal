if (TARGET ponca)
  return()
endif()

CPMAddPackage(
  NAME ponca
  VERSION 1.3
  GITHUB_REPOSITORY "poncateam/ponca"
  SYSTEM TRUE
)

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
    Eigen3_Eigen
    NAMESPACE Ponca::
    FILE PoncaTargets.cmake
)

