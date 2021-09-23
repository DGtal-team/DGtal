#include <iostream>
#include <string>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/geometry/surfaces/DigitalSurfaceRegularization.h>

#include <DGtal/dec/PolygonalCalculus.h>
#include <DGtal/dec/GeodesicsInHeat.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>

#include "ConfigExamples.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;
typedef SurfMesh::Face                   Face;
typedef SurfMesh::Vertex                  Vertex;

//Polyscope global
polyscope::SurfaceMesh *psMesh;
SurfMesh surfmesh;
float dt = 2.0;

GeodesicsInHeat<PolygonalCalculus<SurfMesh>> *heat;
PolygonalCalculus<SurfMesh> *calculus;

void precompute()
{
  calculus = new PolygonalCalculus<SurfMesh>(surfmesh);
  heat = new GeodesicsInHeat<PolygonalCalculus<SurfMesh>>(calculus);
  
  trace.beginBlock("Init solvers");
  heat->init(dt);
  trace.endBlock();
}


void addsource()
{
  heat->addSource( 52889 );
  heat->addSource( rand() % surfmesh.nbVertices());
  GeodesicsInHeat<PolygonalCalculus<SurfMesh>>::Vector source = heat->source();
  psMesh->addVertexScalarQuantity("source", source);

}

void computeGeodisc()
{
  GeodesicsInHeat<PolygonalCalculus<SurfMesh>>::Vector dist = heat->compute();
  psMesh->addVertexDistanceQuantity("geodesic", dist);
}

void myCallback()
{
  ImGui::SliderFloat("dt", &dt, 0.,4.);
  if(ImGui::Button("Precomputation (required if you change the dt)"))
    precompute();
  
  if(ImGui::Button("Add random source"))
    addsource();
  
  if(ImGui::Button("Compute geodesic"))
    computeGeodisc();
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();


  params("surfaceComponents", "All");

  std::string filename = examplesPath + std::string("/samples/bunny-128.vol");
  
  auto binary_image = SH3::makeBinaryImage(filename, params );
  auto K               = SH3::getKSpace( binary_image, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  SH3::Cell2Index c2i;
  auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);
  
  //Need to convert the faces
  std::vector<std::vector<unsigned long>> faces;
  std::vector<RealPoint> positions;
  
  for(auto face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));
  
  //Recasting to vector of vertices
  positions = primalSurface->positions();

  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());

  std::cout<<"number of non-manifold Edges = " << surfmesh.computeNonManifoldEdges().size()<<std::endl;
  
  // Initialize polyscope
  polyscope::init();

  psMesh = polyscope::registerSurfaceMesh("digital surface", positions, faces);

  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
