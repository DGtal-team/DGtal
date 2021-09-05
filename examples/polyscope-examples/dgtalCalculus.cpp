#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/geometry/surfaces/DigitalSurfaceRegularization.h>
#include <DGtal/dec/PolygonalCalculus.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>


#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;
typedef SurfMesh::Vertices                    Vertices;
typedef SurfMesh::RealPoint                   RealPoint;
typedef SurfMesh::Face                   Face;
typedef SurfMesh::Vertex                  Vertex;


//Polyscope global
float dt=2.0;
// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh;
SurfMesh surfmesh;
std::vector<double> phiV;
float scale = 0.1;

//Restriction of a scalar function to vertices
double phiVertex(const Vertex v)
{
  return  cos(scale*(surfmesh.position(v)[0]))*sin(scale*surfmesh.position(v)[1]);
}

//Restriction of a scalar function to vertices
PolygonalCalculus<SurfMesh>::Vector phi(const Face f)
{
  auto vertices = surfmesh.incidentVertices(f);
  auto nf = vertices.size();
  Eigen::VectorXd ph(nf);
  size_t cpt=0;
  for(auto v: vertices)
  {
    ph(cpt) =  phiVertex(v);
    ++cpt;
  }
  return  ph;
}


void initPhi()
{
  phiV.clear();
  for(auto i = 0; i < surfmesh.nbVertices(); ++i)
    phiV.push_back(phiVertex(i));
  psMesh->addVertexScalarQuantity("Phi", phiV);
}

void initQuantities()
{
  PolygonalCalculus<SurfMesh> calculus(surfmesh);

  std::vector<PolygonalCalculus<SurfMesh>::Vector> gradients;
  std::vector<PolygonalCalculus<SurfMesh>::Vector> cogradients;
  std::vector<PolygonalCalculus<SurfMesh>::RealPoint> normals;
  for(auto f=0; f < surfmesh.nbFaces(); ++f)
  {
    auto ph = phi(f);
    auto grad = calculus.gradient(f) * ph;
    gradients.push_back( grad );
    auto cograd =  calculus.coGradient(f) * phi;
    cogradients.push_back( cograd );
    normals.push_back(calculus.correctedFaceNormalAsDGtalVector(f));
  }
  psMesh->addFaceVectorQuantity("Gradient", gradients);
  psMesh->addFaceVectorQuantity("co-Gradient", cogradients);
  psMesh->addFaceVectorQuantity("Normals", normals);
}


void myCallback()
{
  ImGui::SliderFloat("Phi scale", &scale, 0., 1.);
  if (ImGui::Button("Init phi"))
  {
    initPhi();
  }
  
  if (ImGui::Button("Compute quantities"))
    initQuantities();
  
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  
  auto h=.3 ; //gridstep
  params( "polynomial", "goursat" )( "gridstep", h );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto embedder        = SH3::getCellEmbedder( K );
  SH3::Cell2Index c2i;
  auto surfels         = SH3::getSurfelRange( surface, params );
  auto primalSurface   = SH3::makePrimalPolygonalSurface(c2i, surface);
  
  // Convert faces to appropriate indexed format
  std::vector<std::vector<unsigned long>> faces;
  for(auto &face: primalSurface->allFaces())
    faces.push_back(primalSurface->verticesAroundFace( face ));
  
  //Recasting to vector of vertices
  auto pos = primalSurface->positions();
  std::vector<RealPoint> positions(primalSurface->nbVertices());
  for(auto i=0; i < primalSurface->nbVertices(); ++i)
    positions[i] = pos(i);
  
  
  
  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());
  
  
  
  // Initialize polyscope
  polyscope::init();
  
  psMesh = polyscope::registerSurfaceMesh("digital surface", positions, faces);
  
  // Set the callback function
  polyscope::state::userCallback = myCallback;
  
  
  
  polyscope::show();
  return EXIT_SUCCESS;
  
}
