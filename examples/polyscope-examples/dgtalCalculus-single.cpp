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
  std::vector<PolygonalCalculus<SurfMesh>::RealPoint> vectorArea;
  std::vector<PolygonalCalculus<SurfMesh>::RealPoint> centroids;

  std::vector<double> faceArea;
  std::vector<double> d0(surfmesh.nbEdges());

  for(auto f=0; f < surfmesh.nbFaces(); ++f)
  {
    auto ph = phi(f);
    auto grad = calculus.gradient(f) * ph;
    gradients.push_back( grad );
    auto cograd =  calculus.coGradient(f) * ph;
    cogradients.push_back( cograd );
    normals.push_back(calculus.correctedFaceNormalAsDGtalVector(f));
    
    auto vA = calculus.correctedVectorArea(f);
    vectorArea.push_back({vA(0) , vA(1), vA(2)});
    
    faceArea.push_back( calculus.correctedFaceArea(f));
    
    centroids.push_back( calculus.centroidAsDGtalPoint(f) );
  }
  
  psMesh->addFaceVectorQuantity("Gradients", gradients);
  psMesh->addFaceVectorQuantity("co-Gradients", cogradients);
  psMesh->addFaceVectorQuantity("Normals", normals);
  psMesh->addFaceScalarQuantity("Face area", faceArea);
  psMesh->addFaceVectorQuantity("Vector area", vectorArea);
  psMesh->addEdgeScalarQuantity("d0*phi", d0);
  
  polyscope::registerPointCloud("Centroids", centroids);
}


void myCallback()
{
  ImGui::SliderFloat("Phi scale", &scale, 0., 1.);
  if (ImGui::Button("Init phi"))
    initPhi();
  
  if (ImGui::Button("Compute quantities"))
    initQuantities();
  
}

int main()
{
  std::vector<RealPoint> positions={ {0,0,0},{20,0,0},{20,10,0},{10,8,5}, {0,15,1} };
  std::vector<std::vector<size_t>> faces={{ 0,1,2,3,4}};

  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());
  
    
  // Initialize polyscope
  polyscope::init();
  
  psMesh = polyscope::registerSurfaceMesh("Single face", positions, faces);

  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
  
}
