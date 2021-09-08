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
#include <polyscope/curve_network.h>

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
polyscope::PointCloud *psVertices;
polyscope::CurveNetwork *psBoundary;

SurfMesh surfmesh;
PolygonalCalculus<SurfMesh>::Vector phiEigen;


void initPhi()
{
  phiEigen.resize(5);
  phiEigen << 1.0, 2.0, 0.0, 5.0 ,1.5;
  
  psMesh->addVertexScalarQuantity("Phi", phiEigen);
  psVertices->addScalarQuantity("Phi", phiEigen);
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

  PolygonalCalculus<SurfMesh>::Face f = 0; //Id of the face
 
  auto grad = calculus.gradient(f) * phiEigen;
  gradients.push_back( grad );

  auto cograd =  calculus.coGradient(f) * phiEigen;
  cogradients.push_back( cograd );

  normals.push_back(calculus.faceNormalAsDGtalVector(f));
  
  auto vA = calculus.vectorArea(f);
  vectorArea.push_back({vA(0) , vA(1), vA(2)});
  
  faceArea.push_back( calculus.faceArea(f));
  centroids.push_back( calculus.centroidAsDGtalPoint(f) );
  
  PolygonalCalculus<SurfMesh>::Vector dPhi = calculus.D(f)*phiEigen;
  PolygonalCalculus<SurfMesh>::Vector av = calculus.A(f)*phiEigen;
  
  psMesh->addFaceVectorQuantity("Gradients", gradients);
  psMesh->addFaceVectorQuantity("co-Gradients", cogradients);
  psMesh->addFaceVectorQuantity("Normals", normals);
  psMesh->addFaceScalarQuantity("Face area", faceArea);
  psMesh->addFaceVectorQuantity("Vector area", vectorArea);
  
  psBoundary->addEdgeScalarQuantity("d0*phi", dPhi);
  psBoundary->addEdgeScalarQuantity("A*phi", av);
  
  polyscope::registerPointCloud("Centroids", centroids);
  
  
  //Flat Sharp
  PolygonalCalculus<SurfMesh>::Vector v(3);
  v << 50,-50,-100;
  std::vector<PolygonalCalculus<SurfMesh>::Vector> someV={v};
  psMesh->addFaceVectorQuantity("A vector", someV);

  PolygonalCalculus<SurfMesh>::Vector flat = calculus.V(f)*v;
  psBoundary->addEdgeScalarQuantity("flat (1-form)", flat);
  
  PolygonalCalculus<SurfMesh>::Vector sharp = calculus.U(f)*flat;
  std::vector<PolygonalCalculus<SurfMesh>::Vector> sharpRes={sharp};
  psMesh->addFaceVectorQuantity("Sharp", sharpRes);
}


int main()
{
  std::vector<RealPoint> positions={ {0,0,0},{20,0,0},{20,10,0},{10,8,5}, {0,15,1} };
  std::vector<std::vector<size_t>> faces={{ 0,1,2,3,4}};

  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());
  
  psVertices = polyscope::registerPointCloud("Vertices", positions);
    
  std::vector<std::array<size_t,2>> edges={{0,1},{1,2},{2,3},{3,4},{4,0} };
  psBoundary = polyscope::registerCurveNetwork("Edges", positions, edges);
  
  // Initialize polyscope
  polyscope::init();
  
  psMesh = polyscope::registerSurfaceMesh("Single face", positions, faces);

  initPhi();
  initQuantities();
  
  polyscope::show();
  return EXIT_SUCCESS;
  
}
