#include <iostream>
#include <string>
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
float scale = 0.1;
PolygonalCalculus<SurfMesh>::Vector phiEigen;



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
  phiEigen.resize(surfmesh.nbVertices());
  for(auto i = 0; i < surfmesh.nbVertices(); ++i)
    phiEigen(i) = phiVertex(i);
  psMesh->addVertexScalarQuantity("Phi", phiEigen);
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
  
  for(auto f=0; f < surfmesh.nbFaces(); ++f)
  {
    
    auto grad = calculus.gradient(f) * phi(f);
    gradients.push_back( grad );
    
    auto cograd =  calculus.coGradient(f) * phi(f);
    cogradients.push_back( cograd );
    
    normals.push_back(calculus.faceNormalAsDGtalVector(f));
    
    auto vA = calculus.vectorArea(f);
    vectorArea.push_back({vA(0) , vA(1), vA(2)});
    
    faceArea.push_back( calculus.faceArea(f));
  }
  
  
  psMesh->addFaceVectorQuantity("Gradients", gradients);
  psMesh->addFaceVectorQuantity("co-Gradients", cogradients);
  psMesh->addFaceVectorQuantity("Normals", normals);
  psMesh->addFaceScalarQuantity("Face area", faceArea);
  psMesh->addFaceVectorQuantity("Vector area", vectorArea);
  
}


void computeLaplace()
{
  PolygonalCalculus<SurfMesh> calculus(surfmesh);
  PolygonalCalculus<SurfMesh>::SparseMatrix L = calculus.globalLaplaceBeltrami();
  PolygonalCalculus<SurfMesh>::Vector g = PolygonalCalculus<SurfMesh>::Vector::Zero(surfmesh.nbVertices());

  for(auto v=0;v<surfmesh.nbVertices();++v)
  {
    
   if (surfmesh.position(v)[2] > 50)
     g( v )  =  10.0;
   else
     if (surfmesh.position(v)[2] < 25)
       g( v )  =  -10.0;
     else
       g(v)=0.0;
  }

  //Solve Δu=0 with g as boundary conditions
  PolygonalCalculus<SurfMesh>::Solver solver;
  solver.compute(L);
  ASSERT(solver.info()==Eigen::Success);

  PolygonalCalculus<SurfMesh>::Vector u = solver.solve(g);
  ASSERT(solver.info()==Eigen::Success);

  psMesh->addVertexScalarQuantity("g", g);
  psMesh->addVertexScalarQuantity("u", u);
}

void myCallback()
{
  ImGui::SliderFloat("Phi scale", &scale, 0., 1.);
  if (ImGui::Button("Phi and basic operators"))
  {
    initPhi();
    initQuantities();
  }
  if(ImGui::Button("Compute Laplace problem"))
    computeLaplace();
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();

  auto h=1.; //gridstep
  params("surfaceComponents", "All");

  std::string filename = examplesPath + std::string("/samples/bunny-64.vol");
  
  auto binary_image = SH3::makeBinaryImage(filename, params );
  auto K            = SH3::getKSpace( binary_image );
  auto surface      = SH3::makeDigitalSurface( binary_image, K, params );
  auto surfels      = SH3::getSurfelRange( surface, params );
  auto embedder     = SH3::getCellEmbedder( K );
  SH3::Cell2Index c2i;
  auto primalSurface   = SH3::makePrimalPolygonalSurface(c2i, surface);
  
  //Need to convert the faces
  std::vector<std::vector<unsigned int>> faces;

  std::vector<RealPoint> positions;
  unsigned int cpt=0;
  for(auto &surfel: surfels)
  {
    auto verts = SH3::getPrimalVertices(K, surfel, false );
    for(auto &v: verts)
      positions.push_back(embedder(v));

    std::vector<unsigned int> face={cpt, cpt+1, cpt+2,cpt+3};
    cpt+=4;
    faces.push_back(face);
  }

/*  std::vector<std::vector<unsigned long>> faces;
  for(auto &face: primalSurface->allFaces())
    faces.push_back(primalSurface->verticesAroundFace( face ));

  auto pos = primalSurface->positions();
  std::vector<RealPoint> positions(primalSurface->nbVertices());
  for(auto i=0; i < primalSurface->nbVertices(); ++i)
    positions[i] = pos(i);
  */
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
