/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/**
 * @file
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2021/09/02
 *
 * This file is part of the DGtal library.
 */
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
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "ConfigExamples.h"

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
PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector phiEigen;

//Restriction of a scalar function to vertices
double phiVertex(const Vertex v)
{
  return  cos(scale*(surfmesh.position(v)[0]))*sin(scale*surfmesh.position(v)[1]);
}

//Restriction of a scalar function to vertices
PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector phi(const Face f)
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
  PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector> calculus(surfmesh);
  
  std::vector<PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector> gradients;
  std::vector<PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector> cogradients;
  std::vector<PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Real3dPoint> normals;
  std::vector<PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Real3dPoint> vectorArea;
  std::vector<PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Real3dPoint> centroids;
  
  std::vector<double> faceArea;
  
  for(auto f=0; f < surfmesh.nbFaces(); ++f)
  {
    PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector grad = calculus.gradient(f) * phi(f);
    gradients.push_back( grad );
    
    PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector cograd =  calculus.coGradient(f) * phi(f);
    cogradients.push_back( cograd );
    
    normals.push_back(calculus.faceNormalAsDGtalVector(f));
    
    PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector vA = calculus.vectorArea(f);
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
  PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector> calculus(surfmesh);
  trace.beginBlock("Operator construction...");
  PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::SparseMatrix L = calculus.globalLaplaceBeltrami();
  trace.endBlock();
  PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector g = PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector::Zero(surfmesh.nbVertices());

  //Setting some random sources
  for(auto cpt=0; cpt< 10;++cpt)
    g( rand() % surfmesh.nbVertices()) =  rand() % 100;
  
  //Solve Δu=0 with g as boundary conditions
  PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Solver solver;
  PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::SparseMatrix I(surfmesh.nbVertices(),surfmesh.nbVertices());
  I.setIdentity();
  
  trace.beginBlock("Prefactorization...");
  solver.compute(L + 0.0001*I);  //regularization needed for closed surface.
  ASSERT(solver.info()==Eigen::Success);
  trace.endBlock();
  
  trace.beginBlock("Solve...");
  PolygonalCalculus<SurfMesh::RealPoint,SurfMesh::RealVector>::Vector u = solver.solve(g);
  ASSERT(solver.info()==Eigen::Success);
  trace.endBlock();
  
  std::cout << solver.info() << std::endl;
  std::cout << g.maxCoeff()<< std::endl;
  std::cout << g.minCoeff() << std::endl;
  std::cout << u.maxCoeff()<< std::endl;
  std::cout << u.minCoeff() << std::endl;
  
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
  params("surfaceComponents", "All");

  std::string filename = examplesPath + std::string("/samples/bunny-32.vol");
  auto binary_image    = SH3::makeBinaryImage(filename, params );
  auto K               = SH3::getKSpace( binary_image, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  SH3::Cell2Index c2i;
  auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);
  
  //Need to convert the faces
  std::vector<std::vector<SH3::SurfaceMesh::Vertex>> faces;
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
  psMesh = polyscope::registerSurfaceMesh("digital surface", positions, faces);

  // Initialize polyscope
  polyscope::init();
  
  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
