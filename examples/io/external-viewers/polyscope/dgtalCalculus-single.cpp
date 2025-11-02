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
 * @ingroup Examples
 * This file is part of the DGtal library.
 */
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
PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector phiEigen;

void initPhi()
{
  phiEigen.resize(5);
  phiEigen << 1.0, 2.0, 0.0, 5.0 ,1.5;
  
  psMesh->addVertexScalarQuantity("Phi", phiEigen);
  psVertices->addScalarQuantity("Phi", phiEigen);
}

void initQuantities()
{
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector> calculus(surfmesh);

  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector> gradients;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector> cogradients;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Real3dPoint> normals;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Real3dPoint> vectorArea;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Real3dPoint> centroids;
  std::vector<double> faceArea;

  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Face f = 0; //Id of the face
 
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector grad = calculus.gradient(f) * phiEigen;
  gradients.push_back( grad );

  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector cograd =  calculus.coGradient(f) * phiEigen;
  cogradients.push_back( cograd );

  normals.push_back(calculus.faceNormalAsDGtalVector(f));
  
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector vA = calculus.vectorArea(f);
  vectorArea.push_back({vA(0) , vA(1), vA(2)});
  
  faceArea.push_back( calculus.faceArea(f));
  centroids.push_back( calculus.centroidAsDGtalPoint(f) );
  
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector dPhi = calculus.D(f)*phiEigen;
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector av = calculus.A(f)*phiEigen;
  
  psMesh->addFaceVectorQuantity("Gradients", gradients);
  psMesh->addFaceVectorQuantity("co-Gradients", cogradients);
  psMesh->addFaceVectorQuantity("Normals", normals);
  psMesh->addFaceScalarQuantity("Face area", faceArea);
  psMesh->addFaceVectorQuantity("Vector area", vectorArea);
  
  psBoundary->addEdgeScalarQuantity("d0*phi", dPhi);
  psBoundary->addEdgeScalarQuantity("A*phi", av);
  
  //Face centroid
  polyscope::registerPointCloud("Centroids", centroids);
  
  //Flat Sharp
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector v(3);
  v << 50,-50,-100;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector> someV={v};
  psMesh->addFaceVectorQuantity("A vector", someV);

  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector flat = calculus.flat(f)*v;
  psBoundary->addEdgeScalarQuantity("flat (1-form)", flat);
  
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector sharp = calculus.sharp(f)*flat;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector> sharpRes={sharp};
  psMesh->addFaceVectorQuantity("Sharp", sharpRes);
}

int main()
{
  std::vector<RealPoint> positions={ {0,0,0},{20,0,0}, {20,10,0}, {10,8,5}, {0,15,1} };
  std::vector<std::vector<size_t>> faces={ {0,1,2,3,4} };

  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());

  // Initialize polyscope
  polyscope::init();
  
  psVertices = polyscope::registerPointCloud("Vertices", positions);
    
  std::vector<std::array<size_t,2>> edges={{0,1},{1,2},{2,3},{3,4},{4,0} };
  psBoundary = polyscope::registerCurveNetwork("Edges", positions, edges);
  
  
  psMesh = polyscope::registerSurfaceMesh("Single face", positions, faces);

  initPhi();
  initQuantities();
  
  polyscope::show();
  return EXIT_SUCCESS;
  
}
