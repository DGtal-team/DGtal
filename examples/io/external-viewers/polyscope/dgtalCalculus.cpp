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

//Restriction of an ambient scalar function to vertices
double phiVertex(const Vertex v)
{
  return  cos(scale*(surfmesh.position(v)[0]))*sin(scale*surfmesh.position(v)[1]);
}

//Restriction of an ambient scalar function to vertices
PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector phiFace(const Face f)
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

//Vertex valued function for polyscope
void initPhi()
{
  phiV.clear();
  for(auto i = 0; i < surfmesh.nbVertices(); ++i)
    phiV.push_back(phiVertex(i));
  psMesh->addVertexScalarQuantity("Phi", phiV);
}

void initQuantities()
{
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector> calculus(surfmesh);

  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector> gradients;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector> cogradients;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Real3dVector> normals;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Real3dVector> vectorArea;
  std::vector<PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Real3dPoint> centroids;
  std::vector<double> faceArea;

  for(auto f=0; f < surfmesh.nbFaces(); ++f)
  {
    PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector ph = phiFace(f);
    PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector grad = calculus.gradient(f) * ph;
    gradients.push_back( grad );
    PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector cograd =  calculus.coGradient(f) * ph;
    cogradients.push_back( cograd );
    normals.push_back(calculus.faceNormalAsDGtalVector(f));

    auto vA = calculus.vectorArea(f);
    vectorArea.push_back({vA(0) , vA(1), vA(2)});

    faceArea.push_back( calculus.faceArea(f));

    centroids.push_back( calculus.centroidAsDGtalPoint(f) );
  }

  psMesh->addFaceVectorQuantity("Gradients", gradients);
  psMesh->addFaceVectorQuantity("co-Gradients", cogradients);
  psMesh->addFaceVectorQuantity("Normals", normals);
  psMesh->addFaceScalarQuantity("Face area", faceArea);
  psMesh->addFaceVectorQuantity("Vector area", vectorArea);

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
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();

  auto h=.3 ; //gridstep
  params( "polynomial", "goursat" )( "gridstep", h );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  SH3::Cell2Index c2i;
  auto primalSurface   = SH3::makePrimalSurfaceMesh(c2i, surface);

  // Convert faces to appropriate indexed format
  std::vector<std::vector<SH3::SurfaceMesh::Vertex>> faces;
  for(auto face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));

  //Recasting to vector of vertices
  auto positions = primalSurface->positions();

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
