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
 * @file exampleVectorHeatMethod.cpp
 * @author Baptiste GENEST (\c baptistegenest@gmail.com )
 * intership at Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2022/06/15
 *
 * This file is part of the DGtal library.
 */

//////// INCLUDES ////////

#include <iostream>

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/dec/PolygonalCalculus.h>
#include <DGtal/dec/VectorsInHeat.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

////////// NAMESPACES /////////

using namespace DGtal;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< Z3i::RealPoint, Z3i::RealVector > 			  SurfMesh;
typedef SurfMesh::Vertices                   			  Vertices;
typedef SurfMesh::RealPoint                  			  RealPoint;
typedef SurfMesh::Face                       			  Face;
typedef SurfMesh::Vertex                     			  Vertex;
typedef PolygonalCalculus<SH3::RealPoint,SH3::RealVector> PC;
typedef PC::Vector Vector;
typedef PC::SparseMatrix SparseMatrix;

//Polyscope global
polyscope::SurfaceMesh *psMesh;

SurfMesh surfmesh;

//Polygonal Calculus and VectorsInHeat solvers
PC *calculus;
VectorsInHeat<PC> *VHM;

//sources
std::vector<Vector> X_0;
std::vector<Vertex> idX_0;

//rotation matrix
PC::DenseMatrix R;

bool noSources = true;
bool toggle=false;

/**
 * @brief addRandomSource add a random vector in the tangent space
 * of a vertex
 */
void addRandomSource()
{
  size_t id = rand()%surfmesh.nbVertices();
  VHM->addSource(id,Eigen::Vector3d::Random(3).normalized());

  idX_0.push_back(id);
  X_0[id] = VHM->extrinsicVectorSourceAtVertex(id);
  psMesh->addVertexVectorQuantity("X_0",X_0);
  noSources = false;
}

/**
 * @brief Solve the linear systems and add the solution to the
 * display, if no source is given, adds a random one
 */
void diffuse()
{
  if (noSources)
    addRandomSource();
  psMesh->addVertexVectorQuantity("VHM field",VHM->compute());
}

/**
 * @brief Precompute initialize VHM solvers, and source container
 */
void precompute()
{
  auto nv = surfmesh.nbVertices();
  auto ael = surfmesh.averageEdgeLength();
  VHM->init(ael);//init vector heat method solvers (should be ael^2 but smoother results that way)

  X_0.resize(nv,Vector::Zero(3));//extrinsic Source vectors

  psMesh->addVertexVectorQuantity("X_0",X_0);
}

void clearSources()
{
  VHM->clearSource();
  noSources=true;
  idX_0.clear();
  X_0.clear();
  X_0.resize(surfmesh.nbVertices(),Vector::Zero(3));
  //cleanup the visualization
  psMesh->addVertexVectorQuantity("X_0",X_0);
  psMesh->addVertexVectorQuantity("VHM field",X_0);
}

void rotate()
{
  VHM->clearSource();
  for(const auto id: idX_0)
  {
    Vector x = R*X_0[id];
    VHM->addSource(id, x);
    X_0[id] = VHM->extrinsicVectorSourceAtVertex(id);
  }
  psMesh->addVertexVectorQuantity("X_0",X_0);
}


void myCallback()
{
  if(ImGui::Button("Compute Vector Field"))
  {
    diffuse();
  }
  if(ImGui::Button("Add random source"))
  {
    addRandomSource();
  }
  if(ImGui::Button("Clear sources"))
  {
    clearSources();
  }

  if(ImGui::Button("Start/stop rotating sources"))
      toggle = !toggle;


  if (toggle)
  {
    rotate();
    diffuse();
  }
}

int main(int argc, char **argv)
{
  std::vector<std::vector<SH3::SurfaceMesh::Vertex>> faces;
  std::vector<RealPoint> positions;

  if (argc <= 1)
  {
    trace.error()<<"Missing vol file. Usage: exampleVectorHeatMethod bunny.vol"<<std::endl;
    exit(2);
  }

  //load voxel model
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params("surfaceComponents", "All");
  auto binary_image    = SH3::makeBinaryImage(argv[1], params );
  auto K               = SH3::getKSpace( binary_image, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);

  //Need to convert the faces
  for(size_t face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));

  //Recasting to vector of vertices
  positions = primalSurface->positions();

  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());

  //instantiate PolyDEC
  calculus = new PC(surfmesh);

  //instantiate VHM
  VHM = new VectorsInHeat<PC>(calculus);

  //For the rotation of input VF
  PC::DenseMatrix Rotx(3,3),Roty(3,3),Rotz(3,3);
  double theta=0.05;
  Rotx << 1, 0,0,0,cos(theta),-sin(theta),0,sin(theta),cos(theta);
  Roty << cos(theta), 0,sin(theta),0,1,0,-sin(theta),0,cos(theta);
  Rotz << cos(theta), -sin(theta),0,sin(theta),cos(theta),0,0,0,1;
  R=Rotz*Roty*Rotx;

  //Initialize polyscope
  polyscope::init();

  psMesh = polyscope::registerSurfaceMesh("Digital Surface", positions, faces);

  //Initialize solvers
  precompute();

  polyscope::view::upDir = polyscope::view::UpDir::XUp;

  polyscope::state::userCallback = myCallback;

  polyscope::show();
  return EXIT_SUCCESS;

}
