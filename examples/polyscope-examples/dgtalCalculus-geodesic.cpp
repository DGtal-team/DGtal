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
polyscope::SurfaceMesh *psMeshReg;
SurfMesh surfmesh;
SurfMesh surfmeshReg;
float dt = 2.0;

GeodesicsInHeat<PolygonalCalculus<SurfMesh>> *heat;
PolygonalCalculus<SurfMesh> *calculus;

GeodesicsInHeat<PolygonalCalculus<SurfMesh>> *heatReg;
PolygonalCalculus<SurfMesh> *calculusReg;

bool skipReg = true; //Global flag to enable/disable the regularization example.

void precompute()
{
  calculus = new PolygonalCalculus<SurfMesh>(surfmesh);
  heat = new GeodesicsInHeat<PolygonalCalculus<SurfMesh>>(calculus);
  
  if (!skipReg)
  {
    calculusReg = new PolygonalCalculus<SurfMesh>(surfmeshReg);
    heatReg = new GeodesicsInHeat<PolygonalCalculus<SurfMesh>>(calculusReg);
  }
  trace.beginBlock("Init solvers");
  heat->init(dt);
  if (!skipReg)
    heatReg->init(dt);
  trace.endBlock();
}

void addsource()
{
  auto pos =rand() % surfmesh.nbVertices();
  heat->addSource( pos );
  GeodesicsInHeat<PolygonalCalculus<SurfMesh>>::Vector source = heat->source();
  psMesh->addVertexScalarQuantity("source", source);

  if (!skipReg)
  {
    heatReg->addSource( pos );
    GeodesicsInHeat<PolygonalCalculus<SurfMesh>>::Vector source = heatReg->source();
    psMeshReg->addVertexScalarQuantity("source", source);
  }
}

void computeGeodesics()
{

  heat->addSource( 0 ); //Forcing one seed (for screenshots)
  GeodesicsInHeat<PolygonalCalculus<SurfMesh>>::Vector dist = heat->compute();
  psMesh->addVertexDistanceQuantity("geodesic", dist);

  if (!skipReg)
  {
    heatReg->addSource( 0 ); //Forcing one seed (for screenshots)
    GeodesicsInHeat<PolygonalCalculus<SurfMesh>>::Vector dist = heatReg->compute();
    psMeshReg->addVertexDistanceQuantity("geodesic", dist);
  }
}

bool isPrecomputed=false;
void myCallback()
{
  ImGui::SliderFloat("dt", &dt, 0.,4.);
  ImGui::Checkbox("Skip regularization", &skipReg);
  if(ImGui::Button("Precomputation (required if you change the dt)"))
  {
    precompute();
    isPrecomputed=true;
  }
  if(ImGui::Button("Add random source"))
  {
    if (!isPrecomputed)
    {
      precompute();
      isPrecomputed=true;
    }
    addsource();
  }
  
  if(ImGui::Button("Compute geodesic"))
  {
    if (!isPrecomputed)
    {
      precompute();
      isPrecomputed=true;
    }
    computeGeodesics();
  }
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params("surfaceComponents", "All");
  std::string filename = examplesPath + std::string("/samples/bunny-32.vol");
  auto binary_image    = SH3::makeBinaryImage(filename, params );
  auto K               = SH3::getKSpace( binary_image, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
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
  
  //Construction of a regularized surface
  DigitalSurfaceRegularization<SH3::DigitalSurface> regul(surface);
  regul.init();
  regul.attachConvolvedTrivialNormalVectors(params);
  regul.regularize();
  auto regularizedPosition = regul.getRegularizedPositions();

  surfmeshReg = SurfMesh(regularizedPosition.begin(),
                      regularizedPosition.end(),
                      faces.begin(),
                      faces.end());
  
  // Initialize polyscope
  polyscope::init();

  psMesh = polyscope::registerSurfaceMesh("digital surface", positions, faces);
  psMeshReg = polyscope::registerSurfaceMesh("regularized surface", regularizedPosition, faces);

  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
