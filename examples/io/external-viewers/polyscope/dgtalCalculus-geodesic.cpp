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
 * @ingroup Examples
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
typedef PolygonalCalculus<SH3::RealPoint,SH3::RealVector> PolyCalculus;
//Polyscope global
polyscope::SurfaceMesh *psMesh;
polyscope::SurfaceMesh *psMeshReg;
SurfMesh surfmesh;
SurfMesh surfmeshReg;
float dt = 2.0;

CountedPtr<SH3::BinaryImage> binary_image;
CountedPtr<SH3::DigitalSurface> surface;


GeodesicsInHeat<PolyCalculus> *heat;
PolyCalculus *calculus;

GeodesicsInHeat<PolyCalculus> *heatReg;
PolyCalculus *calculusReg;

SHG3::RealVectors iinormals;

int sourceVertexId=0;
float radiusII = 3.0;

bool skipReg = true; //Global flag to enable/disable the regularization example.
bool useProjectedCalculus = true; //Use estimated normal vectors to set up te embedding

void precompute()
{

  if (!useProjectedCalculus)
    calculus = new PolyCalculus(surfmesh);
  else
  {
    //Using the projection embedder
    auto params2 = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
    params2("r-radius", (double) radiusII);
    auto surfels   = SH3::getSurfelRange( surface, params2 );
    iinormals = SHG3::getIINormalVectors(binary_image, surfels,params2);
    trace.info()<<iinormals.size()<<std::endl;
    psMesh->addFaceVectorQuantity("II normals", iinormals);

    calculus = new PolyCalculus(surfmesh);
    functors::EmbedderFromNormalVectors<Z3i::RealPoint, Z3i::RealVector> embedderFromNormals(iinormals,surfmesh);
    calculus->setEmbedder( embedderFromNormals );
  }

  heat = new GeodesicsInHeat<PolyCalculus>(calculus);

  if (!skipReg)
  {
    calculusReg = new PolyCalculus(surfmeshReg);
    heatReg = new GeodesicsInHeat<PolyCalculus>(calculusReg);
  }
  trace.beginBlock("Init solvers");
  heat->init(dt);
  if (!skipReg)
    heatReg->init(dt);
  trace.endBlock();
}



void addSource()
{
  auto pos =rand() % surfmesh.nbVertices();
  heat->addSource( pos );
  GeodesicsInHeat<PolyCalculus>::Vector source = heat->source();
  psMesh->addVertexScalarQuantity("Sources", source);

  if (!skipReg)
  {
    heatReg->addSource( pos );
    GeodesicsInHeat<PolyCalculus>::Vector source = heatReg->source();
    psMeshReg->addVertexScalarQuantity("Sources", source);
  }
}

void clearSources()
{
  heat->clearSource();
  psMesh->addVertexScalarQuantity("source", heat->source());
}

void computeGeodesics()
{
  heat->addSource( sourceVertexId ); //Forcing one seed (for screenshots)
  GeodesicsInHeat<PolyCalculus>::Vector source = heat->source();
  psMesh->addVertexScalarQuantity("Sources", source);
  GeodesicsInHeat<PolyCalculus>::Vector dist = heat->compute();
  psMesh->addVertexDistanceQuantity("geodesic", dist);

  if (!skipReg)
  {
    heatReg->addSource( sourceVertexId ); //Forcing one seed (for screenshots)371672
    GeodesicsInHeat<PolyCalculus>::Vector sourceReg = heatReg->source();
    psMeshReg->addVertexScalarQuantity("Sources", sourceReg);
    GeodesicsInHeat<PolyCalculus>::Vector dist = heatReg->compute();
    psMeshReg->addVertexDistanceQuantity("geodesic", dist);
  }
}

bool isPrecomputed=false;
void myCallback()
{
  ImGui::SliderFloat("dt", &dt, 0.,4.);
  ImGui::SliderFloat("ii radius for normal vector estimation", &radiusII , 0.,10.);
  ImGui::Checkbox("Skip regularization", &skipReg);
  ImGui::Checkbox("Using projection", &useProjectedCalculus);
  ImGui::InputInt("Index of the first source vertex", &sourceVertexId);


  if(ImGui::Button("Precomputation (required if you change parameters)"))
  {
    precompute();
    isPrecomputed=true;
  }
  ImGui::Separator();
  if(ImGui::Button("Add a random source"))
  {
    if (!isPrecomputed)
    {
      precompute();
      isPrecomputed=true;
    }
    addSource();
  }
  if(ImGui::Button("Clear sources"))
  {
    if (!isPrecomputed)
    {
      precompute();
      isPrecomputed=true;
    }
    clearSources();
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
  params("r-radius", (double) radiusII);
  std::string filename = examplesPath + std::string("/samples/bunny-128.vol");
  binary_image         = SH3::makeBinaryImage(filename, params );
  auto K               = SH3::getKSpace( binary_image, params );
  surface              = SH3::makeDigitalSurface( binary_image, K, params );
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
  std::cout << surfmesh << std::endl;
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
  psMeshReg->setEnabled(false);

  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
