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
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/04/10
 * @ingroup Examples
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

#include <DGtal/dec/PolygonalCalculus.h>
#include <DGtal/dec/GeodesicsInHeat.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/pick.h>

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
SurfMesh surfmesh;
float dt = 2.0;
float Lambda = 0.05;
bool Mixed = false;
GeodesicsInHeat<PolyCalculus> *heat;
PolyCalculus *calculus;
int vertex_idx = -1;
int face_idx   = -1;
int edge_idx   = -1;



void precompute()
{
  auto projEmbedder = [&]( int f, int v)
  {
    auto nn = surfmesh.faceNormal( f );
    RealPoint centroid = surfmesh.faceCentroid( f );
    RealPoint p = surfmesh.position( v );
    const auto cp = p - centroid;
    RealPoint q = p - nn.dot(cp)*nn;
    return q;
  };
  calculus = new PolyCalculus(surfmesh);
  calculus->setEmbedder( projEmbedder );
  heat = new GeodesicsInHeat<PolyCalculus>(calculus);
  trace.beginBlock("Init solvers");
  heat->init(dt,Lambda,Mixed);
  trace.endBlock();
}

void picksource( int v_idx )
{
  heat->addSource( v_idx );
  GeodesicsInHeat<PolyCalculus>::Vector source = heat->source();
  psMesh->addVertexScalarQuantity("source", source);
}

void computeGeodesics()
{
  // heat->addSource( 0 ); //Forcing one seed (for screenshots)
  GeodesicsInHeat<PolyCalculus>::Vector dist = heat->compute();
  psMesh->addVertexDistanceQuantity("geodesic", dist);
}

bool isPrecomputed=false;
void myCallback()
{
  // Select a vertex with the mouse
  if (polyscope::haveSelection()) {
    bool goodSelection = false;
    auto selection = polyscope::getSelection();
    auto selectedSurface = static_cast<polyscope::SurfaceMesh*>(selection.structure);
    size_t idx = selection.localIndex;

    // Only authorize selection on the input surface and the reconstruction
    auto surf = polyscope::getSurfaceMesh("digital surface");
    goodSelection = goodSelection || (selectedSurface == surf);
    const auto nv = selectedSurface->nVertices(); 
    // Validate that it its a face index
    if ( goodSelection && idx < nv )
      {
        std::ostringstream otext;
        otext << "Selected vertex = " << idx;
        ImGui::Text( "%s", otext.str().c_str() );
        vertex_idx = idx;
        if (!isPrecomputed)
          {
            precompute();
            isPrecomputed=true;
          }
        picksource( vertex_idx );
      }
  }
  
  ImGui::SliderFloat("Lambda parameter", &Lambda, 0.0, 1.0);
  ImGui::SliderFloat("dt", &dt, 0.,4.);
  ImGui::Checkbox("Use mixed Neumann+Dirichlet heat solution", &Mixed);
  if(ImGui::Button("Precomputation (required if you change the dt)"))
  {
    precompute();
    isPrecomputed=true;
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
  ImGui::Text("You can use the mouse left-click to select the new source point.");
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params("surfaceComponents", "All");
  params("polynomial", "x^2+(y+1)^2+z^2-1.0")
    ("minAABB",-1.0)("maxAABB",1.0)("offset",1.0)
    ("gridstep",0.0625);
  auto shape        = SH3::makeImplicitShape3D( params );
  auto dshape       = SH3::makeDigitizedImplicitShape3D( shape, params );
  auto K            = SH3::getKSpace( params );
  auto binary_image = SH3::makeBinaryImage( dshape, params );
  auto surface      = SH3::makeDigitalSurface( binary_image, K, params );
  auto primalSurface= SH3::makePrimalSurfaceMesh(surface);
  auto surfels      = SH3::getSurfelRange( surface, params );
  auto normals      = SHG3::getNormalVectors( shape, K, surfels, params );
  
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
  surfmesh.setFaceNormals( normals.cbegin(), normals.cend() );
  std::cout << surfmesh << std::endl;
  std::cout<<"number of non-manifold Edges = " << surfmesh.computeNonManifoldEdges().size()<<std::endl;
  
  // Initialize polyscope
  polyscope::init();

  psMesh = polyscope::registerSurfaceMesh("digital surface", positions, faces);

  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
