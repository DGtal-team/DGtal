/*
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
 * @file shapes/exampleSurfaceMesh.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/15
 *
 * @brief An example file for SurfaceMesh in 3D.
 *
 * This file is part of the DGtal library.
 */


/**
   This snippet shows how to build SurfaceMesh objects from different sources:
   
   - pyramid: construction from list of coordinates and list of faces
   - torus:   construction from predefined mesh surface
   - cow:     construction from OBJ file.

   It also shows how to export it as OBJ files.

   @see \ref moduleSurfaceMesh

   \image html exampleSurfaceMesh.jpg "Visualisation of three surface meshes, using conversion from SurfaceMesh to Mesh"

   \example shapes/exampleSurfaceMesh.cpp
*/


///////////////////////////////////////////////////////////////////////////////
#include <string>
#include <iostream>
#include <fstream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/shapes/Mesh.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/SurfaceMeshHelper.h"
#include "DGtal/shapes/MeshHelpers.h"
#include "DGtal/graph/BreadthFirstVisitor.h"
#include "DGtal/io/readers/SurfaceMeshReader.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;


///////////////////////////////////////////////////////////////////////////////
int main()
{
  //! [exampleSurfaceMesh-typedef]
  // The following typedefs are useful
  typedef SurfaceMesh< RealPoint, RealVector >       SurfMesh;
  typedef SurfaceMeshHelper< RealPoint, RealVector > Helper;
  typedef SurfMesh::Vertices                         Vertices;
  //! [exampleSurfaceMesh-typedef]
  
  trace.beginBlock ( "Reading a mesh OBJ file" );
  //! [exampleSurfaceMesh-read-mesh]
  SurfMesh    smesh;
  std::string S = examplesPath + "samples/spot.obj";
  std::ifstream input( S.c_str() );
  bool ok_read  =  SurfaceMeshReader< RealPoint, RealVector >::readOBJ( input, smesh );
  input.close();
  trace.info() << "Read " << ( ok_read ? "OK" : "ERROR" )
               << " mesh=" << smesh << std::endl;
  //! [exampleSurfaceMesh-read-mesh]
  trace.endBlock();

  trace.beginBlock ( "Building a torus" );
  //! [exampleSurfaceMesh-make-torus]
  auto torus_mesh = Helper::makeTorus
    ( 2.5, 0.5, RealPoint { 0.0, 0.0, 0.0 }, 40, 40, 0, Helper::NormalsType::NO_NORMALS );
  //! [exampleSurfaceMesh-make-torus]
  trace.endBlock();

  trace.beginBlock ( "Building a pyramid" );
  //! [exampleSurfaceMesh-make-pyramid]
  std::vector< RealPoint > positions =
    { { 0, 0, 5 }, { 1, 1, 3 }, { -1, 1, 3 }, { -1, -1, 3 }, { 1, -1, 3 } };
  std::vector< Vertices  > faces =
    { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 1 }, { 4, 3, 2, 1 } };
  auto pyramid_mesh = SurfMesh( positions.cbegin(), positions.cend(),
                                faces.cbegin(), faces.cend() );
  //! [exampleSurfaceMesh-make-pyramid]
  trace.endBlock();

  trace.beginBlock ( "Traversing a mesh by BF" );
  //! [exampleSurfaceMesh-graph-bft]
  BreadthFirstVisitor< SurfMesh > visitor( smesh, 0 );
  std::vector<double>             distances( smesh.nbVertices() );
  double biggest_d = 0.0;
  while ( ! visitor.finished() )
    {
      auto v = visitor.current().first;  // current vertex
      auto d = visitor.current().second; // current distance
      biggest_d      = (double) d;
      distances[ v ] = biggest_d;
      visitor.expand();
    }
  //! [exampleSurfaceMesh-graph-bft]
  trace.endBlock();

  trace.beginBlock ( "Colored output of BF traversal" );
  //! [exampleSurfaceMesh-write-obj]
  // Displaying faces colored by their distance to vertex 0.
  auto face_distances = smesh.computeFaceValuesFromVertexValues( distances );
  auto cmap = GradientColorMap< double >( 0.0, biggest_d, CMAP_JET );
  std::vector<Color> face_colors( smesh.nbFaces() );
  for ( SurfMesh::Face j = 0; j < smesh.nbFaces(); ++j )
    face_colors[ j ] = cmap( face_distances[ j ] );      
  typedef SurfaceMeshWriter< RealPoint, RealVector > Writer;
  Writer::writeOBJ( "spot-bft.obj", smesh, face_colors );

  // Displaying three isolines.
  Writer::writeIsoLinesOBJ( "spot-iso-0_25.obj", smesh,
                            face_distances, distances, distances.back() * 0.25, 0.2 );
  Writer::writeIsoLinesOBJ( "spot-iso-0_5.obj",  smesh,
                            face_distances, distances, distances.back() * 0.5,  0.2 );
  Writer::writeIsoLinesOBJ( "spot-iso-0_75.obj", smesh,
                            face_distances, distances, distances.back() * 0.75, 0.2 );
  //! [exampleSurfaceMesh-write-obj]
  trace.endBlock();

  // Conversion to Mesh for easy display.
  Mesh< RealPoint > viewmesh(true);
  Mesh< RealPoint >  viewmesh2, viewmesh3;
  MeshHelpers::surfaceMesh2Mesh( smesh,      viewmesh , face_colors );
  MeshHelpers::surfaceMesh2Mesh( torus_mesh, viewmesh2 );
  MeshHelpers::surfaceMesh2Mesh( pyramid_mesh, viewmesh3 );

  PolyscopeViewer<> viewer;
  viewer << viewmesh << viewmesh2 << viewmesh3;
  viewer.show();
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
