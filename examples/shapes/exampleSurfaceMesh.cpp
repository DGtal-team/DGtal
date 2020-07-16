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
 **/

/**
 * @file geometry/shapes/exampleSurfaceMesh.cpp
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
   This snippet shows how to identify and display digital fully
   subconvex sets of a grid curve form its tangent bundle.

   @see \ref moduleSurfaceMesh

   \image html grid-curve-dig-convexity.png "Extraction of all subconvex triangles to the digital curve."

   \example geometry/shapes/exampleSurfaceMesh.cpp
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
#include "DGtal/io/readers/SurfaceMeshReader.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/Color.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;


///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  trace.beginBlock ( "Reading a mesh OBJ file" );
  //! [exampleSurfaceMesh-read-mesh]
  SurfaceMesh< RealPoint, RealVector > smesh;
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
  typedef SurfaceMeshHelper< RealPoint, RealVector > Helper;
  auto torus_mesh = Helper::makeTorus
    ( 2.5, 0.5, RealPoint { 0.0, 0.0, 0.0 }, 40, 40, 0, Helper::Normals::NO_NORMALS );
  //! [exampleSurfaceMesh-make-torus]
  trace.endBlock();

  trace.beginBlock ( "Building a pyramid" );
  //! [exampleSurfaceMesh-make-pyramid]
  typedef SurfaceMesh< RealPoint, RealVector > MySurfaceMesh;
  typedef MySurfaceMesh::Vertices              Vertices;
  std::vector< RealPoint > positions =
    { { 0, 0, 5 }, { 1, 1, 3 }, { -1, 1, 3 }, { -1, -1, 3 }, { 1, -1, 3 } };
  std::vector< Vertices  > faces =
    { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 1 }, { 4, 3, 2, 1 } };
  auto pyramid_mesh = MySurfaceMesh( positions.cbegin(), positions.cend(),
                                     faces.cbegin(), faces.cend() );
  //! [exampleSurfaceMesh-make-pyramid]
  trace.endBlock();
  
  Mesh< RealPoint > viewmesh, viewmesh2, viewmesh3;
  MeshHelpers::surfaceMesh2Mesh( smesh,      viewmesh  );
  MeshHelpers::surfaceMesh2Mesh( torus_mesh, viewmesh2 );
  MeshHelpers::surfaceMesh2Mesh( pyramid_mesh, viewmesh3 );
  // for ( auto&& v : smesh.positions() )
  //   viewmesh.addVertex( v );
  // for ( auto&& f : smesh.allIncidentVertices() )
  //   {
  //     Mesh< RealPoint >::MeshFace face( f.cbegin(), f.cend() );
  //     viewmesh.addFace( face, Color( 200, 200, 255, 255 ) );
  //   }

  QApplication application(argc,argv);
  Viewer3D<> viewer;
  viewer.show();
  viewer << viewmesh << viewmesh2 << viewmesh3;
  viewer << Viewer3D<>::updateDisplay;
  application.exec();
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
