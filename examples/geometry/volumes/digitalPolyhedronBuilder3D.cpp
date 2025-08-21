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
 * @file geometry/volumes/digitalPolyhedronBuilder3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/06/20
 *
 * An example file named digitalPolyhedronBuilder3D
 *
 * This file is part of the DGtal library.
 */

namespace DGtal {
/**
   This example shows how to use the fully convex envelope to build a
   digital polyhedron from an arbitrary mesh. It uses
   DigitalConvexity::envelope for computations.
   
   @see \ref dgtal_envelope_sec2
   
   For instance, you may call it on object "spot.obj" as

\verbatim
digitalPolyhedronBuilder3D ../examples/samples/spot.obj 0.005 7
\endverbatim

   The last parameter specifies whether you want to see vertices (1),
   edges (2) and faces (4), or any combination.

<table>
<tr><td>
\image html spot-h0_005-all.jpg "Digital polyhedral model of 'spot.obj' at gridstep 0.005" width=90%
</td><td>
\image html spot-h0_005-edges.jpg "Digital polyhedral model of 'spot.obj' at gridstep 0.005 (vertices and edges only)" width=90%
</td></tr>
</table>

 \example geometry/volumes/digitalPolyhedronBuilder3D.cpp
 */
} // namespace DGtal {

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <queue>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/io/readers/SurfaceMeshReader.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
#include "ConfigExamples.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
typedef Z3i::Space          Space;
typedef Z3i::Integer        Integer;
typedef Z3i::KSpace         KSpace;
typedef Z3i::Domain         Domain;
typedef Z3i::SCell          SCell;
typedef Space::Point        Point;
typedef Space::RealPoint    RealPoint;
typedef Space::RealVector   RealVector;
typedef Space::Vector       Vector;
typedef std::vector<Point>  PointRange;

int main( int argc, char** argv )
{
  trace.info() << "Usage: " << argv[ 0 ] << " <input.obj> <h> <view>" << std::endl;
  trace.info() << "\tComputes a digital polyhedron from an OBJ file" << std::endl;
  trace.info() << "\t- input.obj: choose your favorite mesh" << std::endl;
  trace.info() << "\t- h [==1]: the digitization gridstep" << std::endl;
  trace.info() << "\t- view [==7]: display vertices(1), edges(2), faces(4)" << std::endl;
  string filename = examplesPath + "samples/lion.obj";
  std::string  fn = argc > 1 ? argv[ 1 ]         : filename; //< vol filename
  double        h = argc > 2 ? atof( argv[ 2 ] ) : 1.0;
  int        view = argc > 3 ? atoi( argv[ 3 ] ) : 7;
  // Read OBJ file
  std::ifstream input( fn.c_str() );
  DGtal::SurfaceMesh< RealPoint, RealVector > surfmesh;
  bool ok = SurfaceMeshReader< RealPoint, RealVector >::readOBJ( input, surfmesh );
  if ( ! ok )
    {
      trace.error() << "Unable to read obj file : " << fn << std::endl;
      return 1;
    }

  typedef PolyscopeViewer<Space,KSpace> MViewer;
  MViewer viewer;

  Point lo(-500,-500,-500);
  Point up(500,500,500);
  DigitalConvexity< KSpace > dconv( lo, up );
  typedef DigitalConvexity< KSpace >::EnvelopeAlgorithm Algorithm;
  
  auto vertices = std::vector<Point>( surfmesh.nbVertices() );
  for ( auto v : surfmesh )
    {
      RealPoint p = (1.0 / h) * surfmesh.position( v );
      Point q ( (Integer) round( p[ 0 ] ),
                (Integer) round( p[ 1 ] ),
                (Integer) round( p[ 2 ] ) );
      vertices[ v ] = q;
    }
  std::set< Point > faces_set, edges_set;
  auto faceVertices = surfmesh.allIncidentVertices();
  auto edgeVertices = surfmesh.allEdgeVertices();

  trace.beginBlock( "Computing polyhedron" );
  for ( size_t f = 0; f < surfmesh.nbFaces(); ++f )
    {
      PointRange X;
      for ( auto v : faceVertices[ f ] )
        X.push_back( vertices[ v ] );
      auto F = dconv.envelope( X, Algorithm::DIRECT );
      faces_set.insert( F.cbegin(), F.cend() );
    }
  for ( size_t e = 0; e < surfmesh.nbEdges(); ++e )
    {
      PointRange X =
        { vertices[ edgeVertices[ e ].first  ],
          vertices[ edgeVertices[ e ].second ] };
      auto E = dconv.envelope( X, Algorithm::DIRECT );
      edges_set.insert( E.cbegin(), E.cend() );
    }
  trace.endBlock();
  std::vector< Point > face_points, edge_points;
  std::vector< Point > vertex_points = vertices;
  std::sort( vertex_points.begin(), vertex_points.end() );
  std::set_difference( faces_set.cbegin(), faces_set.cend(),
                       edges_set.cbegin(), edges_set.cend(),
                       std::back_inserter( face_points ) );
  std::set_difference( edges_set.cbegin(), edges_set.cend(),
                       vertex_points.cbegin(), vertex_points.cend(),
                       std::back_inserter( edge_points ) );
  auto total = vertex_points.size() + edge_points.size() + face_points.size();
  trace.info() << "#vertex points=" << vertex_points.size() << std::endl;
  trace.info() << "#edge   points=" << edge_points.size() << std::endl;
  trace.info() << "#face   points=" << face_points.size() << std::endl;
  trace.info() << "#total  points=" << total << std::endl;

  // display everything
  Color colors[] = { Color::Black, Color( 100, 100, 100 ), Color( 200, 200, 200 ) };
  if ( view & 0x1 )
    {
      viewer.drawColor( colors[ 0 ] );
      viewer.drawColor( colors[ 0 ] );
      for ( auto p : vertices ) viewer << p;
    }
  if ( view & 0x2 )
    {
      viewer.drawColor( colors[ 1 ] );
      viewer.drawColor( colors[ 1 ] );
      for ( auto p : edge_points ) viewer << p;
    }
  if ( view & 0x4 )
    {
      viewer.drawColor( colors[ 2 ] );
      viewer.drawColor( colors[ 2 ] );
      for ( auto p : face_points ) viewer << p;
    }

  viewer.show();  
  return 0;

}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

