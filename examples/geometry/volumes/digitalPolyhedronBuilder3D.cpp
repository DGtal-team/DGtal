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
 * @date 2021/06/20
 *
 * An example file named digitalPolyhedronBuilder3D
 *
 * This file is part of the DGtal library.
 */


/**
   This example shows how to use tangency to compute shortest paths on 3D digital objects
   
   @see \ref dgtal_dconvexityapp_sec2
   
   For instance, you may call it on object "cube+sphere" as

\verbatim
digitalPolyhedronBuilder3D cps.vol 0 255 0.0
\endverbatim

   The user selects two surfels (with shift + left click), and then
   shortest paths are computed and displayed.

<table>
<tr><td>
\image html cps-geodesics-1.jpg "Geodesic distances and geodesics on cube+sphere shape" width=90%
</td><td>
\image html cps-geodesics-2.jpg "Geodesic distances on cube+sphere shape" width=90%
</td><td>
\image html cps-shortest-path.jpg "Shortest path between two points" width=90%
</td></tr>
</table>

 \example geometry/volumes/digitalPolyhedronBuilder3D.cpp
 */


///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <queue>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/Viewer3D.h"
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
  trace.info() << "Usage: " << argv[ 0 ] << " <input.obj> <h>" << std::endl;
  trace.info() << "\tComputes a digital polyhedron from an OBJ file" << std::endl;
  trace.info() << "\t- input.obj: choose your favorite mesh" << std::endl;
  trace.info() << "\t- h [==1]: the digitization gridstep" << std::endl;
  string filename = examplesPath + "samples/lion.obj";
  std::string  fn = argc > 1 ? argv[ 1 ]         : filename; //< vol filename
  double        h = argc > 2 ? atof( argv[ 2 ] ) : 1.0;

  // Read OBJ file
  std::ifstream input( fn.c_str() );
  DGtal::SurfaceMesh< RealPoint, RealVector > surfmesh;
  bool ok = SurfaceMeshReader< RealPoint, RealVector >::readOBJ( input, surfmesh );
  if ( ! ok )
    {
      trace.error() << "Unable to read obj file : " << fn << std::endl;
      return 1;
    }

  QApplication application(argc,argv);
  typedef Viewer3D<Space,KSpace> MViewer;
  MViewer viewer;
  viewer.setWindowTitle("digitalPolyhedronBuilder3D");
  viewer.show();  

  Point lo(-500,-500,-500);
  Point up(500,500,500);
  DigitalConvexity< KSpace > dconv( lo, up );

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
  auto edgeFaces    = surfmesh.allEdgeFaces();

  trace.beginBlock( "Computing polyhedron" );
  for ( int f = 0; f < surfmesh.nbFaces(); ++f )
    {
      PointRange X;
      for ( auto v : faceVertices[ f ] )
        X.push_back( vertices[ v ] );
      auto F = dconv.envelope( X );
      faces_set.insert( F.cbegin(), F.cend() );
    }
  for ( int e = 0; e < surfmesh.nbEdges(); ++e )
    {
      PointRange X =
        { vertices[ edgeVertices[ e ].first  ],
          vertices[ edgeVertices[ e ].second ] };
      auto E = dconv.envelope( X );
      edges_set.insert( E.cbegin(), E.cend() );
    }
  trace.info() << "#vertex points=" << vertices.size() << std::endl;
  trace.info() << "#edge   points=" << edges_set.size() << std::endl;
  trace.info() << "#face   points=" << faces_set.size() << std::endl;
  trace.endBlock();
  std::vector< Point > face_points, edge_points;
  std::set_difference( faces_set.cbegin(), faces_set.cend(),
                       edges_set.cbegin(), edges_set.cend(),
                       std::back_inserter( face_points ) );
  std::set_difference( edges_set.cbegin(), edges_set.cend(),
                       vertices.cbegin(),  vertices.cend(),
                       std::back_inserter( edge_points ) );

  // display everything
  Color colors[] = { Color::Black, Color( 100, 100, 100 ), Color( 200, 200, 200 ) };
  viewer.setLineColor( colors[ 0 ] );
  viewer.setFillColor( colors[ 0 ] );
  for ( auto p : vertices ) viewer << p;
  viewer.setLineColor( colors[ 1 ] );
  viewer.setFillColor( colors[ 1 ] );
  for ( auto p : edge_points ) viewer << p;
  viewer.setLineColor( colors[ 2 ] );
  viewer.setFillColor( colors[ 2 ] );
  for ( auto p : face_points ) viewer << p;
  viewer << MViewer::updateDisplay;
  return application.exec();

}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

