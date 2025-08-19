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
 * @file geometry/volumes/standardDigitalPolyhedronBuilder3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/06/20
 *
 * An example file named standardDigitalPolyhedronBuilder3D
 *
 * This file is part of the DGtal library.
 */

namespace DGtal {
/**
   This example shows how to use the fully convex envelope to build a
   digital polyhedron from an arbitrary mesh. All faces have also the
   property that their points lies in the naive/standard plane defined
   by its vertices. It uses DigitalConvexity::relativeEnvelope for
   computations.
   
   @see \ref dgtal_envelope_sec2
   
   For instance, you may call it on object "lion-tri.obj" as

\verbatim
standardDigitalPolyhedronBuilder3D ../examples/samples/lion-tri.obj 0.5 31
\endverbatim

   The last parameter specifies whether you want to see vertices (1)
   in black, edges common to both faces (2) in magenta, part of edges
   that are only on one face (4) and (8) (red on one side, blue on the
   other) and faces (16) in grey, or any combination.

<table>
<tr><td>
\image html lion-tri-h0_5-sstd-all.jpg "(Symmetric) standard Digital polyhedral model of 'lion-tri.obj' at gridstep 0.5" width=90%
</td><td>
\image html lion-tri-h0_5-sstd-edges.jpg "(Symmetric) standard Digital polyhedral model of 'lion-tri.obj' at gridstep 0.5 (vertices and edges only)" width=90%
</td></tr>
</table>

 \example geometry/volumes/standardDigitalPolyhedronBuilder3D.cpp
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

// Convenient class to represent different types of arithmetic planes as a predicate.
template < bool Naive, bool Symmetric >
struct MedianPlane {
  Vector  N;
  Integer mu;
  Integer omega;
  MedianPlane() = default;
  MedianPlane( const MedianPlane& other ) = default;
  MedianPlane( MedianPlane&& other ) = default;
  MedianPlane& operator=( const MedianPlane& other ) = default;
  MedianPlane& operator=( MedianPlane&& other ) = default;
  MedianPlane( Point p, Point q, Point r )
    : N ( ( q - p ).crossProduct( r - p ) )
  {
    mu = N.dot( p );
    omega = Naive ? N.norm( N.L_infty ) : N.norm( N.L_1 );
    if ( Symmetric && ( ( omega & 1 ) == 0 ) ) omega += 1;
    mu -= omega / 2;
  }
  bool operator()( const Point& p ) const
  {
    auto r = N.dot( p );
    return ( mu <= r ) && ( r < mu+omega );
  }
};

// Choose your plane !
// typedef MedianPlane< true,  false > Plane; //< Naive, thinnest possible
// typedef MedianPlane< true,  true  > Plane; //< Naive, Symmetric
// typedef MedianPlane< false, false > Plane; //< Standard
typedef MedianPlane< false, true  > Plane; //< Standard, Symmetric, thickest here

int main( int argc, char** argv )
{
  trace.info() << "Usage: " << argv[ 0 ] << " <input.obj> <h> <view>" << std::endl;
  trace.info() << "\tComputes a digital polyhedron from an OBJ file" << std::endl;
  trace.info() << "\t- input.obj: choose your favorite mesh" << std::endl;
  trace.info() << "\t- h [==1]: the digitization gridstep" << std::endl;
  trace.info() << "\t- view [==31]: display vertices(1), common edges(2), positive side f edges(4), negative side f edges (8), faces(16)" << std::endl;
  string filename = examplesPath + "samples/lion.obj";
  std::string  fn = argc > 1 ? argv[ 1 ]         : filename; //< vol filename
  double        h = argc > 2 ? atof( argv[ 2 ] ) : 1.0;
  int        view = argc > 3 ? atoi( argv[ 3 ] ) : 31;
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
  std::set< Point > faces_set, pos_edges_set, neg_edges_set;
  auto faceVertices = surfmesh.allIncidentVertices();

  trace.beginBlock( "Checking face planarity" );
  std::vector< Plane > face_planes;
  face_planes.resize( surfmesh.nbFaces() );
  bool planarity = true;
  for ( int f = 0; f < surfmesh.nbFaces() && planarity; ++f )
    {
      PointRange X;
      for ( auto v : faceVertices[ f ] )
        X.push_back( vertices[ v ] );
      face_planes[ f ] = Plane( X[ 0 ], X[ 1 ], X[ 2 ] );
      for ( int v = 3; v < X.size(); v++ )
        if ( ! face_planes[ f ]( X[ v ] ) )
          {
            trace.error() << "Face " << f << " is not planar." << std::endl;
            planarity = false; break;
          }
    }
  trace.endBlock();
  if ( ! planarity ) return 1;
  trace.beginBlock( "Computing polyhedron" );
  for ( int f = 0; f < surfmesh.nbFaces(); ++f )
    {
      PointRange X;
      for ( auto v : faceVertices[ f ] )
        X.push_back( vertices[ v ] );
      auto F = dconv.relativeEnvelope( X, face_planes[ f ], Algorithm::DIRECT );
      faces_set.insert( F.cbegin(), F.cend() );
      for ( int i = 0; i < X.size(); i++ )
        {
          PointRange Y { X[ i ], X[ (i+1)%X.size() ] };
          if ( Y[ 1 ] < Y[ 0 ] ) std::swap( Y[ 0 ], Y[ 1 ] ); 
          int idx1 = faceVertices[ f ][ i ];
          int idx2 = faceVertices[ f ][ (i+1)%X.size() ];
          // Variant (1): edges of both sides have many points in common
          // auto A = dconv.relativeEnvelope( Y, face_planes[ f ], Algorithm::DIRECT );
          // Variant (2): edges of both sides have much less points in common
          auto A = dconv.relativeEnvelope( Y, F, Algorithm::DIRECT );
          bool pos = idx1 < idx2;
          (pos ? pos_edges_set : neg_edges_set).insert( A.cbegin(), A.cend() );
        }
    }
  trace.endBlock();
  std::vector< Point > face_points, common_edge_points, arc_points, final_arc_points ;
  std::vector< Point > pos_edge_points, neg_edge_points, both_edge_points;
  std::vector< Point > vertex_points = vertices;
  std::sort( vertex_points.begin(), vertex_points.end() );
  std::set_symmetric_difference( pos_edges_set.cbegin(), pos_edges_set.cend(),
                                 neg_edges_set.cbegin(), neg_edges_set.cend(),
                                 std::back_inserter( arc_points ) );
  std::set_intersection( pos_edges_set.cbegin(), pos_edges_set.cend(),
                         neg_edges_set.cbegin(), neg_edges_set.cend(),
                         std::back_inserter( common_edge_points ) );
  std::set_union( pos_edges_set.cbegin(), pos_edges_set.cend(),
                  neg_edges_set.cbegin(), neg_edges_set.cend(),
                  std::back_inserter( both_edge_points ) );
  std::set_difference( faces_set.cbegin(), faces_set.cend(),
                       both_edge_points.cbegin(), both_edge_points.cend(),
                       std::back_inserter( face_points ) );
  std::set_difference( pos_edges_set.cbegin(), pos_edges_set.cend(),
                       common_edge_points.cbegin(), common_edge_points.cend(),
                       std::back_inserter( pos_edge_points ) );
  std::set_difference( neg_edges_set.cbegin(), neg_edges_set.cend(),
                       common_edge_points.cbegin(), common_edge_points.cend(),
                       std::back_inserter( neg_edge_points ) );
  std::set_difference( common_edge_points.cbegin(), common_edge_points.cend(),
                       vertex_points.cbegin(), vertex_points.cend(),
                       std::back_inserter( final_arc_points ) );
  auto total = vertex_points.size() + pos_edge_points.size()
    + neg_edge_points.size()
    + final_arc_points.size() + face_points.size();
  trace.info() << "#vertex   points=" << vertex_points.size() << std::endl;
  trace.info() << "#pos edge points=" << pos_edge_points.size() << std::endl;
  trace.info() << "#neg edge points=" << neg_edge_points.size() << std::endl;
  trace.info() << "#arc      points=" << final_arc_points.size() << std::endl;
  trace.info() << "#face     points=" << face_points.size() << std::endl;
  trace.info() << "#total    points=" << total << std::endl;

  // display everything
  Color colors[] =
    { Color::Black, Color::Blue, Color::Red,
      Color::Magenta, Color( 200, 200, 200 ) };
  if ( view & 0x1 )
    {
      viewer.drawColor( colors[ 0 ] );
      viewer.drawColor( colors[ 0 ] );
      for ( auto p : vertices ) viewer << p;
    }
  if ( view & 0x2 )
    {
      viewer.drawColor( colors[ 3 ] );
      viewer.drawColor( colors[ 3 ] );
      for ( auto p : final_arc_points ) viewer << p;
    }
  if ( view & 0x4 )
    {
      viewer.drawColor( colors[ 1 ] );
      viewer.drawColor( colors[ 1 ] );
      for ( auto p : pos_edge_points ) viewer << p;
    }
  if ( view & 0x8 )
    {
      viewer.drawColor( colors[ 2 ] );
      viewer.drawColor( colors[ 2 ] );
      for ( auto p : neg_edge_points ) viewer << p;
    }
  if ( view & 0x10 )
    {
      viewer.drawColor( colors[ 4 ] );
      viewer.drawColor( colors[ 4 ] );
      for ( auto p : face_points ) viewer << p;
    }

  viewer.show();  
  return 0;

}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

