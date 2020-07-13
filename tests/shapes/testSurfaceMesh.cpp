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
 * @file testSurfaceMesh.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/12
 *
 * Functions for testing class SurfaceMesh.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/graph/CUndirectedSimpleGraph.h"
#include "DGtal/graph/BreadthFirstVisitor.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/MeshHelpers.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SurfaceMesh.
///////////////////////////////////////////////////////////////////////////////



SurfaceMesh< PointVector<3,double>,
             PointVector<3,double> > makeBox()
{
  typedef PointVector<3,double>                 RealPoint;
  typedef SurfaceMesh< RealPoint, RealPoint >   PolygonMesh;
  typedef PolygonMesh::Vertices                 Vertices;
  std::vector< RealPoint > positions;
  std::vector< Vertices  > faces;
  positions.push_back( RealPoint( 0, 0, 0 ) );
  positions.push_back( RealPoint( 1, 0, 0 ) );
  positions.push_back( RealPoint( 0, 1, 0 ) );
  positions.push_back( RealPoint( 1, 1, 0 ) );
  positions.push_back( RealPoint( 0, 0, 1 ) );
  positions.push_back( RealPoint( 1, 0, 1 ) );
  positions.push_back( RealPoint( 0, 1, 1 ) );
  positions.push_back( RealPoint( 1, 1, 1 ) );
  positions.push_back( RealPoint( 1, 0, 2 ) );
  positions.push_back( RealPoint( 0, 0, 2 ) );
  faces.push_back( { 1, 0, 2, 3 } );
  faces.push_back( { 0, 1, 5, 4 } );
  faces.push_back( { 1, 3, 7, 5 } );
  faces.push_back( { 3, 2, 6, 7 } );
  faces.push_back( { 2, 0, 4, 6 } );
  faces.push_back( { 4, 5, 8, 9 } );
  return PolygonMesh( positions.cbegin(), positions.cend(),
                      faces.cbegin(), faces.cend() );
}

SCENARIO( "SurfaceMesh< RealPoint3 > build tests", "[surfmesh][build]" )
{
  typedef PointVector<3,double>                 RealPoint;
  typedef SurfaceMesh< RealPoint, RealPoint >   PolygonMesh;
  typedef PolygonMesh::VertexRange              VertexRange;
  typedef PolygonMesh::Face                     Face;
  typedef PolygonMesh::Edge                     Edge;
  typedef PolygonMesh::Vertex                   Vertex;
  GIVEN( "A box with an open side" ) {
    PolygonMesh polymesh = makeBox();
    THEN( "The mesh has 10 vertices, v0 has 3 neighbors, v1 has 3 neighbors, etc" ) {
      REQUIRE( polymesh.size() == 10 );
      REQUIRE( polymesh.degree( 0 ) == 3 );
      REQUIRE( polymesh.degree( 1 ) == 3 );
      REQUIRE( polymesh.degree( 2 ) == 3 );
      REQUIRE( polymesh.degree( 3 ) == 3 );
      REQUIRE( polymesh.degree( 4 ) == 4 );
      REQUIRE( polymesh.degree( 5 ) == 4 );
      REQUIRE( polymesh.degree( 6 ) == 3 );
      REQUIRE( polymesh.degree( 7 ) == 3 );
      REQUIRE( polymesh.degree( 8 ) == 2 );
      REQUIRE( polymesh.degree( 9 ) == 2 );
    }
    THEN( "Euler number is 1 as is the Euler number of a disk." )
      {
	REQUIRE( polymesh.nbVertices() == 10 );
	REQUIRE( polymesh.nbEdges() == 15 );
	REQUIRE( polymesh.nbFaces() == 6 );
	REQUIRE( polymesh.Euler() == 1 );
      }
    THEN( "Breadth-first visiting the mesh from vertex 0, visit {0}, then {1,2,4}, then {3,5,6,9}, then {7,8}." )
      {
        BreadthFirstVisitor< PolygonMesh > visitor( polymesh, 0 );
        std::vector<unsigned long> vertices;
        std::vector<unsigned long> distances;
        while ( ! visitor.finished() )
          {
            vertices.push_back( visitor.current().first );
            distances.push_back( visitor.current().second );
            visitor.expand();
          }
        REQUIRE( vertices.size() == 10 );
        REQUIRE( distances.size() == 10 );
        int expected_vertices[] = { 0, 1, 2, 4, 3, 5, 6, 9, 7, 8 };
        int expected_distance[] = { 0, 1, 1, 1, 2, 2, 2, 2, 3, 3 };
	auto itb = vertices.begin();
	std::sort( itb+1, itb+4 );
	std::sort( itb+4, itb+8 );
	std::sort( itb+8, itb+10 );
        bool vertices_ok
          = std::equal( vertices.begin(), vertices.end(), expected_vertices );
        REQUIRE( vertices_ok );
        bool distances_ok
          = std::equal( distances.begin(), distances.end(), expected_distance );
        REQUIRE( distances_ok );
      }      
    // THEN( "The mesh has 6 boundary vertices" ) {
    //   VertexRange bv = polymesh.allBoundaryVertices();
    //   std::sort( bv.begin(), bv.end() );
    //   int expected_bv [] = { 4, 5, 6, 7, 8, 9 };
    //   REQUIRE( bv.size() == 6 );
    //   bool bv_ok = std::equal( bv.begin(), bv.end(), expected_bv );
    //   REQUIRE( bv_ok );
    // }
    // THEN( "The mesh has 6 boundary arcs" ) {
    //   ArcRange ba = polymesh.allBoundaryArcs();
    //   REQUIRE( ba.size() == 6 );
    // }
    THEN( "The face along (1,3) is a quadrangle (1,3,7,5)" ) {
      Edge e13      = polymesh.makeEdge( 1, 3 );
      auto lfs      = polymesh.edgeLeftFaces( e13 );
      VertexRange T = polymesh.incidentVertices( lfs[ 0 ] );
      REQUIRE( T.size() == 4 );
      std::sort( T.begin(), T.end() );
      REQUIRE( T[ 0 ] == 1 );
      REQUIRE( T[ 1 ] == 3 );
      REQUIRE( T[ 2 ] == 5 );
      REQUIRE( T[ 3 ] == 7 );
    }
    THEN( "The face along (3,1) is a quadrangle (3,1,0,2)" ) {
      Edge e13      = polymesh.makeEdge( 1, 3 );
      auto rfs      = polymesh.edgeRightFaces( e13 );
      VertexRange T = polymesh.incidentVertices( rfs[ 0 ] );
      REQUIRE( T.size() == 4 );
      std::sort( T.begin(), T.end() );
      REQUIRE( T[ 0 ] == 0 );
      REQUIRE( T[ 1 ] == 1 );
      REQUIRE( T[ 2 ] == 2 );
      REQUIRE( T[ 3 ] == 3 );
    }
    THEN( "The lower part of the mesh has the barycenter (0.5, 0.5, 0.5) " ) {
      auto positions = polymesh.positions();
      RealPoint b;
      for ( Vertex v = 0; v < 8; ++v )
        b += positions[ v ];
      b /= 8;
      REQUIRE( b[ 0 ] == 0.5 );
      REQUIRE( b[ 1 ] == 0.5 );
      REQUIRE( b[ 2 ] == 0.5 );
    }
    THEN( "We can iterate over the vertices" ) {
      auto positions       = polymesh.positions();
      RealPoint    exp_positions[] = { { 0,0,0 }, { 1,0,0 }, { 0,1,0 }, { 1,1,0 },
				       { 0,0,1 }, { 1,0,1 }, { 0,1,1 }, { 1,1,1 },
				       { 1,0,2 }, { 0,0,2 } };
      for ( auto it = polymesh.begin(), itE = polymesh.end(); it != itE; ++it ) {
	REQUIRE( positions[ *it ] == exp_positions[ *it ] );
      }
    }
  }
}

SCENARIO( "SurfaceMesh< RealPoint3 > concept check tests", "[surfmesh][concepts]" )
{
  typedef PointVector<3,double>               RealPoint;
  typedef SurfaceMesh< RealPoint, RealPoint > PolygonMesh;
  BOOST_CONCEPT_ASSERT(( concepts::CUndirectedSimpleGraph< PolygonMesh > ));
}
