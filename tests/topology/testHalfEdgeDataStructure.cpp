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
 * @file testHalfEdgeDataStructure.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/04
 *
 * Functions for testing class HalfEdgeDataStructure.
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
#include "DGtal/topology/HalfEdgeDataStructure.h"
///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class HalfEdgeDataStructure.
///////////////////////////////////////////////////////////////////////////////
typedef HalfEdgeDataStructure::Triangle         Triangle;
typedef HalfEdgeDataStructure::PolygonalFace    PolygonalFace;
typedef HalfEdgeDataStructure::Edge             Edge;
typedef HalfEdgeDataStructure::Arc              ArcT; //Arc already defined in wingdi.h
typedef HalfEdgeDataStructure::VertexIndexRange VertexIndexRange;
typedef HalfEdgeDataStructure::Size Size;



HalfEdgeDataStructure makeTwoTriangles()
{
  std::vector< Triangle > triangles( 2 );
  triangles[0].v = { 0, 1, 2 };
  triangles[1].v = { 2, 1, 3 };

  HalfEdgeDataStructure mesh;
  mesh.build( triangles );
  return mesh;
}

HalfEdgeDataStructure makeThreeTriangles()
{
  std::vector< Triangle > triangles( 3 );
  triangles[0].v = { 0, 1, 2 };
  triangles[1].v = { 2, 1, 3 };
  triangles[2].v = { 2, 3, 0 };

  HalfEdgeDataStructure mesh;
  mesh.build( triangles );
  return mesh;
}

HalfEdgeDataStructure makeTetrahedron()
{
  std::vector< Triangle > triangles( 4 );
  triangles[0].v = { 0, 1, 2 };
  triangles[1].v = { 2, 1, 3 };
  triangles[2].v = { 2, 3, 0 };
  triangles[3].v = { 0, 3, 1 };

  HalfEdgeDataStructure mesh;
  mesh.build( triangles );
  return mesh;
}

HalfEdgeDataStructure makeOctahedron()
{
  std::vector< Triangle > triangles( 8 );
  triangles[0].v = { 0, 1, 2 };
  triangles[1].v = { 0, 2, 3 };
  triangles[2].v = { 0, 3, 4 };
  triangles[3].v = { 0, 4, 1 };
  triangles[4].v = { 5, 2, 1 };
  triangles[5].v = { 5, 3, 2 };
  triangles[6].v = { 5, 4, 3 };
  triangles[7].v = { 5, 1, 4 };
  HalfEdgeDataStructure mesh;
  mesh.build( triangles );
  return mesh;
}

HalfEdgeDataStructure makeRibbonWithHole()
{
  std::vector< Triangle > triangles( 6 );
  triangles[0].v = { 0, 1, 2 };
  triangles[1].v = { 2, 1, 3 };
  triangles[2].v = { 2, 3, 4 };
  triangles[3].v = { 4, 3, 5 };
  triangles[4].v = { 4, 5, 0 };
  triangles[5].v = { 0, 5, 1 };
  std::vector< Edge > edges;
  const auto kNumVertices
    = HalfEdgeDataStructure::getUnorderedEdgesFromTriangles( triangles, edges );
  HalfEdgeDataStructure mesh;
  mesh.build( kNumVertices, triangles, edges );
  return mesh;
}

HalfEdgeDataStructure makeTriangulatedDisk()
{
  std::vector< Triangle > triangles( 7 );
  triangles[0].v = { 0, 1, 2 };
  triangles[1].v = { 2, 1, 3 };
  triangles[2].v = { 2, 3, 4 };
  triangles[3].v = { 4, 3, 5 };
  triangles[4].v = { 4, 5, 0 };
  triangles[5].v = { 0, 5, 1 };
  triangles[6].v = { 4, 0, 2 };
  std::vector< Edge > edges;
  const auto kNumVertices
    = HalfEdgeDataStructure::getUnorderedEdgesFromTriangles( triangles, edges );
  HalfEdgeDataStructure mesh;
  mesh.build( kNumVertices, triangles, edges );
  return mesh;
}

HalfEdgeDataStructure makePyramid()
{
  std::vector< PolygonalFace > faces( 5 );
  faces[ 0 ] = PolygonalFace( { 0, 3, 2, 1 } );
  faces[ 1 ] = PolygonalFace( { 0, 1, 4 } );
  faces[ 2 ] = PolygonalFace( { 1, 2, 4 } );
  faces[ 3 ] = PolygonalFace( { 2, 3, 4 } );
  faces[ 4 ] = PolygonalFace( { 3, 0, 4 } );
  HalfEdgeDataStructure mesh;
  mesh.build( faces );
  return mesh;
}

HalfEdgeDataStructure makeCube()
{
  std::vector< PolygonalFace > faces( 6 );
  faces[ 0 ] = PolygonalFace( { 1, 0, 2, 3 } );
  faces[ 1 ] = PolygonalFace( { 0, 1, 5, 4 } );
  faces[ 2 ] = PolygonalFace( { 1, 3, 7, 5 } );
  faces[ 3 ] = PolygonalFace( { 3, 2, 6, 7 } );
  faces[ 4 ] = PolygonalFace( { 2, 0, 4, 6 } );
  faces[ 5 ] = PolygonalFace( { 4, 5, 7, 6 } );
  HalfEdgeDataStructure mesh;
  mesh.build( faces );
  return mesh;
}

HalfEdgeDataStructure makeBox()
{
  std::vector< PolygonalFace > faces( 6 );
  faces[ 0 ] = PolygonalFace( { 1, 0, 2, 3 } );
  faces[ 1 ] = PolygonalFace( { 0, 1, 5, 4 } );
  faces[ 2 ] = PolygonalFace( { 1, 3, 7, 5 } );
  faces[ 3 ] = PolygonalFace( { 3, 2, 6, 7 } );
  faces[ 4 ] = PolygonalFace( { 2, 0, 4, 6 } );
  faces[ 5 ] = PolygonalFace( { 4, 5, 8, 9 } );
  HalfEdgeDataStructure mesh;
  mesh.build( faces );
  return mesh;
}


SCENARIO( "HalfEdgeDataStructure build", "[halfedge][build]" )
{
  GIVEN( "Two triangles incident by an edge" ) {
    HalfEdgeDataStructure mesh = makeTwoTriangles();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "The mesh has 4 vertices, 5 edges, 2 faces, 10 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  4 );
      REQUIRE( mesh.nbEdges()     ==  5 );
      REQUIRE( mesh.nbFaces()     ==  2 );
      REQUIRE( mesh.nbHalfEdges() == 10 );
    }
    THEN( "The mesh has 4 boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      std::sort( bdry.begin(), bdry.end() );
      REQUIRE( bdry.size() == 4 );
      REQUIRE( bdry[ 0 ] == 0 );
      REQUIRE( bdry[ 1 ] == 1 );
      REQUIRE( bdry[ 2 ] == 2 );
      REQUIRE( bdry[ 3 ] == 3 );
    }
    THEN( "The mesh has 4 boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      std::sort( bdry.begin(), bdry.end() );
      REQUIRE( bdry.size() == 4 );
      // std::cout << " arc=(" << bdry[ 0 ].first << "," << bdry[ 0 ].second << ")" << std::endl;
      REQUIRE( bdry[ 0 ] == ArcT( 0, 2 ) );
      REQUIRE( bdry[ 1 ] == ArcT( 1, 0 ) );
      REQUIRE( bdry[ 2 ] == ArcT( 2, 3 ) );
      REQUIRE( bdry[ 3 ] == ArcT( 3, 1 ) );
    }
  }
  GIVEN( "Three triangles forming a fan around a vertex" ) {
    HalfEdgeDataStructure mesh = makeThreeTriangles();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "The mesh has 4 vertices, 6 edges, 3 faces, 12 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  4 );
      REQUIRE( mesh.nbEdges()     ==  6 );
      REQUIRE( mesh.nbFaces()     ==  3 );
      REQUIRE( mesh.nbHalfEdges() == 12 );
    }
    THEN( "The mesh has 3 boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      std::sort( bdry.begin(), bdry.end() );
      REQUIRE( bdry.size() == 3 );
      REQUIRE( bdry[ 0 ] == 0 );
      REQUIRE( bdry[ 1 ] == 1 );
      REQUIRE( bdry[ 2 ] == 3 );
    }
    THEN( "The mesh has 3 boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      std::sort( bdry.begin(), bdry.end() );
      REQUIRE( bdry.size() == 3 );
      // std::cout << " arc=(" << bdry[ 0 ].first << "," << bdry[ 0 ].second << ")" << std::endl;
      REQUIRE( bdry[ 0 ] == ArcT( 0, 3 ) );
      REQUIRE( bdry[ 1 ] == ArcT( 1, 0 ) );
      REQUIRE( bdry[ 2 ] == ArcT( 3, 1 ) );
    }
  }
  GIVEN( "Four triangles forming a tetrahedron" ) {
    HalfEdgeDataStructure mesh = makeTetrahedron();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "The mesh has 4 vertices, 6 edges, 4 faces, 12 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  4 );
      REQUIRE( mesh.nbEdges()     ==  6 );
      REQUIRE( mesh.nbFaces()     ==  4 );
      REQUIRE( mesh.nbHalfEdges() == 12 );
    }
    THEN( "The mesh has no boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      REQUIRE( bdry.size() == 0 );
    }
    THEN( "The mesh has no boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      REQUIRE( bdry.size() == 0 );
    }
  }
  GIVEN( "A ribbon with a hole" ) {
    HalfEdgeDataStructure mesh = makeRibbonWithHole();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh has 6 vertices, 12 edges, 6 faces, 24 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  6 );
      REQUIRE( mesh.nbEdges()     ==  12 );
      REQUIRE( mesh.nbFaces()     ==  6 );
      REQUIRE( mesh.nbHalfEdges() == 24 );
    }
    THEN( "The mesh has 6 boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      REQUIRE( bdry.size() == 6 );
    }
    THEN( "The mesh has 6 boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      std::sort( bdry.begin(), bdry.end() );
      REQUIRE( bdry.size() == 6 );
      REQUIRE( bdry[ 0 ] == ArcT( 0, 2 ) );
      REQUIRE( bdry[ 1 ] == ArcT( 1, 5 ) );
      REQUIRE( bdry[ 2 ] == ArcT( 2, 4 ) );
      REQUIRE( bdry[ 3 ] == ArcT( 3, 1 ) );
      REQUIRE( bdry[ 4 ] == ArcT( 4, 0 ) );
      REQUIRE( bdry[ 5 ] == ArcT( 5, 3 ) );
    }
  }
  GIVEN( "The same ribbon with his hole closed" ) {
    HalfEdgeDataStructure mesh = makeTriangulatedDisk();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "The mesh has 6 vertices, 12 edges, 7 faces, 24 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  6 );
      REQUIRE( mesh.nbEdges()     ==  12 );
      REQUIRE( mesh.nbFaces()     ==  7 );
      REQUIRE( mesh.nbHalfEdges() == 24 );
    }
    THEN( "The mesh has 3 boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      std::sort( bdry.begin(), bdry.end() );
      REQUIRE( bdry.size() == 3 );
      REQUIRE( bdry[ 0 ] == 1 );
      REQUIRE( bdry[ 1 ] == 3 );
      REQUIRE( bdry[ 2 ] == 5 );
    }
    THEN( "The mesh has 3 boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      std::sort( bdry.begin(), bdry.end() );
      REQUIRE( bdry.size() == 3 );
      REQUIRE( bdry[ 0 ] == ArcT( 1, 5 ) );
      REQUIRE( bdry[ 1 ] == ArcT( 3, 1 ) );
      REQUIRE( bdry[ 2 ] == ArcT( 5, 3 ) );
    }
  }
  GIVEN( "A pyramid with a square base" ) {
    HalfEdgeDataStructure mesh = makePyramid();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh has 5 vertices, 8 edges, 5 faces, 16 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  5 );
      REQUIRE( mesh.nbEdges()     ==  8 );
      REQUIRE( mesh.nbFaces()     ==  5 );
      REQUIRE( mesh.nbHalfEdges() == 16 );
    }
    THEN( "The mesh has 0 boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      REQUIRE( bdry.size() == 0 );
    }
    THEN( "The mesh has 0 boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      REQUIRE( bdry.size() == 0 );
    }
  }
  GIVEN( "A cube" ) {
    HalfEdgeDataStructure mesh = makeCube();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh has 8 vertices, 12 edges, 6 faces, 24 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  8 );
      REQUIRE( mesh.nbEdges()     ==  12 );
      REQUIRE( mesh.nbFaces()     ==  6 );
      REQUIRE( mesh.nbHalfEdges() == 24 );
    }
    THEN( "The mesh has 0 boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      REQUIRE( bdry.size() == 0 );
    }
    THEN( "The mesh has 0 boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      REQUIRE( bdry.size() == 0 );
    }
  }
  GIVEN( "A box with an open side" ) {
    HalfEdgeDataStructure mesh = makeBox();
    THEN( "The mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh has 10 vertices, 15 edges, 6 faces, 30 half-edges" ) {
      REQUIRE( mesh.nbVertices()  ==  10 );
      REQUIRE( mesh.nbEdges()     ==  15 );
      REQUIRE( mesh.nbFaces()     ==  6 );
      REQUIRE( mesh.nbHalfEdges() == 30 );
    }
    THEN( "The mesh has 6 boundary vertices" ) {
      VertexIndexRange bdry = mesh.boundaryVertices();
      REQUIRE( bdry.size() == 6 );
    }
    THEN( "The mesh has 6 boundary arcs" ) {
      std::vector<ArcT> bdry = mesh.boundaryArcs();
      REQUIRE( bdry.size() == 6 );
    }
  }

}

SCENARIO( "HalfEdgeDataStructure neighboring relations", "[halfedge][neighbors]" ){
  GIVEN( "Two triangles incident by an edge" ) {
    HalfEdgeDataStructure mesh = makeTwoTriangles();
    VertexIndexRange nv;
    THEN( "Vertex 0 has 2 neighboring vertices" ) {
      mesh.getNeighboringVertices( 0, nv );
      VertexIndexRange expected = { 1, 2 };
      REQUIRE( nv.size()  ==  2 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 1 has 3 neighboring vertices" ) {
      mesh.getNeighboringVertices( 1, nv );
      VertexIndexRange expected = { 3, 2, 0 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 2 has 3 neighboring vertices" ) {
      mesh.getNeighboringVertices( 2, nv );
      VertexIndexRange expected = { 0, 1, 3 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 3 has 2 neighboring vertices" ) {
      mesh.getNeighboringVertices( 3, nv );
      VertexIndexRange expected = { 2, 1 };
      REQUIRE( nv.size()  ==  2 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
  }
  GIVEN( "A ribbon with a hole" ) {
    HalfEdgeDataStructure mesh = makeRibbonWithHole();
    VertexIndexRange nv;
    THEN( "Vertex 0 has 4 neighboring vertices" ) {
      mesh.getNeighboringVertices( 0, nv );
      VertexIndexRange expected = { 4, 5, 1, 2 };
      REQUIRE( nv.size()  ==  4 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 1 has 4 neighboring vertices" ) {
      mesh.getNeighboringVertices( 1, nv );
      VertexIndexRange expected = { 3, 2, 0, 5 };
      REQUIRE( nv.size()  ==  4 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 2 has 4 neighboring vertices" ) {
      mesh.getNeighboringVertices( 2, nv );
      VertexIndexRange expected = { 0, 1, 3, 4 };
      REQUIRE( nv.size()  ==  4 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
  }

  GIVEN( "A box with an open side" ) {
    HalfEdgeDataStructure mesh = makeBox();
    VertexIndexRange nv;
    THEN( "Vertex 0 has 3 neighboring vertices" ) {
      mesh.getNeighboringVertices( 0, nv );
      VertexIndexRange expected = { 1, 4, 2 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 5 has 4 neighboring vertices" ) {
      mesh.getNeighboringVertices( 5, nv );
      VertexIndexRange expected = { 8, 4, 1, 7 };
      REQUIRE( nv.size()  ==  4 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 7 has 3 neighboring vertices" ) {
      mesh.getNeighboringVertices( 7, nv );
      VertexIndexRange expected = { 5, 3, 6 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
  }
}

SCENARIO( "HalfEdgeDataStructure flips", "[halfedge][flips]" ){
  GIVEN( "Two triangles incident by an edge" ) {
    HalfEdgeDataStructure mesh = makeTwoTriangles();
    THEN( "Only one edge is flippable" ) {
      int nbflippable = 0;
      for ( Size e = 0; e < mesh.nbEdges(); e++ )
        {
          if ( mesh.isFlippable( mesh.halfEdgeIndexFromEdgeIndex( e ) ) )
            nbflippable++;
        }
      REQUIRE( nbflippable == 1 );
    }
  }
  GIVEN( "A pyramid" ) {
    HalfEdgeDataStructure mesh = makePyramid();
    THEN( "Only four edges are flippable" ) {
      int nbflippable = 0;
      for ( Size e = 0; e < mesh.nbEdges(); e++ )
        {
          if ( mesh.isFlippable( mesh.halfEdgeIndexFromEdgeIndex( e ) ) )
            nbflippable++;
        }
      REQUIRE( nbflippable == 4 );
    }
  }
  GIVEN( "A tetrahedron" ) {
    HalfEdgeDataStructure mesh = makeTetrahedron();
    THEN( "No edges are flippable" ) {
      int nbflippable = 0;
      for ( Size e = 0; e < mesh.nbEdges(); e++ )
        {
          if ( mesh.isFlippable( mesh.halfEdgeIndexFromEdgeIndex( e ) ) )
            nbflippable++;
        }
      REQUIRE( nbflippable == 0 );
    }
  }
  GIVEN( "Two triangles incident by an edge" ) {
    HalfEdgeDataStructure mesh = makeTwoTriangles();
    auto he = mesh.halfEdgeIndexFromArc( {1,2} );
    REQUIRE( mesh.isFlippable( he ) );
    mesh.flip( he );
    THEN( "The mesh is valid after flip" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation after flip" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "Vertex 0 has 2,3,1 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 0 );
      VertexIndexRange expected = { 2, 3, 1 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 1 has 0,3 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 1 );
      VertexIndexRange expected = { 0, 3 };
      REQUIRE( nv.size()  ==  2 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 2 has 3,0 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 2 );
      VertexIndexRange expected = { 3, 0 };
      REQUIRE( nv.size()  ==  2 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 3 has 2,0,1 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 3 );
      VertexIndexRange expected = { 1, 0, 2 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
  }
  GIVEN( "Two triangles incident by an edge" ) {
    HalfEdgeDataStructure mesh = makeTwoTriangles();
    auto he  = mesh.findHalfEdgeIndexFromArc( {1,2} );
    REQUIRE( mesh.isFlippable( he ) );
    mesh.flip( he );
    auto he2 = mesh.findHalfEdgeIndexFromArc( {0,3} );
    REQUIRE( mesh.isFlippable( he2 ) );
    mesh.flip( he2 );
    THEN( "The mesh is valid after two flips" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation after two flips" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "Vertex 0 has 2,1 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 0 );
      VertexIndexRange expected = { 2, 1 };
      REQUIRE( nv.size()  ==  2 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 1 has 0,2,3 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 1 );
      VertexIndexRange expected = { 0,2,3 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 2 has 3,1,0 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 2 );
      VertexIndexRange expected = { 3, 1, 0 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
    THEN( "Vertex 3 has 1,2 as neighbors after flip" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 3 );
      VertexIndexRange expected = { 1, 2 };
      REQUIRE( nv.size()  ==  2 );
      REQUIRE( std::equal( nv.begin(), nv.end(), expected.begin() ) );
    }
  }
}

SCENARIO( "HalfEdgeDataStructure splits", "[halfedge][splits]" ){
  GIVEN( "Two triangles incident by an edge" ) {
    HalfEdgeDataStructure mesh = makeTwoTriangles();
    auto he  = mesh.findHalfEdgeIndexFromArc( {1,2} );
    REQUIRE( mesh.isFlippable( he ) );
    //auto vtx =
    mesh.split( he );
    THEN( "After split, mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation after split" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "After split, mesh has 5 vertices, 8 edges, 4 faces" ) {
      REQUIRE( mesh.nbVertices() == 5 );
      REQUIRE( mesh.nbEdges() == 8 );
      REQUIRE( mesh.nbFaces() == 4 );
    }
    THEN( "After split, vertex 4 has 4 neighbors { 0,1,2,3 }" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 4 );
      VertexIndexRange expected = { 0, 1, 2, 3 };
      REQUIRE( nv.size()  ==  4 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
  }
}

SCENARIO( "HalfEdgeDataStructure merges", "[halfedge][merges]" ){
  GIVEN( "An octahedron" ) {
    HalfEdgeDataStructure mesh = makeOctahedron();
    auto he  = mesh.findHalfEdgeIndexFromArc( {1,2} );
    REQUIRE( mesh.isMergeable( he ) );
    auto vtx = mesh.merge( he );
    THEN( "After merge, mesh is valid" ) {
      REQUIRE( mesh.isValid() );
    }
    THEN( "The mesh is a valid triangulation after merge" ) {
      REQUIRE( mesh.isValidTriangulation() );
    }
    THEN( "After merge, merged vertex is 1" ) {
      REQUIRE( vtx == 1 );
    }
    THEN( "After merge, mesh has 5 vertices, 9 edges, 6 faces" ) {
      REQUIRE( mesh.nbVertices() == 5 );
      REQUIRE( mesh.nbEdges() == 9 );
      REQUIRE( mesh.nbFaces() == 6 );
    }
    THEN( "After merge, vertex 0 has 3 neighbors { 1,3,4 }" ) {
      VertexIndexRange nv = mesh.neighboringVertices( 0 );
      VertexIndexRange expected = { 1, 3, 4 };
      REQUIRE( nv.size()  ==  3 );
      REQUIRE( std::is_permutation( nv.begin(), nv.end(), expected.begin() ) );
    }
  }
}

/** @ingroup Tests **/
