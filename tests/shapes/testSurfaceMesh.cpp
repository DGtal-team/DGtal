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
#include <sstream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/graph/CUndirectedSimpleGraph.h"
#include "DGtal/graph/BreadthFirstVisitor.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/SurfaceMeshHelper.h"
#include "DGtal/io/readers/SurfaceMeshReader.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"
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
  typedef PointVector<3,double>                 RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >  PolygonMesh;
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


SurfaceMesh< PointVector<3,double>,
PointVector<3,double> > makeNonManifoldBoundary()
{
  typedef PointVector<3,double>                 RealPoint;
  typedef PointVector<3,double>                 RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >  PolygonMesh;
  typedef PolygonMesh::Vertices                 Vertices;
  std::vector< RealPoint > positions;
  std::vector< Vertices  > faces;
  positions.push_back( RealPoint( 0, 0, 1 ) );
  positions.push_back( RealPoint( 0, -1, 0 ) );
  positions.push_back( RealPoint( 1, 0, 0 ) );
  positions.push_back( RealPoint( 0, 1, 0 ) );
  positions.push_back( RealPoint( 0, 0, 0 ) );
  faces.push_back( { 0, 4, 1 } );
  faces.push_back( { 0, 4, 2 } );
  faces.push_back( { 0, 4, 3 } );
  return PolygonMesh( positions.cbegin(), positions.cend(),
                     faces.cbegin(), faces.cend() );
}

SurfaceMesh< PointVector<3,double>,
PointVector<3,double> > makeTetrahedron()
{
  typedef PointVector<3,double>                 RealPoint;
  typedef PointVector<3,double>                 RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >  PolygonMesh;
  typedef PolygonMesh::Vertices                 Vertices;
  std::vector< RealPoint > positions;
  std::vector< Vertices  > faces;
  positions.push_back( RealPoint( 0, 0, 0 ) );
  positions.push_back( RealPoint( 1, 0, 0 ) );
  positions.push_back( RealPoint( 0, 1, 0 ) );
  positions.push_back( RealPoint( 0, 0, 1 ) );
  faces.push_back( { 0, 1, 2 } );
  faces.push_back( { 1, 0, 3 } );
  faces.push_back( { 0, 2, 3 } );  
  faces.push_back( { 3, 2, 1 } );
  return PolygonMesh( positions.cbegin(), positions.cend(),
                     faces.cbegin(), faces.cend() );
}

SCENARIO( "SurfaceMesh< RealPoint3 > concept check tests", "[surfmesh][concepts]" )
{
  typedef PointVector<3,double>                RealPoint;
  typedef PointVector<3,double>                RealVector;
  typedef SurfaceMesh< RealPoint, RealVector > PolygonMesh;
  BOOST_CONCEPT_ASSERT(( concepts::CUndirectedSimpleGraph< PolygonMesh > ));
}

SCENARIO( "SurfaceMesh< RealPoint3 > build tests", "[surfmesh][build]" )
{
  typedef PointVector<3,double>                 RealPoint;
  typedef PointVector<3,double>                 RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >  PolygonMesh;
  typedef PolygonMesh::Vertices                 Vertices;
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
    THEN( "Checking distances." )
    {
      REQUIRE( polymesh.distance(0,0) == Approx(0.0) );
      REQUIRE( polymesh.distance(0,7) == Approx(std::sqrt(3)));
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
    THEN( "The mesh has 6 boundary edges and 9 manifold inner consistent edges, the boundary is a 1d manifold" ) {
      auto mani_bdry    = polymesh.computeManifoldBoundaryEdges();
      auto mani_inner   = polymesh.computeManifoldInnerEdges();
      auto mani_inner_c = polymesh.computeManifoldInnerConsistentEdges();
      auto mani_inner_u = polymesh.computeManifoldInnerUnconsistentEdges();
      auto non_mani     = polymesh.computeNonManifoldEdges();
      CAPTURE( polymesh );
      REQUIRE( mani_bdry.size()    == 6 );
      REQUIRE( mani_inner.size()   == 9 );
      REQUIRE( mani_inner_c.size() == 9 );
      REQUIRE( mani_inner_u.size() == 0 );
      REQUIRE( non_mani.size()     == 0 );
    }
    THEN( "The face along (1,3) is a quadrangle (1,3,7,5)" ) {
      Edge e13      = polymesh.makeEdge( 1, 3 );
      auto lfs      = polymesh.edgeLeftFaces( e13 );
      Vertices T = polymesh.incidentVertices( lfs[ 0 ] );
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
      Vertices T = polymesh.incidentVertices( rfs[ 0 ] );
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


SCENARIO( "SurfaceMesh< RealPoint3 > mesh helper tests", "[surfmesh][helper]" )
{
  typedef PointVector<3,double>                      RealPoint;
  typedef PointVector<3,double>                      RealVector;
  typedef SurfaceMeshHelper< RealPoint, RealVector > PolygonMeshHelper;
  typedef PolygonMeshHelper::NormalsType             NormalsType;
  GIVEN( "A sphere of radius 10" ) {
    auto polymesh = PolygonMeshHelper::makeSphere( 3.0, RealPoint::zero,
                                                   10, 10, NormalsType::NO_NORMALS );
    THEN( "The mesh has Euler characteristic 2" ) {
      REQUIRE( polymesh.Euler() == 2 );
    }
    THEN( "It is a consistent manifold without boundary" ) {
      auto mani_bdry    = polymesh.computeManifoldBoundaryEdges();
      auto mani_inner   = polymesh.computeManifoldInnerEdges();
      auto mani_inner_c = polymesh.computeManifoldInnerConsistentEdges();
      auto mani_inner_u = polymesh.computeManifoldInnerUnconsistentEdges();
      auto non_mani     = polymesh.computeNonManifoldEdges();
      CAPTURE( polymesh );
      REQUIRE( mani_bdry.size()    == 0 );
      REQUIRE( mani_inner.size()   == mani_inner_c.size() );
      REQUIRE( mani_inner_u.size() == 0 );
      REQUIRE( non_mani.size()     == 0 );
    }
  }
  GIVEN( "A torus with radii 3 and 1" ) {
    auto polymesh = PolygonMeshHelper::makeTorus( 3.0, 1.0, RealPoint::zero,
                                                  10, 10, 0, NormalsType::NO_NORMALS );
    THEN( "The mesh has Euler characteristic 0" ) {
      REQUIRE( polymesh.Euler() == 0 );
    }
    THEN( "It is a consistent manifold without boundary" ) {
      auto mani_bdry    = polymesh.computeManifoldBoundaryEdges();
      auto mani_inner   = polymesh.computeManifoldInnerEdges();
      auto mani_inner_c = polymesh.computeManifoldInnerConsistentEdges();
      auto mani_inner_u = polymesh.computeManifoldInnerUnconsistentEdges();
      auto non_mani     = polymesh.computeNonManifoldEdges();
      CAPTURE( polymesh );
      REQUIRE( mani_bdry.size()    == 0 );
      REQUIRE( mani_inner.size()   == mani_inner_c.size() );
      REQUIRE( mani_inner_u.size() == 0 );
      REQUIRE( non_mani.size()     == 0 );
    }
  }
  GIVEN( "A lantern with radii 3" ) {
    auto polymesh = PolygonMeshHelper::makeLantern( 3.0, 3.0, RealPoint::zero,
                                                    10, 10, NormalsType::NO_NORMALS );
    THEN( "The mesh has Euler characteristic 0" ) {
      REQUIRE( polymesh.Euler() == 0 );
    }
    THEN( "It is a consistent manifold with boundary" ) {
      auto mani_bdry    = polymesh.computeManifoldBoundaryEdges();
      auto mani_inner   = polymesh.computeManifoldInnerEdges();
      auto mani_inner_c = polymesh.computeManifoldInnerConsistentEdges();
      auto mani_inner_u = polymesh.computeManifoldInnerUnconsistentEdges();
      auto non_mani     = polymesh.computeNonManifoldEdges();
      CAPTURE( polymesh );
      REQUIRE( mani_bdry.size()    == 20 );
      REQUIRE( mani_inner.size()   == mani_inner_c.size() );
      REQUIRE( mani_inner_u.size() == 0 );
      REQUIRE( non_mani.size()     == 0 );
    }
  }
}

SCENARIO( "SurfaceMesh< RealPoint3 > reader/writer tests", "[surfmesh][io]" )
{
  typedef PointVector<3,double>                      RealPoint;
  typedef PointVector<3,double>                      RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >       PolygonMesh;
  typedef SurfaceMeshHelper< RealPoint, RealVector > PolygonMeshHelper;
  typedef SurfaceMeshReader< RealPoint, RealVector > PolygonMeshReader;
  typedef SurfaceMeshWriter< RealPoint, RealVector > PolygonMeshWriter;
  typedef PolygonMeshHelper::NormalsType             NormalsType;
  auto polymesh = PolygonMeshHelper::makeSphere( 3.0, RealPoint::zero,
                                                 10, 10, NormalsType::VERTEX_NORMALS );
  WHEN( "Writing the mesh as an OBJ file and reading into another mesh" ) { 
    PolygonMesh readmesh;
    std::ostringstream output;
    bool okw = PolygonMeshWriter::writeOBJ( output, polymesh );
    std::string file = output.str();
    std::istringstream input( file ); 
    bool okr = PolygonMeshReader::readOBJ ( input,  readmesh );
    THEN( "The read mesh is the same as the original one" ) {
      CAPTURE( file );
      CAPTURE( polymesh );
      CAPTURE( readmesh );
      REQUIRE( okw );
      REQUIRE( okr );
      REQUIRE( polymesh.Euler()      == readmesh.Euler() );
      REQUIRE( polymesh.nbVertices() == readmesh.nbVertices() );
      REQUIRE( polymesh.nbEdges()    == readmesh.nbEdges() );
      REQUIRE( polymesh.nbFaces()    == readmesh.nbFaces() );
      REQUIRE( polymesh.neighborVertices( 0 ).size()
               == readmesh.neighborVertices( 0 ).size() );
      REQUIRE( polymesh.neighborVertices( 20 ).size()
               == readmesh.neighborVertices( 20 ).size() );
      REQUIRE( polymesh.vertexNormals().size() == readmesh.vertexNormals().size() );
    }
  }
}

SCENARIO( "SurfaceMesh< RealPoint3 > boundary tests", "[surfmesh][boundary]" )
{
  auto polymesh  = makeNonManifoldBoundary();
  auto polymesh2 = makeBox();
  WHEN( "Checking the topology of the mesh boundary" ) {
    auto chains = polymesh2.computeManifoldBoundaryChains();
    THEN( "The box as a manifold boundary" ) {
      CAPTURE(chains);
      REQUIRE( polymesh2.isBoundariesManifold() == true);
      REQUIRE( polymesh2.isBoundariesManifold(false) == true);
      REQUIRE( chains.size() == 1);
      REQUIRE( chains[0].size() == 6);
    }
    THEN( "The extra mesh does not have a manifold boundary" ) {
      REQUIRE( polymesh.isBoundariesManifold() == false);
    }
  }
}

SCENARIO( "SurfaceMesh< RealPoint3 > flippable tests", "[surfmesh][flip]" )
{
  typedef PointVector<3,double>                      RealPoint;
  typedef PointVector<3,double>                      RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >       PolygonMesh;
  typedef PolygonMesh::Edge                          Edge;  
  typedef SurfaceMeshHelper< RealPoint, RealVector > PolygonMeshHelper;
  typedef PolygonMeshHelper::NormalsType             NormalsType;
  auto meshBox     = makeBox();
  auto meshTetra   = makeTetrahedron();
  auto meshLantern = PolygonMeshHelper::makeLantern( 3.0, 3.0, RealPoint::zero,
						   10, 10, NormalsType::NO_NORMALS );
  auto meshTorus   = PolygonMeshHelper::makeTorus( 3.0, 1.0, RealPoint::zero,
                                                  10, 10, 0, NormalsType::NO_NORMALS );
  WHEN( "Checking if one can flip box edges" ) {
    auto nb_flippable = 0;
    for ( Edge e = 0; e < meshBox.nbEdges(); e++ )
      if ( meshBox.isFlippable( e ) ) nb_flippable++;
    THEN( "No box edges are flippable (they border quads)" ) {
      REQUIRE( nb_flippable == 0 );
    }
  }
  WHEN( "Checking if one can flip tetrahedron edges" ) {
    auto nb_flippable = 0;
    for ( Edge e = 0; e < meshTetra.nbEdges(); e++ )
      if ( meshTetra.isFlippable( e ) ) nb_flippable++;
    THEN( "No tetrahedron edges are flippable (the neihgborhood is not simply connected)" ) {
      REQUIRE( nb_flippable == 0 );
    }
  }
  WHEN( "Checking if one can flip torus edges" ) {
    Edge nb_flippable = 0;
    for ( Edge e = 0; e < meshTorus.nbEdges(); e++ )
      if ( meshTorus.isFlippable( e ) ) nb_flippable++;
    THEN( "All torus edges are flippable (it is a closed triangulated surface)" ) {
      REQUIRE( nb_flippable == meshTorus.nbEdges() );
    }
  }
  WHEN( "Checking if one can flip lantern edges" ) {
    auto bdry_edges  = meshLantern.computeManifoldBoundaryEdges();
    auto inner_edges = meshLantern.computeManifoldInnerEdges();
    Edge nb_flippable       = 0;
    Edge nb_bdry_flippable  = 0;
    Edge nb_inner_flippable = 0;    
    for ( Edge e = 0; e < meshLantern.nbEdges(); e++ )
      if ( meshLantern.isFlippable( e ) ) nb_flippable++;
    for ( Edge e : bdry_edges )
      if ( meshLantern.isFlippable( e ) ) nb_bdry_flippable++;
    for ( Edge e : inner_edges )
      if ( meshLantern.isFlippable( e ) ) nb_inner_flippable++;
    THEN( "Innner lantern edges are flippable while boundary edges are not flippable" ) {
      REQUIRE( nb_flippable == inner_edges.size() );
      REQUIRE( nb_bdry_flippable == 0 );
      REQUIRE( nb_flippable == nb_inner_flippable );
      REQUIRE( nb_flippable == ( meshLantern.nbEdges() - bdry_edges.size() ) );
    }
  }
}

SCENARIO( "SurfaceMesh< RealPoint3 > flip tests", "[surfmesh][flip]" )
{
  typedef PointVector<3,double>                      RealPoint;
  typedef PointVector<3,double>                      RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >       PolygonMesh;
  typedef PolygonMesh::Edge                          Edge;  
  typedef SurfaceMeshHelper< RealPoint, RealVector > PolygonMeshHelper;
  typedef PolygonMeshHelper::NormalsType             NormalsType;
  auto meshLantern = PolygonMeshHelper::makeLantern( 3.0, 3.0, RealPoint::zero,
						     10, 10, NormalsType::NO_NORMALS );
  auto bdry_edges  = meshLantern.computeManifoldBoundaryEdges();
  auto euler       = meshLantern.Euler();
  auto nb_flipped  = 0;
  for ( auto i = 0; i < 100; i++ )
    {
      Edge e = rand() % meshLantern.nbEdges();
      if ( meshLantern.isFlippable( e ) )
	{
	  meshLantern.flip( e, false );
	  nb_flipped++;
	}
    }
  WHEN( "Flipping 100 random edges" ) {
    THEN( "More than 50 edges were flipped" ) {
      REQUIRE( nb_flipped > 50 );
    }
    THEN( "Euler number is not changed" ) {
      auto post_euler = meshLantern.Euler();
      REQUIRE( euler == post_euler );
    }
    THEN( "Boundary is unchanged" ) {
      auto post_bdry_edges  = meshLantern.computeManifoldBoundaryEdges();      
      REQUIRE( bdry_edges.size() == post_bdry_edges.size() );
    }
  }
}

SCENARIO( "SurfaceMesh< RealPoint3 > restore lantern with flips tests", "[surfmesh][flip]" )
{
  typedef PointVector<3,double>                      RealPoint;
  typedef PointVector<3,double>                      RealVector;
  typedef SurfaceMesh< RealPoint, RealVector >       PolygonMesh;
  typedef PolygonMesh::Edge                          Edge;
  typedef SurfaceMeshHelper< RealPoint, RealVector > PolygonMeshHelper;
  typedef SurfaceMeshWriter< RealPoint, RealVector > PolygonMeshWriter;
  typedef PolygonMeshHelper::NormalsType             NormalsType;
  auto meshLantern = PolygonMeshHelper::makeLantern( 3.0, 3.0, RealPoint::zero,
                                                     10, 10, NormalsType::NO_NORMALS );
  {
    std::ofstream output( "lantern.obj" );
    PolygonMeshWriter::writeOBJ( output, meshLantern );
    output.close();
  }
  Edge nb_flipped = 0;
  const auto&   X = meshLantern.positions();
  for ( Edge e = 0; e < meshLantern.nbEdges(); e++ )
    {
      if ( meshLantern.isFlippable( e ) )
	{
	  auto   ij    = meshLantern.edgeVertices ( e );
	  auto   kl    = meshLantern.otherDiagonal( e );
	  double l2_ij = ( X[ ij.first ] - X[ ij.second ] ).squaredNorm();
	  double l2_kl = ( X[ kl.first ] - X[ kl.second ] ).squaredNorm();
	  if ( l2_kl < l2_ij )
	    {
	      meshLantern.flip( e, false );
	      nb_flipped++;
	    }
	}
    }
  {
    std::ofstream output( "flipped-lantern.obj" );
    PolygonMeshWriter::writeOBJ( output, meshLantern );
    output.close();
  }
  WHEN( "Flipping all long edges" ) {
    THEN( "80 edges were flipped" ) {
      REQUIRE( nb_flipped == 80 );
    }
  }
}
  
