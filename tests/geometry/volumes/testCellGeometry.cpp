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
 * @file testCellGeometry.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/01/04
 *
 * Functions for testing class CellGeometry.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/volumes/CellGeometry.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CellGeometry.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "CellGeometry< Z2 > segment tests", "[cell_geometry][2d][segment]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef CellGeometry< KSpace >           CGeometry;
  typedef DigitalConvexity< KSpace >       DConvexity;

  KSpace K;
  K.init( Point( -5, -5 ), Point( 10, 10 ), true );
  DConvexity dconv( K );
  GIVEN( "two points (1,1), Point(3,-2)" ) {
    CGeometry geometry( K, 0, 2, false );
    geometry.addCellsTouchingSegment( Point(1,1), Point(3,-2) );
    THEN( "Its cell geometry contains 2 0-cells, 11 1-cells, 10 2-cells" ) {
      auto C0 = geometry.getKPoints( 0 );
      auto C1 = geometry.getKPoints( 1 );
      auto C2 = geometry.getKPoints( 2 );
      CAPTURE( C0 );
      CAPTURE( C1 );
      CAPTURE( C2 );
      REQUIRE( C0.size() == 2 );
      REQUIRE( C1.size() == 11 );
      REQUIRE( C2.size() == 10 );
    }
  }
  WHEN( "Computing random segments in domain (-5,-5)-(10,10)." ) {
    unsigned int nb = 20;
    for ( unsigned int i = 0; i < nb; ++i )
      {
        Point a( rand() % 13 - 4, rand() % 13 - 4 );
        Point b( rand() % 13 - 4, rand() % 13 - 4 );
        if ( a == b ) continue;
        CGeometry segm_geometry( K, 0, 2, false );
        segm_geometry.addCellsTouchingSegment( a, b );
        CGeometry splx_geometry( K, 0, 2, false );
        auto splx   = dconv.makeSimplex( { a, b } );
        splx_geometry.addCellsTouchingPolytope( splx );
        auto segm_0 = segm_geometry.getKPoints( 0 );
        auto splx_0 = splx_geometry.getKPoints( 0 );
        auto segm_1 = segm_geometry.getKPoints( 1 );
        auto splx_1 = splx_geometry.getKPoints( 1 );
        auto segm_2 = segm_geometry.getKPoints( 2 );
        auto splx_2 = splx_geometry.getKPoints( 2 );
        THEN( "Generic addCellsTouchingPolytope and specialized addCellsTouchingSegment should provide the same result" ) {
          REQUIRE( segm_0.size() == splx_0.size() );
          REQUIRE( segm_1.size() == splx_1.size() );
          REQUIRE( segm_2.size() == splx_2.size() );
          std::sort( segm_0.begin(), segm_0.end() );
          std::sort( segm_1.begin(), segm_1.end() );
          std::sort( segm_2.begin(), segm_2.end() );
          std::sort( splx_0.begin(), splx_0.end() );
          std::sort( splx_1.begin(), splx_1.end() );
          std::sort( splx_2.begin(), splx_2.end() );
          CAPTURE( segm_0 );
          CAPTURE( segm_1 );
          CAPTURE( segm_2 );
          CAPTURE( splx_0 );
          CAPTURE( splx_1 );
          CAPTURE( splx_2 );
          REQUIRE( std::equal( segm_0.cbegin(), segm_0.cend(), splx_0.cbegin() ) );
          REQUIRE( std::equal( segm_1.cbegin(), segm_1.cend(), splx_1.cbegin() ) );
          REQUIRE( std::equal( segm_2.cbegin(), segm_2.cend(), splx_2.cbegin() ) );
        }
      }
  }
}

SCENARIO( "CellGeometry< Z3 > segment tests", "[cell_geometry][3d][segment]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef CellGeometry< KSpace >           CGeometry;
  typedef DigitalConvexity< KSpace >       DConvexity;

  KSpace K;
  K.init( Point( -5, -5, -5 ), Point( 10, 10, 10 ), true );
  DConvexity dconv( K );
  WHEN( "Computing random segments in domain (-5,-5,-5)-(10,10,10)." ) {
    unsigned int nb = 20;
    for ( unsigned int i = 0; i < nb; ++i )
      {
        Point a( rand() % 13 - 4, rand() % 13 - 4, rand() % 13 - 4 );
        Point b( rand() % 13 - 4, rand() % 13 - 4, rand() % 13 - 4 );
        if ( a == b ) continue;
        CGeometry segm_geometry( K, 0, 3, false );
        segm_geometry.addCellsTouchingSegment( a, b );
        CGeometry splx_geometry( K, 0, 3, false );
        auto splx   = dconv.makeSimplex( { a, b } );
        splx_geometry.addCellsTouchingPolytope( splx );
        auto segm_0 = segm_geometry.getKPoints( 0 );
        auto splx_0 = splx_geometry.getKPoints( 0 );
        auto segm_1 = segm_geometry.getKPoints( 1 );
        auto splx_1 = splx_geometry.getKPoints( 1 );
        auto segm_2 = segm_geometry.getKPoints( 2 );
        auto splx_2 = splx_geometry.getKPoints( 2 );
        auto segm_3 = segm_geometry.getKPoints( 3 );
        auto splx_3 = splx_geometry.getKPoints( 3 );
        THEN( "Generic addCellsTouchingPolytope and specialized addCellsTouchingSegment should provide the same result" ) {
          REQUIRE( segm_0.size() == splx_0.size() );
          REQUIRE( segm_1.size() == splx_1.size() );
          REQUIRE( segm_2.size() == splx_2.size() );
          REQUIRE( segm_3.size() == splx_3.size() );
          std::sort( segm_0.begin(), segm_0.end() );
          std::sort( segm_1.begin(), segm_1.end() );
          std::sort( segm_2.begin(), segm_2.end() );
          std::sort( segm_3.begin(), segm_3.end() );
          std::sort( splx_0.begin(), splx_0.end() );
          std::sort( splx_1.begin(), splx_1.end() );
          std::sort( splx_2.begin(), splx_2.end() );
          std::sort( splx_3.begin(), splx_3.end() );
          CAPTURE( segm_0 );
          CAPTURE( segm_1 );
          CAPTURE( segm_2 );
          CAPTURE( segm_3 );
          CAPTURE( splx_0 );
          CAPTURE( splx_1 );
          CAPTURE( splx_2 );
          CAPTURE( splx_3 );
          REQUIRE( std::equal( segm_0.cbegin(), segm_0.cend(), splx_0.cbegin() ) );
          REQUIRE( std::equal( segm_1.cbegin(), segm_1.cend(), splx_1.cbegin() ) );
          REQUIRE( std::equal( segm_2.cbegin(), segm_2.cend(), splx_2.cbegin() ) );
          REQUIRE( std::equal( segm_3.cbegin(), segm_3.cend(), splx_3.cbegin() ) );
        }
      }
  }
}

SCENARIO( "CellGeometry< Z2 > unit tests", "[cell_geometry][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef CellGeometry< KSpace >           CGeometry;

  KSpace K;
  K.init( Point( -5, -5 ), Point( 10, 10 ), true );
  GIVEN( "Some points (0,0), Point(2,1), Point(1,3)" ) {
    std::vector< Point > V = { Point(0,0), Point(2,1), Point(1,3) };
    CGeometry geometry( K, 0, 2, false );
    geometry.addCellsTouchingPoints( V.begin(), V.end() );
    THEN( "Its cell geometry contains more 1-cells and 2-cells than points" ) {
      REQUIRE( V.size() == geometry.computeNbCells( 0 ) );
      REQUIRE( V.size() <  geometry.computeNbCells( 1 ) );
      REQUIRE( V.size() <  geometry.computeNbCells( 2 ) );
    }
    THEN( "Its cells form an open complex with euler characteristic 3" ) {
      REQUIRE( geometry.computeEuler() == 3 );
    }
  }
}

SCENARIO( "CellGeometry< Z3 > unit tests", "[cell_geometry][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Space                    Space;
  typedef KSpace::Point                    Point;
  typedef CellGeometry< KSpace >           CGeometry;
  typedef HyperRectDomain<Space>           Domain;

  KSpace K;
  const int N = 10;
  K.init( Point( -1, -1, -1 ), Point( N, N, N ), true );
  Domain D( Point( 0, 0, 0 ), Point( N-1, N-1, N-1 ) );

  GIVEN( "Some a block of points (0,0,0)-(N-1,N-1,N-1)" ) {
    CGeometry geometry( K, 0, 3, false );
    geometry.addCellsTouchingPoints( D.begin(), D.end() );
    THEN( "Its cell geometry contains more 1-cells and 2-cells than points" ) {
      REQUIRE( D.size() == geometry.computeNbCells( 0 ) );
      REQUIRE( D.size() <  geometry.computeNbCells( 1 ) );
      REQUIRE( D.size() <  geometry.computeNbCells( 2 ) );
      REQUIRE( D.size() <  geometry.computeNbCells( 3 ) );
    }
    THEN( "Its cells form an open complex with euler characteristic -1" ) {
      REQUIRE( geometry.computeEuler() == -1 );
    }
  }

  GIVEN( "Some a block of points (0,0,0)-(N-1,N-1,N-1)" ) {
    CGeometry geometry( K, 0, 2, false );
    geometry.addCellsTouchingPoints( D.begin(), D.end() );
    THEN( "Its cell geometry contains more 1-cells and 2-cells than points" ) {
      REQUIRE( D.size() == geometry.computeNbCells( 0 ) );
      REQUIRE( D.size() <  geometry.computeNbCells( 1 ) );
      REQUIRE( D.size() <  geometry.computeNbCells( 2 ) );
    }
  }
}

SCENARIO( "CellGeometry< Z2 > intersections", "[cell_geometry][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef CellGeometry< KSpace >           CGeometry;
  typedef BoundedLatticePolytope< Space >  Polytope;

  GIVEN( "A simplex P={ Point(0,0), Point(4,2), Point(-1,4) }" ) {
    KSpace K;
    K.init( Point( -5, -5 ), Point( 10, 10 ), true );
    std::vector< Point > V = { Point(0,0), Point(4,2), Point(-1,4) };
    Polytope P( V.begin(), V.end() );
    CGeometry intersected_cover( K, 0, 2, false );
    intersected_cover.addCellsTouchingPolytope( P );
    CGeometry touched_cover( K, 0, 2, false );
    touched_cover.addCellsTouchingPoints( V.begin(), V.end() );
    CGeometry touched_points_cover( K, 0, 2, false );
    touched_points_cover.addCellsTouchingPolytopePoints( P );
    // trace.info() << "Polytope P=" << P << std::endl;
    THEN( "The cells intersected by its convex hull form an open and simply connected complex." ) {
      REQUIRE( intersected_cover.computeEuler() == 1 );
    }
    THEN( "Its convex hull intersects more cells than its vertices touch." ) {
      REQUIRE( touched_cover.computeNbCells( 0 )
               < intersected_cover.computeNbCells( 0 ) );
      REQUIRE( touched_cover.computeNbCells( 1 )
               < intersected_cover.computeNbCells( 1 ) );
      REQUIRE( touched_cover.computeNbCells( 2 )
               < intersected_cover.computeNbCells( 2 ) );
    }
    THEN( "Its convex hull intersects at least as many cells as its inside points touch." ) {
      REQUIRE( touched_points_cover.computeNbCells( 0 )
               <= intersected_cover.computeNbCells( 0 ) );
      REQUIRE( touched_points_cover.computeNbCells( 1 )
               <= intersected_cover.computeNbCells( 1 ) );
      REQUIRE( touched_points_cover.computeNbCells( 2 )
               <= intersected_cover.computeNbCells( 2 ) );
    }
  }
}

SCENARIO( "CellGeometry< Z3 > intersections", "[cell_geometry][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef CellGeometry< KSpace >           CGeometry;
  typedef BoundedLatticePolytope< Space >  Polytope;

  GIVEN( "A simplex P={ Point(0,0,0), Point(4,2,1), Point(-1,4,1), Point(3,3,5) }" ) {
    KSpace K;
    K.init( Point( -5, -5, -5 ), Point( 10, 10, 10 ), true );
    CGeometry intersected_cover( K, 0, 3, false );
    Polytope P = { Point(0,0,0), Point(4,2,1), Point(-1,4,1), Point(3,3,5) };
    intersected_cover.addCellsTouchingPolytope( P );
    std::vector< Point > V = { Point(0,0,0), Point(4,2,1), Point(-1,4,1), Point(3,3,5) };
    CGeometry touched_cover( K, 0, 3, false );
    touched_cover.addCellsTouchingPoints( V.begin(), V.end() );
    CGeometry touched_points_cover( K, 0, 3, false );
    touched_points_cover.addCellsTouchingPolytopePoints( P );
    THEN( "The cells intersected by its convex hull form an open and simply connected complex." ) {
      REQUIRE( intersected_cover.computeEuler() == -1 );
    }
    THEN( "Its convex hull intersects more cells than its vertices touch." ) {
      REQUIRE( touched_cover.computeNbCells( 0 )
               < intersected_cover.computeNbCells( 0 ) );
      REQUIRE( touched_cover.computeNbCells( 1 )
               < intersected_cover.computeNbCells( 1 ) );
      REQUIRE( touched_cover.computeNbCells( 2 )
               < intersected_cover.computeNbCells( 2 ) );
      REQUIRE( touched_cover.computeNbCells( 3 )
               < intersected_cover.computeNbCells( 3 ) );
    }
    THEN( "Its convex hull intersects at least as many cells as its inside points touch." ) {
      REQUIRE( touched_points_cover.computeNbCells( 0 )
               <= intersected_cover.computeNbCells( 0 ) );
      REQUIRE( touched_points_cover.computeNbCells( 1 )
               <= intersected_cover.computeNbCells( 1 ) );
      REQUIRE( touched_points_cover.computeNbCells( 2 )
               <= intersected_cover.computeNbCells( 2 ) );
      REQUIRE( touched_points_cover.computeNbCells( 3 )
               <= intersected_cover.computeNbCells( 3 ) );
    }
    THEN( "The cells touched by its inside points is a subset of the cells its convex hull intersects." ) {
      REQUIRE( touched_points_cover.subset( intersected_cover, 0 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 1 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 2 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 3 ) );
    }
  }
}

SCENARIO( "CellGeometry< Z2 > rational intersections",
          "[cell_geometry][2d][rational]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef CellGeometry< KSpace >           CGeometry;
  typedef BoundedRationalPolytope< Space >  Polytope;

  GIVEN( "A rational simplex P={ Point(0/4,0/4), Point(17/4,8/4), Point(-5/4,15/4) }" ) {
    KSpace K;
    K.init( Point( -50, -50 ), Point( 100, 100 ), true );
    std::vector< Point > V = { Point(0,0), Point(17,8), Point(-5,15) };
    Polytope P( 4, V.begin(), V.end() );
    CGeometry intersected_cover( K, 0, 2, false );
    intersected_cover.addCellsTouchingPolytope( P );
    CGeometry touched_points_cover( K, 0, 2, false );
    touched_points_cover.addCellsTouchingPolytopePoints( P );
    // trace.info() << "Polytope P=" << P << std::endl;
    THEN( "The cells intersected by its convex hull form an open and simply connected complex." ) {
      REQUIRE( intersected_cover.computeEuler() == 1 );
    }
    THEN( "Its convex hull intersects at least as many cells as its inside points touch." ) {
      REQUIRE( touched_points_cover.computeNbCells( 0 )
               <= intersected_cover.computeNbCells( 0 ) );
      REQUIRE( touched_points_cover.computeNbCells( 1 )
               <= intersected_cover.computeNbCells( 1 ) );
      REQUIRE( touched_points_cover.computeNbCells( 2 )
               <= intersected_cover.computeNbCells( 2 ) );
    }
  }
  GIVEN( "A thin rational simplex P={ Point(6/4,6/4), Point(17/4,8/4), Point(-5/4,15/4) }" ) {
    KSpace K;
    K.init( Point( -50, -50 ), Point( 100, 100 ), true );
    std::vector< Point > V = { Point(6,6), Point(17,8), Point(-5,15) };
    Polytope P( 4, V.begin(), V.end() );
    CGeometry intersected_cover( K, 0, 2, false );
    intersected_cover.addCellsTouchingPolytope( P );
    CGeometry touched_points_cover( K, 0, 2, false );
    touched_points_cover.addCellsTouchingPolytopePoints( P );
    // trace.info() << "Polytope P=" << P << std::endl;
    THEN( "The cells intersected by its convex hull form an open and simply connected complex." ) {
      REQUIRE( intersected_cover.computeEuler() == 1 );
    }
    THEN( "Its convex hull intersects at least as many cells as its inside points touch." ) {
      REQUIRE( touched_points_cover.computeNbCells( 0 )
               <= intersected_cover.computeNbCells( 0 ) );
      REQUIRE( touched_points_cover.computeNbCells( 1 )
               <= intersected_cover.computeNbCells( 1 ) );
      REQUIRE( touched_points_cover.computeNbCells( 2 )
               <= intersected_cover.computeNbCells( 2 ) );
    }
  }
} // SCENARIO( "CellGeometry< Z2 > rational intersections","[cell_geometry][2d][rational]" )


SCENARIO( "CellGeometry< Z3 > rational intersections",
          "[cell_geometry][3d]{rational]" )
{
  typedef KhalimskySpaceND<3,int>           KSpace;
  typedef KSpace::Point                     Point;
  typedef KSpace::Space                     Space;
  typedef CellGeometry< KSpace >            CGeometry;
  typedef BoundedRationalPolytope< Space >  Polytope;

  GIVEN( "A simplex P={ Point(1/2,0/2,-1/2), Point(7/2,3/2,1/2), Point(-2/2,9/2,3/2), Point(6/2,7/2,10/2) }" ) {
    KSpace K;
    K.init( Point( -50, -50, -50 ), Point( 100, 100, 100 ), true );
    CGeometry intersected_cover( K, 0, 3, false );
    Polytope P = { Point(2,2,2),
                   Point(1,0,-1), Point(7,3,1), Point(-2,9,3), Point(6,7,10) };
    intersected_cover.addCellsTouchingPolytope( P );
    CGeometry touched_points_cover( K, 0, 3, false );
    touched_points_cover.addCellsTouchingPolytopePoints( P );
    THEN( "The cells intersected by its convex hull form an open and simply connected complex." ) {
      REQUIRE( intersected_cover.computeEuler() == -1 );
    }
    THEN( "Its convex hull intersects at least as many cells as its inside points touch." ) {
      REQUIRE( touched_points_cover.computeNbCells( 0 )
               <= intersected_cover.computeNbCells( 0 ) );
      REQUIRE( touched_points_cover.computeNbCells( 1 )
               <= intersected_cover.computeNbCells( 1 ) );
      REQUIRE( touched_points_cover.computeNbCells( 2 )
               <= intersected_cover.computeNbCells( 2 ) );
      REQUIRE( touched_points_cover.computeNbCells( 3 )
               <= intersected_cover.computeNbCells( 3 ) );
    }
    THEN( "The cells touched by its inside points is a subset of the cells its convex hull intersects." ) {
      REQUIRE( touched_points_cover.subset( intersected_cover, 0 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 1 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 2 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 3 ) );
    }
  }
} // SCENARIO( "CellGeometry< Z3 > rational intersections", "[cell_geometry][3d]{rational]" )



//                                                                           //
///////////////////////////////////////////////////////////////////////////////
