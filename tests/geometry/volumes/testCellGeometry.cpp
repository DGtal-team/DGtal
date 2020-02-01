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
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CellGeometry.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "CellGeometry< Z2 > unit tests", "[cell_geometry][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
  typedef CellGeometry< KSpace >           CGeometry;

  KSpace K;
  K.init( Point( -5, -5 ), Point( 10, 10 ), true );
  GIVEN( "Some points (0,0), Point(2,1), Point(1,3)" ) {
    std::vector< Point > V = { Point(0,0), Point(2,1), Point(1,3) }; 
    CGeometry geometry( K, 0, 2, false );
    geometry.addCellsTouchingPoints( V.begin(), V.end() );
    THEN( "Its cell geometry contains more 1-cells and 2-cells than points" ) {
      CAPTURE( geometry.cubicalComplex() );
      REQUIRE( V.size() == geometry.cubicalComplex().nbCells( 0 ) );
      REQUIRE( V.size() <  geometry.cubicalComplex().nbCells( 1 ) );
      REQUIRE( V.size() <  geometry.cubicalComplex().nbCells( 2 ) );
    }
    THEN( "Its cells form an open complex with euler characteristic 3" ) {
      REQUIRE( geometry.cubicalComplex().euler() == 3 );
    }
  }
}

SCENARIO( "CellGeometry< Z3 > unit tests", "[cell_geometry][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Space                    Space;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
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
      CAPTURE( geometry.cubicalComplex() );
      REQUIRE( D.size() == geometry.cubicalComplex().nbCells( 0 ) );
      REQUIRE( D.size() <  geometry.cubicalComplex().nbCells( 1 ) );
      REQUIRE( D.size() <  geometry.cubicalComplex().nbCells( 2 ) );
      REQUIRE( D.size() <  geometry.cubicalComplex().nbCells( 3 ) );
    }
    THEN( "Its cells form an open complex with euler characteristic -1" ) {
      REQUIRE( geometry.cubicalComplex().euler() == -1 );
    }
  }

  GIVEN( "Some a block of points (0,0,0)-(N-1,N-1,N-1)" ) {
    CGeometry geometry( K, 0, 2, false );
    geometry.addCellsTouchingPoints( D.begin(), D.end() );
    THEN( "Its cell geometry contains more 1-cells and 2-cells than points" ) {
      CAPTURE( geometry.cubicalComplex() );
      REQUIRE( D.size() == geometry.cubicalComplex().nbCells( 0 ) );
      REQUIRE( D.size() <  geometry.cubicalComplex().nbCells( 1 ) );
      REQUIRE( D.size() <  geometry.cubicalComplex().nbCells( 2 ) );
    }
  }
}

SCENARIO( "CellGeometry< Z2 > intersections", "[cell_geometry][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
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
      REQUIRE( intersected_cover.cubicalComplex().euler() == 1 );
    }
    THEN( "Its convex hull intersects more cells than its vertices touch." ) {
      REQUIRE( touched_cover.cubicalComplex().nbCells( 0 )
	       < intersected_cover.cubicalComplex().nbCells( 0 ) );
      REQUIRE( touched_cover.cubicalComplex().nbCells( 1 )
	       < intersected_cover.cubicalComplex().nbCells( 1 ) );
      REQUIRE( touched_cover.cubicalComplex().nbCells( 2 )
	       < intersected_cover.cubicalComplex().nbCells( 2 ) );
    }
    THEN( "Its convex hull intersects at least as many cells as its inside points touch." ) {
      REQUIRE( touched_points_cover.cubicalComplex().nbCells( 0 )
	       <= intersected_cover.cubicalComplex().nbCells( 0 ) );
      REQUIRE( touched_points_cover.cubicalComplex().nbCells( 1 )
	       <= intersected_cover.cubicalComplex().nbCells( 1 ) );
      REQUIRE( touched_points_cover.cubicalComplex().nbCells( 2 )
	       <= intersected_cover.cubicalComplex().nbCells( 2 ) );
    }
  }
}
SCENARIO( "CellGeometry< Z3 > intersections", "[cell_geometry][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
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
      REQUIRE( intersected_cover.cubicalComplex().euler() == -1 );
    }
    THEN( "Its convex hull intersects more cells than its vertices touch." ) {
      REQUIRE( touched_cover.cubicalComplex().nbCells( 0 )
	       < intersected_cover.cubicalComplex().nbCells( 0 ) );
      REQUIRE( touched_cover.cubicalComplex().nbCells( 1 )
	       < intersected_cover.cubicalComplex().nbCells( 1 ) );
      REQUIRE( touched_cover.cubicalComplex().nbCells( 2 )
	       < intersected_cover.cubicalComplex().nbCells( 2 ) );
      REQUIRE( touched_cover.cubicalComplex().nbCells( 3 )
	       < intersected_cover.cubicalComplex().nbCells( 3 ) );
    }
    THEN( "Its convex hull intersects at least as many cells as its inside points touch." ) {
      REQUIRE( touched_points_cover.cubicalComplex().nbCells( 0 )
	       <= intersected_cover.cubicalComplex().nbCells( 0 ) );
      REQUIRE( touched_points_cover.cubicalComplex().nbCells( 1 )
	       <= intersected_cover.cubicalComplex().nbCells( 1 ) );
      REQUIRE( touched_points_cover.cubicalComplex().nbCells( 2 )
	       <= intersected_cover.cubicalComplex().nbCells( 2 ) );
      REQUIRE( touched_points_cover.cubicalComplex().nbCells( 3 )
	       <= intersected_cover.cubicalComplex().nbCells( 3 ) );
    }
    THEN( "The cells touched by its inside points is a subset of the cells its convex hull intersects." ) {
      REQUIRE( touched_points_cover.subset( intersected_cover, 0 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 1 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 2 ) );
      REQUIRE( touched_points_cover.subset( intersected_cover, 3 ) );
    }
  }
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
