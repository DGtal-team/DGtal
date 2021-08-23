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
 * @file testFullConvexity.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/01
 *
 * Functions for testing full convexity.
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
// Functions for testing class DigitalConvexity.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "DigitalConvexity< Z2 > full convexity tests", "[digital_convexity][2d][full_convexity]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -5, -5 ), Point( 10, 10 ) );

  std::vector<Point> V1 = { Point(0,0), Point(-1,0), Point(1,0), Point(0,1) };
  REQUIRE( dconv.isFullyConvex( V1 ) );
  std::vector<Point> V2 = { Point(-1,0), Point(1,0), Point(0,1) };
  REQUIRE( ! dconv.isFullyConvex( V2 ) );
  std::vector<Point> V3 = { Point(0,0), Point(-1,0), Point(1,0) };
  REQUIRE( dconv.isFullyConvex( V3 ) );
  std::vector<Point> V4 = { Point(0,0), Point(-1,0), Point(1,0), Point(0,1),
    Point(0,-1) };
  REQUIRE( dconv.isFullyConvex( V4 ) );
  std::vector<Point> V5 = { Point(-1,0), Point(1,0), Point(0,1), Point(0,-1) };
  REQUIRE( ! dconv.isFullyConvex( V5 ) );
}

SCENARIO( "FullConvexity< Z3 > full convexity tests", "[full_convexity][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -5, -5, -5 ), Point( 10, 10, 10 ) );
  Point a( 0, 2, 3 );
  Point b( 3, 1, 0 );
  Point c( 2, 0, 1 );
  Point d( 2, 1, 2 );
  std::vector< Point > X;
  REQUIRE( dconv.isSimplexFullDimensional( { a, b, c, d } ) );
  auto tetra= dconv.makeSimplex( { a, b, c, d } );
  tetra.getPoints( X );
  bool cvx0 = dconv.isKConvex( tetra, 0 );
  bool cvx1 = dconv.isKConvex( tetra, 1 );
  bool cvx2 = dconv.isKConvex( tetra, 2 );
  bool cvx3 = dconv.isKConvex( tetra, 3 );
  bool cvxf = dconv.isFullyConvex( tetra );
  bool cvxg = dconv.isFullyConvex( X, false );
  REQUIRE( cvxf == cvxg );
  REQUIRE( ( cvx0 && cvx1 && cvx2 && cvx3 ) == cvxf );
}
  
