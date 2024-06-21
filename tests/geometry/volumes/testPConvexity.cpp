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
 * @file testPConvexity.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University Savoie Mont Blanc, France
 *
 * @date 2024/06/21
 *
 * Functions for testing P-convexity.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/geometry/volumes/PConvexity.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PConvexity.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "PConvexity< Z2 > P-convexity tests", "[p_convexity][2d]" )
{
  typedef SpaceND< 2, int >    Space;
  typedef Space::Point         Point;
  typedef PConvexity< Space >  Convexity;

  Convexity pconv;

  std::vector<Point> V1 = { Point(0,0), Point(-1,0), Point(1,0), Point(0,1) };
  REQUIRE( pconv.is0Convex( V1 ) );
  REQUIRE( pconv.isPConvex( V1 ) );
  std::vector<Point> V2 = { Point(-1,0), Point(1,0), Point(0,1) };
  REQUIRE( ! pconv.is0Convex( V2 ) );
  REQUIRE( ! pconv.isPConvex( V2 ) );
  std::vector<Point> V3 = { Point(0,0), Point(-1,0), Point(1,0) };
  REQUIRE( pconv.is0Convex( V3 ) );
  REQUIRE( pconv.isPConvex( V3 ) );
  std::vector<Point> V4 = { Point(0,0), Point(-1,0), Point(1,0), Point(0,1),
    Point(0,-1) };
  REQUIRE( pconv.is0Convex( V4 ) );
  REQUIRE( pconv.isPConvex( V4 ) );
  std::vector<Point> V5 = { Point(-1,0), Point(0,0), Point(3,1) };
  REQUIRE( pconv.is0Convex( V5 ) );
  REQUIRE( ! pconv.isPConvex( V5 ) );
}

SCENARIO( "PConvexity< Z3 > ball tests", "[p_convexity][3d]" )
{
  GIVEN( "Given a 3D digital ball of radius 5 " ) {
    typedef SpaceND<3,int>          Space;
    typedef Space::Point            Point;
    typedef PConvexity< Space >     Convexity;
    typedef HyperRectDomain< Space >         Domain;
    typedef DigitalSetBySTLSet< Domain >     DigitalSet;
    
    Convexity  conv;
    Point      lo = Point::diagonal( -7 );
    Point      hi = Point::diagonal(  7 );
    Point      c  = Point::zero;
    Domain     domain( lo, hi );
    DigitalSet ball  ( domain );
    Shapes< Domain >::addNorm2Ball( ball, c, 5 );
    std::vector<Point> V( ball.begin(), ball.end() );
    bool cvx0 = conv.is0Convex( V );
    bool fcvx = conv.isPConvex( V );
    THEN( "It is a 0-convex and P-convex by morphological characterization" ) {
      REQUIRE( cvx0 );
      REQUIRE( fcvx );
    }
  }
}

SCENARIO( "PConvexity< Z4 > ball tests", "[p_convexity][4d]" )
{
  GIVEN( "Given a 4D digital ball of radius 5 " ) {
    typedef SpaceND<4,int>          Space;
    typedef Space::Point            Point;
    typedef PConvexity< Space >     Convexity;
    typedef HyperRectDomain< Space >         Domain;
    typedef DigitalSetBySTLSet< Domain >     DigitalSet;
    
    Convexity  conv;
    Point      lo = Point::diagonal( -7 );
    Point      hi = Point::diagonal(  7 );
    Point      c  = Point::zero;
    Domain     domain( lo, hi );
    DigitalSet ball  ( domain );
    Shapes< Domain >::addNorm2Ball( ball, c, 5 );
    std::vector<Point> V( ball.begin(), ball.end() );
    bool cvx0 = conv.is0Convex( V );
    bool fcvx = conv.isPConvex( V );
    THEN( "It is a 0-convex and P-convex by morphological characterization" ) {
      REQUIRE( cvx0 );
      REQUIRE( fcvx );
    }
  }
}

