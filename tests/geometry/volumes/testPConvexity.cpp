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
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/geometry/volumes/PConvexity.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
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
  REQUIRE( pconv.convexityMeasure( V1 ) == 1.0 );
  REQUIRE( pconv.fullConvexityMeasure( V1 ) == 1.0 );
  std::vector<Point> V2 = { Point(-1,0), Point(1,0), Point(0,1) };
  REQUIRE( ! pconv.is0Convex( V2 ) );
  REQUIRE( ! pconv.isPConvex( V2 ) );
  REQUIRE( pconv.convexityMeasure( V2 ) < 1.0 );
  REQUIRE( pconv.fullConvexityMeasure( V2 ) < 1.0 );
  std::vector<Point> V3 = { Point(0,0), Point(-1,0), Point(1,0) };
  REQUIRE( pconv.is0Convex( V3 ) );
  REQUIRE( pconv.isPConvex( V3 ) );
  REQUIRE( pconv.convexityMeasure( V3 ) == 1.0 );
  REQUIRE( pconv.fullConvexityMeasure( V3 ) == 1.0 );
  std::vector<Point> V4 = { Point(0,0), Point(-1,0), Point(1,0), Point(0,1),
    Point(0,-1) };
  REQUIRE( pconv.is0Convex( V4 ) );
  REQUIRE( pconv.isPConvex( V4 ) );
  std::vector<Point> V5 = { Point(-1,0), Point(0,0), Point(3,1) };
  REQUIRE( pconv.is0Convex( V5 ) );
  REQUIRE( ! pconv.isPConvex( V5 ) );
  REQUIRE( pconv.convexityMeasure( V5 ) == 1.0 );
  REQUIRE( pconv.fullConvexityMeasure( V5 ) < 1.0 );
}

SCENARIO( "PConvexity< Z3 > ball tests", "[p_convexity][3d]" )
{
  GIVEN( "Given a 3D digital ball of radius 5 " ) {
    typedef SpaceND<3,int>          Space;
    typedef Space::Point            Point;
    typedef PConvexity< Space >     Convexity;
    typedef HyperRectDomain< Space >         Domain;
    typedef DigitalSetBySTLSet< Domain >     DigitalSet;

    Convexity  pconv;
    Point      lo = Point::diagonal( -7 );
    Point      hi = Point::diagonal(  7 );
    Point      c  = Point::zero;
    Domain     domain( lo, hi );
    DigitalSet ball  ( domain );
    Shapes< Domain >::addNorm2Ball( ball, c, 5 );
    std::vector<Point> V( ball.begin(), ball.end() );
    bool cvx0 = pconv.is0Convex( V );
    bool fcvx = pconv.isPConvex( V );
    THEN( "It is a 0-convex and P-convex by morphological characterization" ) {
      REQUIRE( cvx0 );
      REQUIRE( fcvx );
    }
    THEN( "Then both its convexity measure and its full convexity measure is 1.0" ) {
      REQUIRE( pconv.convexityMeasure( V ) == 1.0 );
      REQUIRE( pconv.fullConvexityMeasure( V ) == 1.0 );
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
    THEN( "Then both its convexity measure and its full convexity measure is 1.0" ) {
      REQUIRE( conv.convexityMeasure( V ) == 1.0 );
      REQUIRE( conv.fullConvexityMeasure( V ) == 1.0 );
    }
  }
}


SCENARIO( "DigitalConvexity< Z3 > fully convex and p-convex tetrahedra", "[p_convexity][full_convexity][convex_simplices][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;
  typedef PConvexity< Space >              PConvexity;

  Domain     domain( Point( 0, 0, 0 ), Point( 3, 3, 3 ) );
  DConvexity dconv( Point( -1, -1, -1 ), Point( 4, 4, 4 ) );
  PConvexity pconv;

  WHEN( "Computing many tetrahedra in domain (0,0,0)-(4,4,4)." ) {
    const unsigned int nb = 100;
    unsigned int nbsimplex= 0;
    unsigned int nb0      = 0;
    unsigned int nb1      = 0;
    unsigned int nb2      = 0;
    unsigned int nb3      = 0;
    unsigned int nb012_not3 = 0;
    unsigned int nbf      = 0;
    unsigned int nbfg     = 0;
    unsigned int nbffast  = 0;
    unsigned int nbp      = 0;
    unsigned int nb0123   = 0;
    for ( unsigned int i = 0; i < nb; ++i )
      {
        Point a( rand() % 5, rand() % 5, rand() % 5 );
        Point b( rand() % 5, rand() % 5, rand() % 5 );
        Point c( rand() % 5, rand() % 5, rand() % 5 );
        Point d( rand() % 5, rand() % 5, rand() % 5 );
        if ( ! dconv.isSimplexFullDimensional( { a, b, c, d } ) ) continue;
        auto tetra = dconv.makeSimplex( { a, b, c, d } );
        std::vector< Point > X;
        tetra.getPoints( X );
        bool cvx0     = dconv.isKConvex( tetra, 0 );
        bool cvx1     = dconv.isKConvex( tetra, 1 );
        bool cvx2     = dconv.isKConvex( tetra, 2 );
        bool cvx3     = dconv.isKConvex( tetra, 3 );
        bool cvxf     = dconv.isFullyConvex( tetra );
        bool cvxfg    = dconv.isFullyConvex( X, false );
        bool cvxffast = dconv.isFullyConvexFast( X );
        bool cvxp     = pconv.isPConvex( X );
        if ( cvxf != cvxfg || cvxf != cvxffast || cvxf != cvxp ) {
          std::cout << "[" << cvx0 << cvx1 << cvx2 << cvx3 << "] "
                    << "[" << cvxf << "] [" << cvxfg
                    << "] [" << cvxffast << "]"
                    << "] [" << cvxp << "]"
                    << a << b << c << d << std::endl;
        }
        nbsimplex += 1;
        nb0       += cvx0 ? 1 : 0;
        nb1       += cvx1 ? 1 : 0;
        nb2       += cvx2 ? 1 : 0;
        nb3       += cvx3 ? 1 : 0;
        nbf       += cvxf ? 1 : 0;
        nbfg      += cvxfg ? 1 : 0;
        nbffast   += cvxffast ? 1 : 0;
        nbp       += cvxp ? 1 : 0;
        nb0123    += ( cvx0 && cvx1 && cvx2 && cvx3 ) ? 1 : 0;
        nb012_not3+= ( cvx0 && cvx1 && cvx2 && ! cvx3 ) ? 1 : 0;
      }
    THEN( "All valid tetrahedra are 0-convex." ) {
      REQUIRE( nb0 == nbsimplex );
    }
    THEN( "There are less 1-convex, 2-convex and 3-convex than 0-convex." ) {
      REQUIRE( nb1 < nb0 );
      REQUIRE( nb2 < nb0 );
      REQUIRE( nb3 < nb0 );
    }
    THEN( "When the tetrahedron is 0-convex, 1-convex and 2-convex, then it is 3-convex, so fully convex and also P-convex." ) {
      REQUIRE( nb1 <= nb3 );
      REQUIRE( nb2 <= nb3 );
      REQUIRE( nb012_not3 == 0 );
      REQUIRE( nbf == nb0123 );
      REQUIRE( nbf == nbp );
    }
    THEN( "All methods for computing full convexity and P-convexity agree." ) {
      REQUIRE( nbf == nbfg );
      REQUIRE( nbf == nbffast );
      REQUIRE( nbf == nbp );
    }
  }
}
