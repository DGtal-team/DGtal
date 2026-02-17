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
 * @file testBoundedLatticePolytopeCounter.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/01/04
 *
 * Functions for testing class BoundedLatticePolytopeCounter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytopeCounter.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class BoundedLatticePolytopeCounter.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "BoundedLatticePolytopeCounter< Z2 > unit tests", "[lattice_polytope][2d]" )
{
  typedef SpaceND<2,int>                   Space;
  typedef Space::Point                     Point;
  typedef BoundedLatticePolytope< Space >  Polytope;
  typedef BoundedLatticePolytopeCounter< Space > Counter;

  GIVEN( "A triangle P at (0,3), (5,0), (15,7)" ) {
    Point a( 0, 3 );
    Point b( 8, 0 );
    Point c( 15, 7 );
    Polytope P { a, b, c };
    int nbInside = P.count();
    int nbInterior = P.countInterior();
    Counter C( P );
    int nb0 = C.countAlongAxis( 0 );
    int nb1 = C.countAlongAxis( 1 );
    int nb0_int = C.countInteriorAlongAxis( 0 );
    int nb1_int = C.countInteriorAlongAxis( 1 );
    THEN( "Its longest axis is 0" )
      {
        REQUIRE( C.longestAxis() == 0 );
      }
    THEN( "We can count its points per point or per axis" )
      {
        REQUIRE( nbInside == nb0 );
        REQUIRE( nbInside == nb1 );
      }
    THEN( "We can count its interior points per point or per axis" )
      {
        REQUIRE( nbInterior == nb0_int );
        REQUIRE( nbInterior == nb1_int );
      }
  }
}

SCENARIO( "BoundedLatticePolytope< Z3 > unit tests", "[lattice_polytope][3d]" )
{
  typedef SpaceND<3,int>                   Space;
  typedef Space::Point                     Point;
  typedef BoundedLatticePolytope< Space >  Polytope;
  typedef BoundedLatticePolytopeCounter< Space > Counter;

  GIVEN( "A closed arbitrary simplex P at (0,0,0), (6,3,0), (0,5,-10), (-6,4,8)" ) {
    Point a( 0, 0, 0 );
    Point b( 6, 3, 0 );
    Point c( 0, 5, -10 );
    Point d( -6, 4, 8 );
    Polytope P { a, b, c, d };
    int nbInside   = P.count();
    int nbInterior = P.countInterior();
    Counter C( P );
    int nb0 = C.countAlongAxis( 0 );
    int nb1 = C.countAlongAxis( 1 );
    int nb2 = C.countAlongAxis( 2 );
    int nb0_int = C.countInteriorAlongAxis( 0 );
    int nb1_int = C.countInteriorAlongAxis( 1 );
    int nb2_int = C.countInteriorAlongAxis( 2 );
    // std::cout << P << std::endl;
    THEN( "Its longest axis is 2" )
      {
        REQUIRE( C.longestAxis() == 2 );
      }
    THEN( "We can count its points per point or per axis" )
      {
        REQUIRE( nbInside == nb0 );
        REQUIRE( nbInside == nb1 );
        REQUIRE( nbInside == nb2 );
      }
    THEN( "We can count its interior points per point or per axis" )
      {
        REQUIRE( nbInterior == nb0_int );
        REQUIRE( nbInterior == nb1_int );
        REQUIRE( nbInterior == nb2_int );
      }
  }
}
