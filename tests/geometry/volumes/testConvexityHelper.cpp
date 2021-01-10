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
 * @file testConvexityHelper.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/01
 *
 * Functions for testing class ConvexityHelper.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/volumes/ConvexityHelper.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ConvexityHelper in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "ConvexityHelper< 2 > unit tests", "[convexity_helper][2d]" )
{
  typedef ConvexityHelper< 2 >    Helper;
  typedef Helper::Point           Point;
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef Space::Vector                    Vector;
  typedef QHull::Index                     Index;
  typedef QHull::IndexRange                IndexRange;

  
  GIVEN( "Given a star { (0,0), (-4,-1), (-3,5), (7,3), (5, -2) } " ) {
    std::vector<Point> V
      = { Point(0,0), Point(-4,-1), Point(-3,5), Point(7,3), Point(5, -2) };
    WHEN( "Computing its lattice polytope" ){
      auto P = Helper::computeLatticePolytope( V, false, true );
      THEN( "The polytope is valid and has 4 facets" ) {
        REQUIRE( P.nbHalfSpaces() == 4 );
      }
      THEN( "The polytope is Minkowski summable" ) {
        REQUIRE( P.canBeSummed() );
      }
      THEN( "The polytope contains the input points" ) {
        REQUIRE( P.isInside( V[ 0 ] ) );
        REQUIRE( P.isInside( V[ 1 ] ) );
        REQUIRE( P.isInside( V[ 2 ] ) );
        REQUIRE( P.isInside( V[ 3 ] ) );
        REQUIRE( P.isInside( V[ 4 ] ) );
      }
    }
  }
} 
