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
 * @file testEhrhartPolynomial.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/08/24
 *
 * Functions for testing class EhrhartPolynomial.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/volumes/EhrhartPolynomial.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class EhrhartPolynomial.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "EhrhartPolynomial< Z2 > unit tests", "[ehrhart_polynomial][2d]" )
{
  typedef SpaceND< 2, int >          Space;
  typedef KhalimskySpaceND< 2, int > KSpace;
  typedef Space::Point               Point;
  typedef DGtal::int64_t             Integer;
  typedef EhrhartPolynomial< Space, Integer > Ehrhart;

  GIVEN( "Simplex (0,0), (1,0), (2,1)" ) {
    std::vector< Point > T = { Point(0,0), Point(1,0), Point(2,1) };
    DigitalConvexity< KSpace > dconv( Point::diagonal( -100 ), Point::diagonal( 100 ) );
    auto P = dconv.makeSimplex( T.cbegin(), T.cend() );
    Ehrhart E( P );
    THEN( "Its Ehrhart polynomial is 1/2( 2+ 3t+t^2) ") {
      auto expP = mmonomial<Integer>( 2 ) + 3 * mmonomial<Integer>( 1 ) + 2;
      REQUIRE( E.numerator()   == expP );
      REQUIRE( E.denominator() == 2 );
    }
    THEN( "Its Ehrhart polynomial counts lattice points" ) {
      auto P0 = 0 * P;
      auto n0 = E.count( 0 );
      REQUIRE( P0.count() == n0 );
      auto P1 = 1 * P;
      auto n1 = E.count( 1 );
      REQUIRE( P1.count() == n1 );
      auto P2 = 2 * P;
      auto n2 = E.count( 2 );
      REQUIRE( P2.count() == n2 );
      auto P3 = 3 * P;
      auto n3 = E.count( 3 );
      REQUIRE( P3.count() == n3 );
    }
    THEN( "Its Ehrhart polynomial counts interior lattice points" ) {
      auto P1 = 1 * P;
      auto n1 = E.countInterior( 1 );
      REQUIRE( P1.countInterior() == n1 );
      auto P2 = 2 * P;
      auto n2 = E.countInterior( 2 );
      REQUIRE( P2.countInterior() == n2 );
      auto P3 = 3 * P;
      auto n3 = E.countInterior( 3 );
      REQUIRE( P3.countInterior() == n3 );
      auto P4 = 4 * P;
      auto n4 = E.countInterior( 4 );
      REQUIRE( P4.countInterior() == n4 );
    }
  }
}
