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
 * @file testLagrangeInterpolation.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/08/24
 *
 * Functions for testing class LagrangeInterpolation.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/math/LagrangeInterpolation.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LagrangeInterpolation.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "LagrangeInterpolation< int64_t > unit tests", "[lagrange_interpolation]" )
{
  typedef DGtal::int64_t Ring;
  typedef LagrangeInterpolation< Ring > LInterp;
  typedef LInterp::Polynomial Polynomial;

  GIVEN( "A polynomial of degree 2 and 3 interpolation points" ) {
    Polynomial P1 = mmonomial<Ring>( 2 ); // P1(x) = x^2
    std::vector< Ring > X = { 1, 2, 3 };
    std::vector< Ring > Y;
    for ( auto x : X ) Y.push_back( P1( x ) );
    LInterp L1( X );
    CAPTURE( L1 );
    THEN( "Its Lagrange polynomial is itself" ) {
      auto Lag1 = L1.polynomial( Y );
      REQUIRE( Lag1 == P1 * L1.denominator() );
    }
  }

  GIVEN( "A polynomial of degree 3 and 4 interpolation points" ) {
    Polynomial P2 = mmonomial<Ring>( 3 ) + 3*mmonomial<Ring>( 1 ) ; // P1(x) = x^3+3x
    std::vector< Ring > X = { -1, 1, 2, 3 };
    std::vector< Ring > Y;
    for ( auto x : X ) Y.push_back( P2( x ) );
    LInterp L2( X );
    CAPTURE( L2 );
    THEN( "Its Lagrange polynomial is itself" ) {
      auto Lag2 = L2.polynomial( Y );
      REQUIRE( Lag2 == P2 * L2.denominator() );
    }
  }

  GIVEN( "3 interpolation points (0,1), (1,3), (2,6)" ) {
    std::vector< Ring > X = { 0, 1, 2 };
    std::vector< Ring > Y = { 1, 3, 6 };
    LInterp L3( X );
    CAPTURE( L3 );
    THEN( "Its Lagrange polynomial is 1/2*(2+3x+x^2)" ) {
      auto Lag3 = L3.polynomial( Y );
      Polynomial Exp3 = mmonomial<Ring>( 2 ) + 3 * mmonomial<Ring>( 1 ) + 2;
      REQUIRE( Lag3 == Exp3 );
    }
  }

  GIVEN( "3 interpolation points (0,1), (1,7), (2,20)" ) {
    std::vector< Ring > X = { 0, 1, 2 };
    std::vector< Ring > Y = { 1, 7, 20 };
    LInterp L4( X );
    CAPTURE( L4 );
    THEN( "Its Lagrange polynomial is 1/2*(2+5x+7x^2)" ) {
      auto Lag4 = L4.polynomial( Y );
      Polynomial Exp4 = 7 * mmonomial<Ring>( 2 ) + 5 * mmonomial<Ring>( 1 ) + 2;
      REQUIRE( Lag4 == Exp4 );
    }
  }
}
