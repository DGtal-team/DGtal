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

SCENARIO( "EhrhartPolynomial< Z3 > unit tests", "[ehrhart_polynomial][3d]" )
{
  typedef SpaceND< 3, int >          Space;
  typedef KhalimskySpaceND< 3, int > KSpace;
  typedef Space::Point               Point;
  typedef DGtal::int64_t             Integer;
  typedef EhrhartPolynomial< Space, Integer > Ehrhart;

  GIVEN( "A convex polytope" ) {
    std::vector< Point > T = { Point(0,0,0), Point(1,0,1), Point(2,1,0), Point(0,0,2),
      Point(0,3,1) };
    DigitalConvexity< KSpace > dconv( Point::diagonal( -100 ), Point::diagonal( 100 ) );
    auto P = dconv.makePolytope( T );
    Ehrhart E( P );
    CAPTURE( E.numerator() );
    CAPTURE( E.denominator() );
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


SCENARIO( "EhrhartPolynomial< Z4 > unit tests", "[ehrhart_polynomial][4d]" )
{
  typedef SpaceND< 4, int >          Space;
  typedef KhalimskySpaceND< 4, int > KSpace;
  typedef Space::Point               Point;
  typedef DGtal::int64_t             Integer;
  typedef EhrhartPolynomial< Space, Integer > Ehrhart;

  GIVEN( "A convex polytope" ) {
    std::vector< Point > T = { Point(0,0,0,0), Point(1,0,1,-1), Point(2,1,0,2),
      Point(0,0,2,0), Point(0,4,1,3), Point(3,0,-1,1) };
    DigitalConvexity< KSpace > dconv( Point::diagonal( -100 ), Point::diagonal( 100 ) );
    auto P = dconv.makePolytope( T );
    Ehrhart E( P );
    CAPTURE( E.numerator() );
    CAPTURE( E.denominator() );
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

SCENARIO( "EhrhartPolynomial< Z2 > triangle tests", "[ehrhart_polynomial][2d]" )
{
  typedef SpaceND< 2, int >          Space;
  typedef HyperRectDomain< Space >   Domain;
  typedef KhalimskySpaceND< 2, int > KSpace;
  typedef Space::Point               Point;
  typedef DGtal::int64_t             Integer;
  typedef EhrhartPolynomial< Space, Integer > Ehrhart;
  typedef Ehrhart::LatticePolytope   Polytope;
  typedef Polytope::UnitSegment      UnitSegment;

  Domain     domain( Point( 0, 0 ), Point( 4, 4 ) );
  DigitalConvexity<KSpace> dconv( Point( -1, -1 ), Point( 5, 5 ) );

  WHEN( "Computing all triangles in domain (0,0)-(4,4)." ) {
    unsigned int nbsimplex = 0;
    unsigned int nb0_ok = 0;
    unsigned int nb1_ok = 0;
    unsigned int nb_cvx_ok = 0;
    for ( auto a : domain )
      for ( auto b : domain )
        for ( auto c : domain )
          {
            if ( ! ( ( a < b ) && ( b < c ) ) ) continue;
            if ( ! dconv.isSimplexFullDimensional( { a, b, c } ) ) continue;
            const auto    P = dconv.makeSimplex( { a, b, c } );
            const bool fcvx = dconv.isFullyConvex( P );
            const auto   P0 = P + UnitSegment( 0 );
            const auto   P1 = P + UnitSegment( 1 );
            const auto    D = P.getDomain();
            const auto    W = D.upperBound() - D.lowerBound();
            const auto  E_P = Ehrhart( P  );
            const auto E_P0 = Ehrhart( P0 );
            const auto E_P1 = Ehrhart( P1 );
            const auto   LP = E_P.numerator();
            const auto  LP0 = E_P0.numerator();
            const auto  LP1 = E_P1.numerator();
            const auto   LD = E_P.denominator();
            const auto  LD0 = E_P0.denominator();
            const auto  LD1 = E_P1.denominator();
            const bool c2_0 = ( LP[ 2 ] + Integer( W[ 1 ] ) * LD ) == LP0[ 2 ];
            const bool c1_0 = ( LP[ 1 ] + Integer( 1 ) * LD ) == LP0[ 1 ];
            const bool c0_0 = LP[ 0 ] == LP0[ 0 ];
            const bool  d_0 = LD == LD0;
            const bool c2_1 = ( LP[ 2 ] + Integer( W[ 0 ] ) * LD ) == LP1[ 2 ];
            const bool c1_1 = ( LP[ 1 ] + Integer( 1 ) * LD ) == LP1[ 1 ];
            const bool c0_1 = LP[ 0 ] == LP1[ 0 ];
            const bool  d_1 = LD == LD1;
            nb0_ok += ( c2_0 && c1_0 && c0_0 && d_0 ) ? 1 : 0;
            nb1_ok += ( c2_1 && c1_1 && c0_1 && d_1 ) ? 1 : 0;
            nbsimplex += 1;
            nb_cvx_ok += fcvx ? 1 : 0;
            if ( ! ( c2_0 && c1_0 && c0_0 && d_0 ) ){
              trace.info() << "------------------------------------" << std::endl;
              trace.info() << ( fcvx ? "FULLCVX" : "NOTFULLCVX" ) << a << b << c << std::endl;
              trace.info() << "W[0] = " << W[ 0 ] << "W[1] = " << W[ 1 ] << std::endl;
              trace.info() << "LP = (" << LP << ")/" << E_P.denominator() << std::endl;
              trace.info() << "LP0= (" << LP0 << ")/" << E_P0.denominator() << std::endl;
              break;
            }
            if ( ! ( c2_1 && c1_1 && c0_1 && d_1 ) ){
              trace.info() << "------------------------------------" << std::endl;
              trace.info() << ( fcvx ? "FULLCVX" : "NOTFULLCVX" ) << a << b << c << std::endl;
              trace.info() << "W[0] = " << W[ 0 ] << "W[1] = " << W[ 1 ] << std::endl;
              trace.info() << "LP = (" << LP << ")/" << E_P.denominator() << std::endl;
              trace.info() << "LP1= (" << LP1 << ")/" << E_P1.denominator() << std::endl;
              break;
            }
          }
    THEN( "Not all triangles are fully convex." ) {
      REQUIRE( nb_cvx_ok < nbsimplex );
    }
    THEN( "Ehrhart polynomials are predictable." ) {
      CAPTURE( nb_cvx_ok );
      REQUIRE( nb0_ok == nbsimplex );
      REQUIRE( nb1_ok == nbsimplex );
    }
  }
}
