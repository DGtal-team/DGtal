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
 * @file testIntegralIntervals.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/07/01
 *
 * Functions for testing class IntegralIntervals.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/IntegralIntervals.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegralIntervals.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "IntegralIntervals< int > unit tests", "[intervals]" )
{
  typedef int Integer;
  typedef IntegralIntervals< Integer > Intervals;

  Intervals V;
  WHEN( "Inserting many intervals" ) {
    int nb = 1000;
    int nb_ok = 0;
    int nb_count_ok = 0;
    std::set< int > X;
    for ( int i = 0; i < nb; i++ )
      {
        int f = rand() % 1000;
        int l = std::min( f + rand() % 10, 999 );
        // std::cout << "V + (" << f << "," << l << ") = ";
        V.insert( f, l );
        for ( int k = f; k <= l; k++ ) X.insert( k );
        nb_ok += V.isValid() ? 1 : 0;
        nb_count_ok += ( V.size() == X.size() ) ? 1 : 0;
        if ( ! V.isValid() )
          std::cout << V << " => " << ( V.isValid()  ? "OK" : "ERROR" ) << std::endl;
        if ( V.size() != X.size() )
          {
            std::cout << "Bad count #V=" << V.size() << " #X=" << X.size()
                      << std::endl;
            for ( auto j : X ) std::cout << " " << j;
            std::cout << std::endl;
            break;
          }
      }
    THEN( "The object remains consistent." ) {
      REQUIRE( nb == nb_ok );
      REQUIRE( nb == nb_count_ok );
    }
  }

  WHEN( "Erasing many intervals" ) {
    int nb = 1000;
    int nb_ok = 0;
    int nb_count_ok = 0;
    std::set< int > X;
    V.insert( 0, 999 );
    for ( int i = 0; i < 1000; i++ ) X.insert( i );
    for ( int i = 0; i < nb; i++ )
      {
        int f = rand() % 1000;
        int l = std::min( f + rand() % 10, 999 );
        // std::cout << "V + (" << f << "," << l << ") = ";
        V.erase( f, l );
        for ( int k = f; k <= l; k++ ) X.erase( k );
        nb_ok += V.isValid() ? 1 : 0;
        nb_count_ok += ( V.size() == X.size() ) ? 1 : 0;
        if ( ! V.isValid() )
          std::cout << V << " => " << ( V.isValid()  ? "OK" : "ERROR" ) << std::endl;
        if ( V.size() != X.size() )
          {
            std::cout << "Bad count #V=" << V.size() << " #X=" << X.size()
                      << std::endl;
            for ( auto j : X ) std::cout << " " << j;
            std::cout << std::endl;
            break;
          }
      }
    THEN( "The object remains consistent." ) {
      REQUIRE( nb == nb_ok );
      REQUIRE( nb == nb_count_ok );
    }
  }
}

SCENARIO( "IntegralIntervals< int > set operations tests", "[intervals]" )
{
  typedef int Integer;
  typedef IntegralIntervals< Integer > Intervals;

  std::set< int > X,Y;
  for ( int i = 0; i < 1000; i++ ) X.insert( rand() % 1000 );
  for ( int i = 0; i < 1000; i++ ) Y.insert( rand() % 1000 );
  Intervals A( X.cbegin(), X.cend() );
  Intervals B( Y.cbegin(), Y.cend() );
  std::vector< int > X_cup_Y, X_cap_Y, X_minus_Y, X_delta_Y;
  std::set_union( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                  std::back_inserter( X_cup_Y ) );
  std::set_intersection( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                         std::back_inserter( X_cap_Y ) );
  std::set_difference( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                       std::back_inserter( X_minus_Y ) );
  std::set_symmetric_difference( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                                 std::back_inserter( X_delta_Y ) );
  Intervals A_cup_B = A.set_union( B );
  Intervals A_cap_B = A.set_intersection( B );
  Intervals A_minus_B = A.set_difference( B );
  Intervals A_delta_B = A.set_symmetric_difference( B );
  THEN( "Interval can be constructed from sets" ) {
    REQUIRE( X.size() ==  A.size() );
    REQUIRE( Y.size() ==  B.size() );
  }
  THEN( "Set operations on intervals are correct" ) {
    REQUIRE( X_cup_Y.size()   ==  A_cup_B.size() );
    REQUIRE( X_cap_Y.size()   ==  A_cap_B.size() );
    REQUIRE( X_minus_Y.size() ==  A_minus_B.size() );
    REQUIRE( X_delta_Y.size() ==  A_delta_B.size() );
  }
  THEN( "Inclusions are correct" ) {
    REQUIRE( A_cup_B.includes( A_cup_B ) );
    REQUIRE( A_cup_B.includes( A ) );
    REQUIRE( ! A_cup_B.equals( A ) );
    REQUIRE( ! A.includes( A_cup_B ) );
    REQUIRE( A_cup_B.includes( B ) );
    REQUIRE( ! B.includes( A_cup_B ) );
    REQUIRE( ! B.equals( A_cup_B ) );
    REQUIRE( A_cup_B.includes( A_cap_B ) );
    REQUIRE( ! A_cap_B.includes( A_cup_B ) );
    REQUIRE( A.includes( A_minus_B ) );
    REQUIRE( ! A_minus_B.includes( A ) );
    REQUIRE( A_cup_B.includes( A_delta_B ) );
    REQUIRE( ! A_delta_B.includes( A_cup_B ) );
    REQUIRE( ! A.includes( A_delta_B ) );
    REQUIRE( ! B.includes( A_delta_B ) );
  }
}
