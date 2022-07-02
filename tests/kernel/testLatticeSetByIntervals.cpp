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
 * @file testLatticeSetByIntervals.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/07/02
 *
 * Functions for testing class LatticeSetByIntervals.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/LatticeSetByIntervals.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LatticeSetByIntervals.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "LatticeSetByIntervals< int > unit tests", "[lattice_set]" )
{
  typedef DGtal::Z3i::Space   Space;
  typedef Space::Point Point;
  typedef LatticeSetByIntervals< Space > LatticeSet;

  WHEN( "Inserting many points" ) {
    std::set< Point > S;
    LatticeSet        L;
    unsigned int nb = 10000;
    for ( auto i = 0; i < nb; i++ )
      {
        Point p( rand() % 20, rand() % 20, rand() % 20 );
        S.insert( p );
        L.insert( p );
      }
    auto mem_L = L.memory_usage();
    auto mem_S = ( L.size()  * ( sizeof( Point ) + sizeof( void* ) ) );
    auto vec_L = L.toPointRange();
    std::sort( vec_L.begin(), vec_L.end() );
    std::vector< Point > vec_S( S.begin(), S.end() );
    std::cout << "mem(L)=" << mem_L << std::endl;
    std::cout << "mem(S)=" << mem_S << std::endl;
    for ( auto i = 0; i < vec_L.size(); i++ )
      {
        if ( vec_L[ i ] != vec_S[ i ] )
          std::cout << i << " " << vec_L[ i ] << " != " << vec_S[ i ]
                    << std::endl;
      }
    THEN( "The lattice set contains the same points as std::set< Point >" ) {
      REQUIRE( S.size() == L.size() );
      REQUIRE( L.size() == vec_L.size() );
      REQUIRE( std::equal( vec_L.begin(), vec_L.end(), vec_S.begin() ) );
    }
    THEN( "The lattice set is less costly to store than std::set< Point >" ) {
      REQUIRE( mem_L < mem_S );
    }
    THEN( "One can create directly a lattice set from a range" ) {
      LatticeSet L2( S.begin(), S.end() );
      REQUIRE( L2.size() == S.size() );
    }
    THEN( "When erasing all elements from the lattice set, it becomes empty" ) {
      LatticeSet L2( S.begin(), S.end() );
      for ( auto&& p : S )
        L2.erase( p );
      REQUIRE( L2.empty() );
      REQUIRE( L2.data().empty() );
    }
  }
}

SCENARIO( "LatticeSetByIntervals< int > set operations tests", "[lattice_set]" )
{
  typedef DGtal::Z3i::Space   Space;
  typedef Space::Point Point;
  typedef LatticeSetByIntervals< Space > LatticeSet;

  std::set< Point > X,Y;
  int nb = 300000;
  int R  = 50;
  for ( auto i = 0; i < nb; i++ )
    {
      Point p( rand() % R, rand() % R, rand() % R );
      X.insert( p );
      Point q( rand() % R, rand() % R, rand() % R );
      Y.insert( q );
    }
  LatticeSet A( X.cbegin(), X.cend() );
  LatticeSet B( Y.cbegin(), Y.cend() );
  std::vector< Point > X_cup_Y, X_cap_Y, X_minus_Y, X_delta_Y;
  Clock c;
  c.startClock();
  std::set_union( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                  std::back_inserter( X_cup_Y ) );
  std::set_intersection( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                         std::back_inserter( X_cap_Y ) );
  std::set_difference( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                       std::back_inserter( X_minus_Y ) );
  std::set_symmetric_difference( X.cbegin(), X.cend(), Y.cbegin(), Y.cend(),
                                 std::back_inserter( X_delta_Y ) );
  auto tv = c.stopClock();
  
  c.startClock();
  LatticeSet A_cup_B = A.set_union( B );
  LatticeSet A_cap_B = A.set_intersection( B );
  LatticeSet A_minus_B = A.set_difference( B );
  LatticeSet A_delta_B = A.set_symmetric_difference( B );
  auto tl = c.stopClock();
  THEN( "Lattice sets can be constructed from sets" ) {
    REQUIRE( X.size() ==  A.size() );
    REQUIRE( Y.size() ==  B.size() );
  }
  THEN( "Set operations on lattice sets are correct" ) {
    REQUIRE( X_cup_Y.size()   ==  A_cup_B.size() );
    REQUIRE( X_cap_Y.size()   ==  A_cap_B.size() );
    REQUIRE( X_minus_Y.size() ==  A_minus_B.size() ); 
    REQUIRE( X_delta_Y.size() ==  A_delta_B.size() ); 
  }
  THEN( "Set operations on lattice sets are faster" ) {
    REQUIRE( tl < tv );
  }
}
