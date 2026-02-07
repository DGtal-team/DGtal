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
    for ( unsigned int i = 0; i < nb; i++ )
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
    for ( size_t i = 0; i < vec_L.size(); i++ )
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
  THEN( "Inclusions are correct" ) {
    REQUIRE( ! A.equals( B ) );
    REQUIRE( A_cup_B.equals( A_cup_B ) );
    REQUIRE( A_cup_B.includes( A_cup_B ) );
    REQUIRE( A_cup_B.includes( A ) );
    REQUIRE( ! A_cup_B.equals( A ) );
    REQUIRE( ! A.includes( A_cup_B ) );
    REQUIRE( A_cup_B.includes( B ) );
    REQUIRE( ! B.includes( A_cup_B ) );
    REQUIRE( ! A_cup_B.equals( B ) );
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

SCENARIO( "LatticeSetByIntervals< int > 3d topology operations tests", "[lattice_set][3d]" )
{
  typedef DGtal::Z3i::Space   Space;
  typedef Space::Point Point;
  typedef LatticeSetByIntervals< Space > LatticeSet;
  WHEN( "Computing the star of n isolated points, the number of cells is 27*n, and taking its skel brings back the n points" ) {
    std::vector< Point > X { {0,0,0}, {10,0,0}, {5,5,0}, {0,0,8} };
    LatticeSet P( X.cbegin(), X.cend() );
    auto StarP = P.starOfPoints();
    auto SkelStarP = StarP.skeletonOfCells();
    REQUIRE( P.size() == X.size() );
    REQUIRE( StarP.size() == 27 * X.size() );
    REQUIRE( SkelStarP.size() == X.size() );
  }
  WHEN( "Computing the star of n points consecutive along space diagonal, the number of cells is 26*n+1, and taking its skel brings back the n points" ) {
    std::vector< Point > X { {0,0,0}, {1,1,1}, {2,2,2}, {3,3,3} };
    LatticeSet P( X.cbegin(), X.cend() );
    auto StarP = P.starOfPoints();
    auto SkelStarP = StarP.skeletonOfCells();
    REQUIRE( P.size() == X.size() );
    REQUIRE( StarP.size() == ( 26 * X.size() + 1 ) );
    REQUIRE( SkelStarP.size() == X.size() );
  }
  WHEN( "Computing the star of n points consecutive along any axis, the number of cells is 18*n+9, and taking its skel brings back the n points" ) {
    std::vector< Point > X { {0,0,0}, {1,0,0}, {2,0,0}, {3,0,0} };
    for ( Dimension a = 0; a < 3; a++ )
      {
        LatticeSet P( X.cbegin(), X.cend(), a );
        auto StarP = P.starOfPoints();
        auto SkelStarP = StarP.skeletonOfCells();
        CAPTURE( StarP.toPointRange() );
        CAPTURE( SkelStarP.toPointRange() );
        CAPTURE( a );
        REQUIRE( P.size() == X.size() );
        REQUIRE( StarP.size() == ( 18 * X.size() + 9 ) );
        REQUIRE( SkelStarP.size() == X.size() );
      }
  }
  WHEN( "Computing the skeleton of the star of a random set of points X, Skel(Star(X)) = X" ) {
    std::vector< Point > X;
    for ( int i = 0; i < 1000; i++ )
      X.push_back( Point( rand() % 10, rand() % 10, rand() % 10 ) );
    for ( Dimension a = 0; a < 3; a++ )
      {
        LatticeSet P( X.cbegin(), X.cend(), a );
        auto StarP = P.starOfPoints();
        auto SkelStarP = StarP.skeletonOfCells();
        REQUIRE( SkelStarP.size() == P.size() );
      }
  }
  WHEN( "Computing the skeleton of an open random set of cells O, O = Star(O), Skel(O) subset O and O = Star(Skel(O))" ) {
    std::vector< Point > X;
    for ( int i = 0; i < 50; i++ )
      X.push_back( Point( rand() % 5, rand() % 5, rand() % 5 ) );
    for ( Dimension a = 0; a < 3; a++ )
      {
        LatticeSet C( X.cbegin(), X.cend(), a );
        auto O     = C.starOfCells();
        auto StarO = O.starOfCells();
        auto SkelO = O.skeletonOfCells();
        auto StarSkelO = SkelO.starOfCells();
        CAPTURE( O.axis() );
        CAPTURE( O.size() );
        CAPTURE( SkelO.size() );
        CAPTURE( StarSkelO.size() );
        CAPTURE( StarO.size() );
        REQUIRE( O.includes( SkelO ) );
        REQUIRE( StarSkelO.equals( O ) );
        REQUIRE( StarO.equals( O ) );
      }
  }
  WHEN( "Computing the skeleton of a random set of cells C, C subset Star(C), Skel(C) subset C, C subset Star(Skel(C)), Star(C) = Star(Skel(Star(C)))" ) {
    std::vector< Point > X;
    for ( int i = 0; i < 50; i++ )
      X.push_back( Point( rand() % 5, rand() % 5, rand() % 5 ) );
    for ( Dimension a = 0; a < 3; a++ )
      {
        LatticeSet C( X.cbegin(), X.cend(), a );
        auto StarC = C.starOfCells();
        auto SkelC = C.skeletonOfCells();
        auto StarSkelC = SkelC.starOfCells();
        auto StarSkelStarC = StarC.skeletonOfCells().starOfCells();
        CAPTURE( C.axis() );
        CAPTURE( C.size() );
        CAPTURE( SkelC.size() );
        CAPTURE( StarSkelC.size() );
        CAPTURE( StarC.size() );
        CAPTURE( StarSkelStarC.size() );
        REQUIRE( StarC.includes( C ) );
        REQUIRE( C.includes( SkelC) );
        REQUIRE( StarSkelC.includes( C ) );
        REQUIRE( StarSkelStarC.equals( StarC ) );
      }
  }
}

SCENARIO( "LatticeSetByIntervals< int > 2d topology operations tests", "[lattice_set][2d]" )
{
  typedef DGtal::Z2i::Space   Space;
  typedef Space::Point Point;
  typedef LatticeSetByIntervals< Space > LatticeSet;
  WHEN( "Computing the skeleton of an open random set of cells O, O = Star(O), Skel(O) subset O and O = Star(Skel(O))" ) {
    std::vector< Point > X;
    for ( int i = 0; i < 30; i++ )
      X.push_back( Point( rand() % 10, rand() % 10 ) );
    for ( Dimension a = 0; a < 2; a++ )
      {
        LatticeSet C( X.cbegin(), X.cend(), a );
        auto O     = C.starOfCells();
        auto StarO = O.starOfCells();
        auto SkelO = O.skeletonOfCells();
        auto StarSkelO = SkelO.starOfCells();
        auto debug = StarO.set_symmetric_difference( O );
        CAPTURE( O.axis() );
        CAPTURE( O.size() );
        CAPTURE( StarO.size() );
        CAPTURE( SkelO.size() );
        CAPTURE( StarSkelO.size() );
        CAPTURE( O.toPointRange() );
        CAPTURE( StarO.toPointRange() );
        CAPTURE( SkelO.toPointRange() );
        CAPTURE( StarSkelO.toPointRange() );
        CAPTURE( debug.toPointRange() );
        REQUIRE( O.includes( SkelO ) );
        REQUIRE( StarSkelO.equals( O ) );
        REQUIRE( StarO.equals( O ) );
      }
  }
  WHEN( "Computing the skeleton of a random set of cells C, C subset Star(C), Skel(C) subset C, C subset Star(Skel(C)), Star(C) = Star(Skel(Star(C)))" ) {
    std::vector< Point > X;
    for ( int i = 0; i < 10; i++ )
      X.push_back( Point( rand() % 10, rand() % 10 ) );
    for ( Dimension a = 0; a < 2; a++ )
      {
        LatticeSet C( X.cbegin(), X.cend(), a );
        auto StarC = C.starOfCells();
        auto SkelC = C.skeletonOfCells();
        auto StarSkelC = SkelC.starOfCells();
        auto StarSkelStarC = StarC.skeletonOfCells().starOfCells();
        CAPTURE( C.axis() );
        CAPTURE( C.size() );
        CAPTURE( SkelC.size() );
        CAPTURE( StarSkelC.size() );
        CAPTURE( StarC.size() );
        CAPTURE( StarSkelStarC.size() );
        REQUIRE( StarC.includes( C ) );
        REQUIRE( C.includes( SkelC) );
        REQUIRE( StarSkelC.includes( C ) );
        REQUIRE( StarSkelStarC.equals( StarC ) );
      }
    WHEN( "Computing the extrema of a set of cells C, extremas are correct" ) {
      std::vector< Point > XX;
      XX.push_back( Point(10,-2) );
      XX.push_back( Point(5,5) );
      XX.push_back( Point(4,5) );
      XX.push_back( Point(3,5) );
      XX.push_back( Point(2,1) );
      XX.push_back( Point(2,3) );
      XX.push_back( Point(1,1) );
      XX.push_back( Point(-2,3) );
      XX.push_back( Point(-3,2) );
      LatticeSet C( XX.cbegin(), XX.cend(), 0 );
      auto ExtrC = C.extremaOfCells();
      CAPTURE( C.toPointRange() );
      CAPTURE( ExtrC );
      REQUIRE( ExtrC.size() ==  14 );
    }
  }
}
