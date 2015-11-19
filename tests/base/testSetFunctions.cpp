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
 * @file testCubicalComplex.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/08/28
 *
 * Functions for testing class CubicalComplex.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <random>
#include <boost/unordered_set.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/base/SetFunctions.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace DGtal::functions;
using namespace DGtal::functions::setops;

TEMPLATE_TEST_CASE_4( "SetFunctions module unit tests", "[set_functions]",
                      Container, 
                      std::vector<int>, 
                      std::list<int>, 
                      std::set<int>, 
                      boost::unordered_set<int> )
{
  // typedef vector<int> Container;
  int S1[ 10 ] = { 4, 15, 20, 17, 9, 7, 13, 12, 1, 3 }; 
  int S2[ 6 ]  = { 17, 14, 19, 2, 3, 4 };
  Container C1( S1, S1 + 10 );
  Container C2( S2, S2 + 6 );
  Container C1_minus_C2 = C1 - C2;
  REQUIRE( C1_minus_C2.size() == 7 );
  Container C2_minus_C1 = C2 - C1;
  REQUIRE( C2_minus_C1.size() == 3 );
  REQUIRE( ( C1_minus_C2 - C1 ).size() == 0 );
  REQUIRE( ( C2_minus_C1 - C2 ).size() == 0 );
  REQUIRE( ( C1_minus_C2 - C2 ).size() == C1_minus_C2.size() );
  REQUIRE( ( C2_minus_C1 - C1 ).size() == C2_minus_C1.size() );
  Container C1_union_C2 = C1 | C2;
  Container C2_union_C1 = C2 | C1;
  REQUIRE( C1_union_C2.size() == 13 );
  REQUIRE( C1_union_C2.size() == C2_union_C1.size() );
  REQUIRE( ( C1_minus_C2 | C2 ).size() == (C2_minus_C1 | C1 ).size() );

  Container C1_intersection_C2 = C1 & C2;
  Container C2_intersection_C1 = C2 & C1;
  REQUIRE( C1_intersection_C2.size() == 3 );
  REQUIRE( C1_intersection_C2.size() == C2_intersection_C1.size() );

  REQUIRE( ( C1_minus_C2 | C1_intersection_C2 | C2_minus_C1 ).size() == C1_union_C2.size() );

  Container C1_symdiff_C2 = C1 ^ C2;
  Container C2_symdiff_C1 = C2 ^ C1;
  REQUIRE( C1_symdiff_C2.size() == C2_symdiff_C1.size() );
  REQUIRE( C1_symdiff_C2.size() == ( C1_union_C2 - C1_intersection_C2 ).size() );
  REQUIRE( C1_symdiff_C2.size() == ( C1_minus_C2 | C2_minus_C1 ).size() );
}

static const int NB = 1000000;

TEMPLATE_TEST_CASE_2( "SetFunctions module benchmark tests (sets)", "[set_functions]",
                      Container, 
                      std::set<int>, 
                      boost::unordered_set<int> )
{
  typedef typename Container::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( random() % NB );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( random() % NB );
  Container A( S1.begin(), S1.end() );
  Container B( S2.begin(), S2.end() );
  Container AorB, AminusB, BminusA, AandB, AxorB;

  SECTION( "  - benchmark set operators | - & ^ " )
    {
      AorB    = A | B;
      AminusB = A - B;
      BminusA = B - A;
      AandB   = A & B;
      AxorB   = A ^ B;
    }
  Size size_A       = A.size();
  Size size_B       = B.size();
  Size size_AorB    = AorB.size();
  Size size_AminusB = AminusB.size();
  Size size_BminusA = BminusA.size();
  Size size_AandB   = AandB.size();
  Size size_AxorB   = AxorB.size();
  REQUIRE( size_AorB    >= std::max( size_A, size_B ) );
  REQUIRE( size_AminusB <= size_A );
  REQUIRE( size_BminusA <= size_B );
  REQUIRE( size_AandB   <= std::min( size_A, size_B ) );
  REQUIRE( size_AandB + size_AxorB == size_AorB );
}

TEMPLATE_TEST_CASE_1( "SetFunctions module benchmark tests (sequences)", "[set_functions]",
                      Container, 
                      std::vector<int> )
{
  typedef typename Container::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( random() % NB );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( random() % NB );
  Container A( S1.begin(), S1.end() );
  Container B( S2.begin(), S2.end() );
  Container AorB, AminusB, BminusA, AandB, AxorB;

  std::random_shuffle( A.begin(), A.end() );
  std::random_shuffle( B.begin(), B.end() );

  SECTION( "  - benchmark set operators | - & ^ " )
    {
      AorB    = A | B;
      AminusB = A - B;
      BminusA = B - A;
      AandB   = A & B;
      AxorB   = A ^ B;
    }
  Size size_A       = A.size();
  Size size_B       = B.size();
  Size size_AorB    = AorB.size();
  Size size_AminusB = AminusB.size();
  Size size_BminusA = BminusA.size();
  Size size_AandB   = AandB.size();
  Size size_AxorB   = AxorB.size();
  REQUIRE( size_AorB    >= std::max( size_A, size_B ) );
  REQUIRE( size_AminusB <= size_A );
  REQUIRE( size_BminusA <= size_B );
  REQUIRE( size_AandB   <= std::min( size_A, size_B ) );
  REQUIRE( size_AandB + size_AxorB == size_AorB );
}
