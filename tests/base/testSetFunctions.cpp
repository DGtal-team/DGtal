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
#include <boost/unordered_set.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/base/SetFunctions.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace DGtal::functions;
using namespace DGtal::functions::setops;

TEMPLATE_TEST_CASE_3( "SetFunctions module unit tests", "[set_functions]",
                      Container, 
                      std::vector<int>, 
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

}
