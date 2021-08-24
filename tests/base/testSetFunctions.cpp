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
#include <unordered_set>
#include "DGtal/base/Common.h"
#include "DGtal/base/SetFunctions.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace DGtal::functions;
//using  DGtal::functions::setops::operator|;
using  DGtal::functions::setops::operator&;
using  DGtal::functions::setops::operator-;
using  DGtal::functions::setops::operator^;


////////////////////////////// unit tests /////////////////////////////////
TEMPLATE_TEST_CASE( "SetFunctions module unit tests", "[set_functions]",
                    std::vector<int>,
                    std::list<int>,
                    std::set<int>,
                    std::unordered_set<int> )

{
  int S1[ 10 ] = { 4, 15, 20, 17, 9, 7, 13, 12, 1, 3 }; 
  int S2[ 6 ]  = { 17, 14, 19, 2, 3, 4 };
  TestType C1( S1, S1 + 10 );
  TestType C2( S2, S2 + 6 );
  TestType C1_minus_C2 = C1 - C2;
  REQUIRE( C1_minus_C2.size() == 7 );
  TestType C2_minus_C1 = C2 - C1;
  REQUIRE( C2_minus_C1.size() == 3 );
  REQUIRE( ( C1_minus_C2 - C1 ).size() == 0 );
  REQUIRE( ( C2_minus_C1 - C2 ).size() == 0 );
  REQUIRE( ( C1_minus_C2 - C2 ).size() == C1_minus_C2.size() );
  REQUIRE( ( C2_minus_C1 - C1 ).size() == C2_minus_C1.size() );
  TestType C1_union_C2 = DGtal::functions::setops::operator|(C1 , C2);
  TestType C2_union_C1 = DGtal::functions::setops::operator|(C2 , C1);
  REQUIRE( C1_union_C2.size() == 13 );
  REQUIRE( C1_union_C2.size() == C2_union_C1.size() );
  REQUIRE( ( DGtal::functions::setops::operator|(C1_minus_C2 , C2) ).size() == (DGtal::functions::setops::operator|(C2_minus_C1 , C1) ).size() );

  TestType C1_intersection_C2 = C1 & C2;
  TestType C2_intersection_C1 = C2 & C1;
  REQUIRE( C1_intersection_C2.size() == 3 );
  REQUIRE( C1_intersection_C2.size() == C2_intersection_C1.size() );

  REQUIRE( ( DGtal::functions::setops::operator|(DGtal::functions::setops::operator|(C1_minus_C2 , C1_intersection_C2) , C2_minus_C1) ).size() == C1_union_C2.size() );

  TestType C1_symdiff_C2 = C1 ^ C2;
  TestType C2_symdiff_C1 = C2 ^ C1;
  REQUIRE( C1_symdiff_C2.size() == C2_symdiff_C1.size() );
  REQUIRE( C1_symdiff_C2.size() == ( C1_union_C2 - C1_intersection_C2 ).size() );
  REQUIRE( C1_symdiff_C2.size() == ( DGtal::functions::setops::operator|(C1_minus_C2 , C2_minus_C1) ).size() );

  REQUIRE( isEqual( C1_symdiff_C2, C1_union_C2 - C1_intersection_C2 ) );
  REQUIRE( isEqual( C1_symdiff_C2, DGtal::functions::setops::operator|(C1_minus_C2 , C2_minus_C1) ) );
  REQUIRE( isEqual( DGtal::functions::setops::operator|(DGtal::functions::setops::operator|(C1_minus_C2 , C1_intersection_C2) , C2_minus_C1), C1_union_C2 ) );
  REQUIRE(   isSubset( C1_minus_C2, C1 ) );
  REQUIRE( ! isSubset( C1_minus_C2, C2 ) );
  REQUIRE(   isSubset( C2_minus_C1, C2 ) );
  REQUIRE( ! isSubset( C2_minus_C1, C1 ) );
  REQUIRE(   isSubset( C1, C1_union_C2 ) );
  REQUIRE(   isSubset( C2, C1_union_C2 ) );
  REQUIRE(   isSubset( C1_intersection_C2, C1 ) );
  REQUIRE(   isSubset( C1_intersection_C2, C2 ) );
  REQUIRE(   isSubset( C1_symdiff_C2, C1_union_C2 ) );
  REQUIRE( ! isSubset( C1, C1_symdiff_C2 ) );
  REQUIRE( ! isSubset( C2, C1_symdiff_C2 ) );
}


static const int NB = 10000;
static std::default_random_engine generator;
static std::uniform_int_distribution<int> distribution(1,NB);

int randomNB( )
{
  return distribution(generator);
}

////////////////////////////// operator | //////////////////////////////
TEMPLATE_TEST_CASE( "SetFunctions benchmark operator | (sequences)", "[set_functions]",
                    std::vector<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AorB;
  std::random_device rd;
  std::mt19937 g(rd());
  
  std::shuffle( A.begin(), A.end(), g);
  std::shuffle( B.begin(), B.end(), g);

  SECTION( "  - benchmark set operators |" )
    {
      AorB    = DGtal::functions::setops::operator|(A , B);
    }
  Size size_A       = A.size();
  Size size_B       = B.size();
  Size size_AorB    = AorB.size();
  REQUIRE( size_AorB    >= std::max( size_A, size_B ) );
}

TEMPLATE_TEST_CASE( "SetFunctions benchmark operator | (sets)", "[set_functions]",
                    std::set<int>,
                    std::unordered_set<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AorB;

  SECTION( "  - benchmark set operators |" )
    {
      AorB    = DGtal::functions::setops::operator|(A , B);
    }
  Size size_A       = A.size();
  Size size_B       = B.size();
  Size size_AorB    = AorB.size();
  REQUIRE( size_AorB    >= std::max( size_A, size_B ) );
}

////////////////////////////// operator & //////////////////////////////
TEMPLATE_TEST_CASE( "SetFunctions benchmark operator & (sequences)", "[set_functions]",
                    std::vector<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AandB;
  std::random_device rd;
  std::mt19937 g(rd());
  
  std::shuffle( A.begin(), A.end(), g );
  std::shuffle( B.begin(), B.end(), g );

  SECTION( "  - benchmark set operators &" )
    {
      AandB    = A & B;
    }
  Size size_A        = A.size();
  Size size_B        = B.size();
  Size size_AandB    = AandB.size();
  REQUIRE( size_AandB   <= std::min( size_A, size_B ) );
}

TEMPLATE_TEST_CASE( "SetFunctions benchmark operator & (sets)", "[set_functions]",
                    std::set<int>,
                    std::unordered_set<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AandB;

  SECTION( "  - benchmark set operators &" )
    {
      AandB    = A & B;
    }
  Size size_A        = A.size();
  Size size_B        = B.size();
  Size size_AandB    = AandB.size();
  REQUIRE( size_AandB   <= std::min( size_A, size_B ) );
}


////////////////////////////// operator - //////////////////////////////
TEMPLATE_TEST_CASE( "SetFunctions benchmark operator - (sequences)", "[set_functions]",
                    std::vector<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AminusB;
  std::random_device rd;
  std::mt19937 g(rd());
  
  std::shuffle( A.begin(), A.end(), g );
  std::shuffle( B.begin(), B.end(), g );

  SECTION( "  - benchmark set operators -" )
    {
      AminusB    = A - B;
    }
  Size size_A        = A.size();
  Size size_B        = B.size();
  Size size_AminusB  = AminusB.size();
  REQUIRE( size_AminusB <= size_A );
  boost::ignore_unused_variable_warning(size_B);
}

TEMPLATE_TEST_CASE( "SetFunctions benchmark operator - (sets)", "[set_functions]",
                    std::set<int>,
                    std::unordered_set<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AminusB;

  SECTION( "  - benchmark set operators -" )
    {
      AminusB    = A - B;
    }
  Size size_A        = A.size();
  Size size_B        = B.size();
  Size size_AminusB  = AminusB.size();
  REQUIRE( size_AminusB <= size_A );
  boost::ignore_unused_variable_warning(size_B);
}


////////////////////////////// operator ^ //////////////////////////////
TEMPLATE_TEST_CASE( "SetFunctions benchmark operator ^ (sequences)", "[set_functions]",
                    std::vector<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AxorB;
  std::random_device rd;
  std::mt19937 g(rd());
  
  std::shuffle( A.begin(), A.end(), g );
  std::shuffle( B.begin(), B.end(), g );

  SECTION( "  - benchmark set operators ^" )
    {
      AxorB    = A ^ B;
    }
  Size size_A        = A.size();
  Size size_B        = B.size();
  Size size_AxorB    = AxorB.size();
  REQUIRE( size_AxorB   <= std::max( size_A, size_B ) );
}

TEMPLATE_TEST_CASE( "SetFunctions benchmark operator ^ (sets)", "[set_functions]",
                    std::set<int>,
                    std::unordered_set<int> )
{
  typedef typename TestType::size_type Size;
  std::set<int> S1; 
  for ( int i = 0; i < NB; ++i ) S1.insert( randomNB(  ) );
  std::set<int> S2; 
  for ( int i = 0; i < NB; ++i ) S2.insert( randomNB(  ) );
  TestType A( S1.begin(), S1.end() );
  TestType B( S2.begin(), S2.end() );
  TestType AxorB;

  SECTION( "  - benchmark set operators ^" )
    {
      AxorB    = A ^ B;
    }
  Size size_A        = A.size();
  Size size_B        = B.size();
  Size size_AxorB    = AxorB.size();
  REQUIRE( size_AxorB   <= std::max( size_A, size_B ) );
}




