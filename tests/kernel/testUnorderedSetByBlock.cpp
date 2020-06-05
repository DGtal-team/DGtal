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
 * @file testUnorderedSetByBlock.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/01/04
 *
 * Functions for testing class UnorderedSetByBlock.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/UnorderedSetByBlock.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

template < typename Point >
Point randomPoint( int S )
{
  Point p;
  for ( DGtal::Dimension i = 0; i < Point::dimension; i++ )
    p[ i ] = random() % S;
  return p;
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class UnorderedSetByBlock.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "UnorderedSetByBlock< PointVector< 2, int > unit tests", "[unorderedsetbyblock][2d]" )
{
  typedef PointVector< 2, int32_t >        Point;
  typedef std::unordered_set< Point >      StdUnorderedSet;
  typedef UnorderedSetByBlock< Point >     BlockUnorderedSet;

  const int nb_inserted = 100;
  const int nb_sought   = 200;

  StdUnorderedSet   stdSet;
  BlockUnorderedSet blkSet;
  for ( int i = 0; i < nb_inserted; i++ )
    {
      Point p = randomPoint<Point>( 10 );
      stdSet.insert( p );
      blkSet.insert( p );
    }
  // Check conversion iterator -> const_iterator
  BlockUnorderedSet::iterator       itB   = blkSet.begin();
  BlockUnorderedSet::const_iterator citB  = blkSet.begin();
  BlockUnorderedSet::const_iterator citBp = blkSet.cbegin();
  bool ok1  =  itB == blkSet.begin();  
  bool ok2  =  itB == blkSet.cbegin();  
  bool ok3  = citB == blkSet.begin();  
  bool ok4  = citB == blkSet.cbegin();  
  WHEN( "Inserting identical elements in std::unordered_set<> and in UnorderedSetByBlock<>, they contains the same number of elements" ) {
    REQUIRE( blkSet.size() <= nb_inserted   );
    REQUIRE( blkSet.size() == stdSet.size() );
  }
  WHEN( "Cloning identical std::unordered_set<> and UnorderedSetByBlock<>, they contains the same number of elements" ) {
    StdUnorderedSet   stdSet2( stdSet );
    BlockUnorderedSet blkSet2( blkSet );
    REQUIRE( blkSet2.size() == blkSet.size()  );
    REQUIRE( blkSet2.size() == stdSet2.size() );
  }
  WHEN( "Traversing identical std::unordered_set<> and UnorderedSetByBlock<>, they traverse the same elements" ) {
    std::vector< Point > stdTrv;
    std::vector< Point > blkTrv;
    for ( auto&& p : stdSet ) stdTrv.push_back( p );
    for ( auto&& p : blkSet ) blkTrv.push_back( p );
    REQUIRE( blkTrv.size() == stdTrv.size() );
    std::sort( stdTrv.begin(), stdTrv.end() );
    std::sort( blkTrv.begin(), blkTrv.end() );
    REQUIRE( std::equal( stdTrv.cbegin(), stdTrv.cend(), blkTrv.cbegin() ) );
  }
  WHEN( "Looking for elements in identical std::unordered_set<> and UnorderedSetByBlock<>, the same elements are find with method `count`" ) {
    std::vector< Point > stdFound, stdNotFound;
    std::vector< Point > blkFound, blkNotFound;
    for ( int i = 0; i < nb_sought; i++ )
      {
	Point p = randomPoint<Point>( 10 );
	if ( stdSet.count( p ) != 0 ) stdFound.push_back( p );
	else stdNotFound.push_back( p );
	if ( blkSet.count( p ) != 0 ) blkFound.push_back( p );
	else blkNotFound.push_back( p );
      }
    REQUIRE( blkFound   .size() == stdFound   .size() );
    REQUIRE( blkNotFound.size() == stdNotFound.size() );
    std::sort( stdFound   .begin(), stdFound   .end() );
    std::sort( stdNotFound.begin(), stdNotFound.end() );
    std::sort( blkFound   .begin(), blkFound   .end() );
    std::sort( blkNotFound.begin(), blkNotFound.end() );
    REQUIRE( std::equal( stdFound.cbegin(), stdFound.cend(),
			 blkFound.cbegin() ) );
    REQUIRE( std::equal( stdNotFound.cbegin(), stdNotFound.cend(),
			 blkNotFound.cbegin() ) );
  }
  WHEN( "Looking for elements in identical std::unordered_set<> and UnorderedSetByBlock<>, the same elements are find with method `find`" ) {
    std::vector< Point > stdFound, stdNotFound;
    std::vector< Point > blkFound, blkNotFound;
    int std_nb_value_ok = 0;
    int blk_nb_value_ok = 0;
    for ( int i = 0; i < nb_sought; i++ )
      {
	Point p = randomPoint<Point>( 10 );
	const auto stdIt = stdSet.find( p );
	if ( stdIt != stdSet.end() )
	  {
	    stdFound.push_back( p );
	    std_nb_value_ok += ( *stdIt == p ) ? 1 : 0;
	  }
	else stdNotFound.push_back( p );
	const auto blkIt = blkSet.find( p );
	if ( blkIt != blkSet.end() )
	  {
	    blkFound.push_back( p );
	    blk_nb_value_ok += ( *blkIt == p ) ? 1 : 0;
	  }
	else blkNotFound.push_back( p );
      }
    REQUIRE( blkFound   .size() == stdFound   .size() );
    REQUIRE( blkNotFound.size() == stdNotFound.size() );
    std::sort( stdFound   .begin(), stdFound   .end() );
    std::sort( stdNotFound.begin(), stdNotFound.end() );
    std::sort( blkFound   .begin(), blkFound   .end() );
    std::sort( blkNotFound.begin(), blkNotFound.end() );
    REQUIRE( std::equal( stdFound.cbegin(), stdFound.cend(),
			 blkFound.cbegin() ) );
    REQUIRE( std::equal( stdNotFound.cbegin(), stdNotFound.cend(),
			 blkNotFound.cbegin() ) );
    REQUIRE( std_nb_value_ok == stdFound.size() );
    REQUIRE( blk_nb_value_ok == std_nb_value_ok );
    REQUIRE( blk_nb_value_ok == blkFound.size() );
  }
}




//                                                                           //
///////////////////////////////////////////////////////////////////////////////
