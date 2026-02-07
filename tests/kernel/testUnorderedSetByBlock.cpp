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
#include <iterator>
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
    p[ i ] = (rand() % S) - S/2;
  return p;
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class UnorderedSetByBlock.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "UnorderedSetByBlock< PointVector< 2, int > unit tests with 32 bits blocks", "[unorderedsetbyblock][2d]" )
{
  typedef PointVector< 2, int32_t >        Point;
  typedef std::unordered_set< Point >      StdUnorderedSet;
  typedef UnorderedSetByBlock< Point >     BlockUnorderedSet;

  const size_t nb_inserted = 100;
  const size_t nb_sought   = 200;
  const size_t nb_erased   = 100;

  StdUnorderedSet   stdSet;
  BlockUnorderedSet blkSet;
  for ( size_t i = 0; i < nb_inserted; i++ )
    {
      Point p = randomPoint<Point>( 10 );
      stdSet.insert( p );
      blkSet.insert( p );
    }
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
    for ( size_t i = 0; i < nb_sought; i++ )
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
    size_t std_nb_value_ok = 0;
    size_t blk_nb_value_ok = 0;
    for ( size_t i = 0; i < nb_sought; i++ )
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
  WHEN( "Erasing elements in identical std::unordered_set<> and UnorderedSetByBlock<>, the same elements are left" ) {
    std::vector< Point > stdErase;
    std::vector< Point > blkErase;
    size_t std_nb_erase_ok = 0;
    size_t blk_nb_erase_ok = 0;
    for ( size_t i = 0; i < nb_erased; i++ )
      {
        Point p = randomPoint<Point>( 10 );
        auto stdFindIt   = stdSet.find ( p );
        auto stdIsErased = stdSet.erase( p );
        if ( stdFindIt != stdSet.cend() )
          {
            std_nb_erase_ok  += ( stdIsErased != 0 ) ? 1 : 0;
            stdErase.push_back( p );
          }
        else std_nb_erase_ok += ( stdIsErased == 0 ) ? 1 : 0;
        auto blkFindIt   = blkSet.find ( p );
        auto blkIsErased = blkSet.erase( p );
        if ( blkFindIt != blkSet.cend() )
          {
            blk_nb_erase_ok  += ( blkIsErased != 0 ) ? 1 : 0;
            blkErase.push_back( p );
          }
        else blk_nb_erase_ok += ( blkIsErased == 0 ) ? 1 : 0;
      }
    REQUIRE( blkSet  .size() == stdSet  .size() );
    REQUIRE( blkErase.size() == stdErase.size() );
    REQUIRE( blk_nb_erase_ok == std_nb_erase_ok );
    std::vector< Point > stdTrv;
    std::vector< Point > blkTrv;
    for ( auto&& p : stdSet ) stdTrv.push_back( p );
    for ( auto&& p : blkSet ) blkTrv.push_back( p );
    REQUIRE( blkTrv.size() == stdTrv.size() );
    std::sort( stdTrv.begin(), stdTrv.end() );
    std::sort( blkTrv.begin(), blkTrv.end() );
    REQUIRE( std::equal( stdTrv.cbegin(), stdTrv.cend(), blkTrv.cbegin() ) );
  }
  WHEN( "Erasing a range in identical std::unordered_set<> and UnorderedSetByBlock<>, the same number of elements is left" ) {
    auto nb_std_before = stdSet.size();
    auto nb_blk_before = blkSet.size();
    auto stdItB = stdSet.begin(); std::advance( stdItB, 10 );
    auto blkItB = blkSet.begin(); std::advance( blkItB, 10 );
    auto stdItE = stdItB;         std::advance( stdItE, 20 );
    auto blkItE = blkItB;         std::advance( blkItE, 20 );
    stdSet.erase( stdItB, stdItE );
    blkSet.erase( blkItB, blkItE );
    size_t nb_std = std::distance(stdSet.begin(), stdSet.end());
    size_t nb_blk = std::distance(blkSet.begin(), blkSet.end());
    REQUIRE( stdSet.size() == nb_std );
    REQUIRE( stdSet.size() == nb_std_before - 20 );
    REQUIRE( blkSet.size() == nb_blk );
    REQUIRE( blkSet.size() == nb_blk_before - 20 );
    REQUIRE( blkSet.size() == stdSet.size() );
  }
  THEN( "The memory usage of UnorderedSetByBlock<> is inferior to the one of std::unordered_set<>" ) {
    const auto stdMem = blkSet.memory_usage_unordered_set();
    const auto blkMem = blkSet.memory_usage();
    REQUIRE( blkMem <= stdMem );
  }


  THEN( "Comparisons and valid assignments between iterator and const_iterator should be seamless for the user" ) {
    BlockUnorderedSet::iterator        itB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citBp = blkSet.cbegin();
    BlockUnorderedSet::iterator        itE  = blkSet.end();
    BlockUnorderedSet::const_iterator citE  = blkSet.end();
    BlockUnorderedSet::const_iterator citEp = blkSet.cend();
    REQUIRE(  itB  == blkSet.begin()  );
    REQUIRE(  itB  == blkSet.cbegin() );
    REQUIRE( citB  == blkSet.begin()  );
    REQUIRE( citB  == blkSet.cbegin() );
    REQUIRE( citBp == blkSet.cbegin() );
    REQUIRE(  itE  == blkSet.end()  );
    REQUIRE(  itE  == blkSet.cend() );
    REQUIRE( citE  == blkSet.end()  );
    REQUIRE( citE  == blkSet.cend() );
    REQUIRE( citEp == blkSet.cend() );
    REQUIRE(  itB  !=  itE  );
    REQUIRE(  itB  != citE  );
    REQUIRE(  itB  != citEp );
    REQUIRE( citB  !=  itE  );
    REQUIRE( citB  != citE  );
    REQUIRE( citB  != citEp );
    REQUIRE( citBp !=  itE  );
    REQUIRE( citBp != citE  );
    REQUIRE( citBp != citEp );
  }
}


SCENARIO( "UnorderedSetByBlock< PointVector< 3, int64 > unit tests with 32 bits blocks", "[unorderedsetbyblock][3d]" )
{
  typedef PointVector< 3, int64_t >        Point;
  typedef std::unordered_set< Point >      StdUnorderedSet;
  typedef UnorderedSetByBlock< Point >     BlockUnorderedSet;

  const size_t nb_inserted = 1000;
  const size_t nb_sought   = 2000;
  const size_t nb_erased   = 1000;

  StdUnorderedSet   stdSet;
  BlockUnorderedSet blkSet;
  for ( size_t i = 0; i < nb_inserted; i++ )
    {
      Point p = randomPoint<Point>( 10 );
      stdSet.insert( p );
      blkSet.insert( p );
    }
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
    for ( size_t i = 0; i < nb_sought; i++ )
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
    size_t std_nb_value_ok = 0;
    size_t blk_nb_value_ok = 0;
    for ( size_t i = 0; i < nb_sought; i++ )
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
  WHEN( "Erasing elements in identical std::unordered_set<> and UnorderedSetByBlock<>, the same elements are left" ) {
    std::vector< Point > stdErase;
    std::vector< Point > blkErase;
    size_t std_nb_erase_ok = 0;
    size_t blk_nb_erase_ok = 0;
    for ( size_t i = 0; i < nb_erased; i++ )
      {
        Point p = randomPoint<Point>( 10 );
        auto stdFindIt   = stdSet.find ( p );
        auto stdIsErased = stdSet.erase( p );
        if ( stdFindIt != stdSet.cend() )
          {
            std_nb_erase_ok  += ( stdIsErased != 0 ) ? 1 : 0;
            stdErase.push_back( p );
          }
        else std_nb_erase_ok += ( stdIsErased == 0 ) ? 1 : 0;
        auto blkFindIt   = blkSet.find ( p );
        auto blkIsErased = blkSet.erase( p );
        if ( blkFindIt != blkSet.cend() )
          {
            blk_nb_erase_ok  += ( blkIsErased != 0 ) ? 1 : 0;
            blkErase.push_back( p );
          }
        else blk_nb_erase_ok += ( blkIsErased == 0 ) ? 1 : 0;
      }
    REQUIRE( blkSet  .size() == stdSet  .size() );
    REQUIRE( blkErase.size() == stdErase.size() );
    REQUIRE( blk_nb_erase_ok == std_nb_erase_ok );
    std::vector< Point > stdTrv;
    std::vector< Point > blkTrv;
    for ( auto&& p : stdSet ) stdTrv.push_back( p );
    for ( auto&& p : blkSet ) blkTrv.push_back( p );
    REQUIRE( blkTrv.size() == stdTrv.size() );
    std::sort( stdTrv.begin(), stdTrv.end() );
    std::sort( blkTrv.begin(), blkTrv.end() );
    REQUIRE( std::equal( stdTrv.cbegin(), stdTrv.cend(), blkTrv.cbegin() ) );
  }
  WHEN( "Erasing a range in identical std::unordered_set<> and UnorderedSetByBlock<>, the same number of elements is left" ) {
    auto nb_std_before = stdSet.size();
    auto nb_blk_before = blkSet.size();
    auto stdItB = stdSet.begin(); std::advance( stdItB, 10 );
    auto blkItB = blkSet.begin(); std::advance( blkItB, 10 );
    auto stdItE = stdItB;         std::advance( stdItE, 20 );
    auto blkItE = blkItB;         std::advance( blkItE, 20 );
    stdSet.erase( stdItB, stdItE );
    blkSet.erase( blkItB, blkItE );
    size_t nb_std = std::distance(stdSet.begin(), stdSet.end());
    size_t nb_blk = std::distance(blkSet.begin(), blkSet.end());
    REQUIRE( stdSet.size() == nb_std );
    REQUIRE( stdSet.size() == nb_std_before - 20 );
    REQUIRE( blkSet.size() == nb_blk );
    REQUIRE( blkSet.size() == nb_blk_before - 20 );
    REQUIRE( blkSet.size() == stdSet.size() );
  }
  THEN( "The memory usage of UnorderedSetByBlock<> is inferior to the one of std::unordered_set<>" ) {
    const auto stdMem = blkSet.memory_usage_unordered_set();
    const auto blkMem = blkSet.memory_usage();
    REQUIRE( blkMem <= stdMem );
  }


  THEN( "Comparisons and valid assignments between iterator and const_iterator should be seamless for the user" ) {
    BlockUnorderedSet::iterator        itB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citBp = blkSet.cbegin();
    BlockUnorderedSet::iterator        itE  = blkSet.end();
    BlockUnorderedSet::const_iterator citE  = blkSet.end();
    BlockUnorderedSet::const_iterator citEp = blkSet.cend();
    REQUIRE(  itB  == blkSet.begin()  );
    REQUIRE(  itB  == blkSet.cbegin() );
    REQUIRE( citB  == blkSet.begin()  );
    REQUIRE( citB  == blkSet.cbegin() );
    REQUIRE( citBp == blkSet.cbegin() );
    REQUIRE(  itE  == blkSet.end()  );
    REQUIRE(  itE  == blkSet.cend() );
    REQUIRE( citE  == blkSet.end()  );
    REQUIRE( citE  == blkSet.cend() );
    REQUIRE( citEp == blkSet.cend() );
    REQUIRE(  itB  !=  itE  );
    REQUIRE(  itB  != citE  );
    REQUIRE(  itB  != citEp );
    REQUIRE( citB  !=  itE  );
    REQUIRE( citB  != citE  );
    REQUIRE( citB  != citEp );
    REQUIRE( citBp !=  itE  );
    REQUIRE( citBp != citE  );
    REQUIRE( citBp != citEp );
  }
}

SCENARIO( "UnorderedSetByBlock< PointVector< 2, int > unit tests with 64 bits blocks", "[unorderedsetbyblock][2d]" )
{
  typedef PointVector< 2, int32_t >                Point;
  typedef std::unordered_set< Point >              StdUnorderedSet;
  typedef Splitter< Point, DGtal::uint64_t >       Splitter64;
  typedef UnorderedSetByBlock< Point, Splitter64 > BlockUnorderedSet;

  const size_t nb_inserted = 10000;
  const size_t nb_sought   = 200;
  const size_t nb_erased   = 100;

  StdUnorderedSet   stdSet;
  BlockUnorderedSet blkSet;
  for ( size_t i = 0; i < nb_inserted; i++ )
    {
      Point p = randomPoint<Point>( 200 );
      stdSet.insert( p );
      blkSet.insert( p );
    }
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
    for ( size_t i = 0; i < nb_sought; i++ )
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
    size_t std_nb_value_ok = 0;
    size_t blk_nb_value_ok = 0;
    for ( size_t i = 0; i < nb_sought; i++ )
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
  WHEN( "Erasing elements in identical std::unordered_set<> and UnorderedSetByBlock<>, the same elements are left" ) {
    std::vector< Point > stdErase;
    std::vector< Point > blkErase;
    size_t std_nb_erase_ok = 0;
    size_t blk_nb_erase_ok = 0;
    for ( size_t i = 0; i < nb_erased; i++ )
      {
        Point p = randomPoint<Point>( 10 );
        auto stdFindIt   = stdSet.find ( p );
        auto stdIsErased = stdSet.erase( p );
        if ( stdFindIt != stdSet.cend() )
          {
            std_nb_erase_ok  += ( stdIsErased != 0 ) ? 1 : 0;
            stdErase.push_back( p );
          }
        else std_nb_erase_ok += ( stdIsErased == 0 ) ? 1 : 0;
        auto blkFindIt   = blkSet.find ( p );
        auto blkIsErased = blkSet.erase( p );
        if ( blkFindIt != blkSet.cend() )
          {
            blk_nb_erase_ok  += ( blkIsErased != 0 ) ? 1 : 0;
            blkErase.push_back( p );
          }
        else blk_nb_erase_ok += ( blkIsErased == 0 ) ? 1 : 0;
      }
    REQUIRE( blkSet  .size() == stdSet  .size() );
    REQUIRE( blkErase.size() == stdErase.size() );
    REQUIRE( blk_nb_erase_ok == std_nb_erase_ok );
    std::vector< Point > stdTrv;
    std::vector< Point > blkTrv;
    for ( auto&& p : stdSet ) stdTrv.push_back( p );
    for ( auto&& p : blkSet ) blkTrv.push_back( p );
    REQUIRE( blkTrv.size() == stdTrv.size() );
    std::sort( stdTrv.begin(), stdTrv.end() );
    std::sort( blkTrv.begin(), blkTrv.end() );
    REQUIRE( std::equal( stdTrv.cbegin(), stdTrv.cend(), blkTrv.cbegin() ) );
  }
  WHEN( "Erasing a range in identical std::unordered_set<> and UnorderedSetByBlock<>, the same number of elements is left" ) {
    auto nb_std_before = stdSet.size();
    auto nb_blk_before = blkSet.size();
    auto stdItB = stdSet.begin(); std::advance( stdItB, 10 );
    auto blkItB = blkSet.begin(); std::advance( blkItB, 10 );
    auto stdItE = stdItB;         std::advance( stdItE, 20 );
    auto blkItE = blkItB;         std::advance( blkItE, 20 );
    stdSet.erase( stdItB, stdItE );
    blkSet.erase( blkItB, blkItE );
    size_t nb_std = std::distance(stdSet.begin(), stdSet.end());
    size_t nb_blk = std::distance(blkSet.begin(), blkSet.end());
    REQUIRE( stdSet.size() == nb_std );
    REQUIRE( stdSet.size() == nb_std_before - 20 );
    REQUIRE( blkSet.size() == nb_blk );
    REQUIRE( blkSet.size() == nb_blk_before - 20 );
    REQUIRE( blkSet.size() == stdSet.size() );
  }
  THEN( "The memory usage of UnorderedSetByBlock<> is inferior to the one of std::unordered_set<>" ) {
    const auto stdMem = blkSet.memory_usage_unordered_set();
    const auto blkMem = blkSet.memory_usage();
    REQUIRE( blkMem <= stdMem );
  }


  THEN( "Comparisons and valid assignments between iterator and const_iterator should be seamless for the user" ) {
    BlockUnorderedSet::iterator        itB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citBp = blkSet.cbegin();
    BlockUnorderedSet::iterator        itE  = blkSet.end();
    BlockUnorderedSet::const_iterator citE  = blkSet.end();
    BlockUnorderedSet::const_iterator citEp = blkSet.cend();
    REQUIRE(  itB  == blkSet.begin()  );
    REQUIRE(  itB  == blkSet.cbegin() );
    REQUIRE( citB  == blkSet.begin()  );
    REQUIRE( citB  == blkSet.cbegin() );
    REQUIRE( citBp == blkSet.cbegin() );
    REQUIRE(  itE  == blkSet.end()  );
    REQUIRE(  itE  == blkSet.cend() );
    REQUIRE( citE  == blkSet.end()  );
    REQUIRE( citE  == blkSet.cend() );
    REQUIRE( citEp == blkSet.cend() );
    REQUIRE(  itB  !=  itE  );
    REQUIRE(  itB  != citE  );
    REQUIRE(  itB  != citEp );
    REQUIRE( citB  !=  itE  );
    REQUIRE( citB  != citE  );
    REQUIRE( citB  != citEp );
    REQUIRE( citBp !=  itE  );
    REQUIRE( citBp != citE  );
    REQUIRE( citBp != citEp );
  }
}


SCENARIO( "UnorderedSetByBlock< PointVector< 3, int64 > unit tests with 64 bits blocks", "[unorderedsetbyblock][3d]" )
{
  typedef PointVector< 3, int64_t >                Point;
  typedef std::unordered_set< Point >              StdUnorderedSet;
  typedef Splitter< Point, DGtal::uint64_t >       Splitter64;
  typedef UnorderedSetByBlock< Point, Splitter64 > BlockUnorderedSet;

  const size_t nb_inserted = 40000;
  const size_t nb_sought   = 2000;
  const size_t nb_erased   = 1000;

  StdUnorderedSet   stdSet;
  BlockUnorderedSet blkSet;
  for ( size_t i = 0; i < nb_inserted; i++ )
    {
      Point p = randomPoint<Point>( 100 );
      stdSet.insert( p );
      blkSet.insert( p );
    }
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
    for ( size_t i = 0; i < nb_sought; i++ )
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
    size_t std_nb_value_ok = 0;
    size_t blk_nb_value_ok = 0;
    for ( size_t i = 0; i < nb_sought; i++ )
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
  WHEN( "Erasing elements in identical std::unordered_set<> and UnorderedSetByBlock<>, the same elements are left" ) {
    std::vector< Point > stdErase;
    std::vector< Point > blkErase;
    size_t std_nb_erase_ok = 0;
    size_t blk_nb_erase_ok = 0;
    for ( size_t i = 0; i < nb_erased; i++ )
      {
        Point p = randomPoint<Point>( 10 );
        auto stdFindIt   = stdSet.find ( p );
        auto stdIsErased = stdSet.erase( p );
        if ( stdFindIt != stdSet.cend() )
          {
            std_nb_erase_ok  += ( stdIsErased != 0 ) ? 1 : 0;
            stdErase.push_back( p );
          }
        else std_nb_erase_ok += ( stdIsErased == 0 ) ? 1 : 0;
        auto blkFindIt   = blkSet.find ( p );
        auto blkIsErased = blkSet.erase( p );
        if ( blkFindIt != blkSet.cend() )
          {
            blk_nb_erase_ok  += ( blkIsErased != 0 ) ? 1 : 0;
            blkErase.push_back( p );
          }
        else blk_nb_erase_ok += ( blkIsErased == 0 ) ? 1 : 0;
      }
    REQUIRE( blkSet  .size() == stdSet  .size() );
    REQUIRE( blkErase.size() == stdErase.size() );
    REQUIRE( blk_nb_erase_ok == std_nb_erase_ok );
    std::vector< Point > stdTrv;
    std::vector< Point > blkTrv;
    for ( auto&& p : stdSet ) stdTrv.push_back( p );
    for ( auto&& p : blkSet ) blkTrv.push_back( p );
    REQUIRE( blkTrv.size() == stdTrv.size() );
    std::sort( stdTrv.begin(), stdTrv.end() );
    std::sort( blkTrv.begin(), blkTrv.end() );
    REQUIRE( std::equal( stdTrv.cbegin(), stdTrv.cend(), blkTrv.cbegin() ) );
  }
  WHEN( "Erasing a range in identical std::unordered_set<> and UnorderedSetByBlock<>, the same number of elements is left" ) {
    auto nb_std_before = stdSet.size();
    auto nb_blk_before = blkSet.size();
    auto stdItB = stdSet.begin(); std::advance( stdItB, 10 );
    auto blkItB = blkSet.begin(); std::advance( blkItB, 10 );
    auto stdItE = stdItB;         std::advance( stdItE, 20 );
    auto blkItE = blkItB;         std::advance( blkItE, 20 );
    stdSet.erase( stdItB, stdItE );
    blkSet.erase( blkItB, blkItE );
    size_t nb_std = std::distance(stdSet.begin(), stdSet.end());
    size_t nb_blk = std::distance(blkSet.begin(), blkSet.end());
    REQUIRE( stdSet.size() == nb_std );
    REQUIRE( stdSet.size() == nb_std_before - 20 );
    REQUIRE( blkSet.size() == nb_blk );
    REQUIRE( blkSet.size() == nb_blk_before - 20 );
    REQUIRE( blkSet.size() == stdSet.size() );
  }
  THEN( "The memory usage of UnorderedSetByBlock<> is inferior to the one of std::unordered_set<>" ) {
    const auto stdMem = blkSet.memory_usage_unordered_set();
    const auto blkMem = blkSet.memory_usage();
    REQUIRE( blkMem <= stdMem );
  }


  THEN( "Comparisons and valid assignments between iterator and const_iterator should be seamless for the user" ) {
    BlockUnorderedSet::iterator        itB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citB  = blkSet.begin();
    BlockUnorderedSet::const_iterator citBp = blkSet.cbegin();
    BlockUnorderedSet::iterator        itE  = blkSet.end();
    BlockUnorderedSet::const_iterator citE  = blkSet.end();
    BlockUnorderedSet::const_iterator citEp = blkSet.cend();
    REQUIRE(  itB  == blkSet.begin()  );
    REQUIRE(  itB  == blkSet.cbegin() );
    REQUIRE( citB  == blkSet.begin()  );
    REQUIRE( citB  == blkSet.cbegin() );
    REQUIRE( citBp == blkSet.cbegin() );
    REQUIRE(  itE  == blkSet.end()  );
    REQUIRE(  itE  == blkSet.cend() );
    REQUIRE( citE  == blkSet.end()  );
    REQUIRE( citE  == blkSet.cend() );
    REQUIRE( citEp == blkSet.cend() );
    REQUIRE(  itB  !=  itE  );
    REQUIRE(  itB  != citE  );
    REQUIRE(  itB  != citEp );
    REQUIRE( citB  !=  itE  );
    REQUIRE( citB  != citE  );
    REQUIRE( citB  != citEp );
    REQUIRE( citBp !=  itE  );
    REQUIRE( citBp != citE  );
    REQUIRE( citBp != citEp );
  }
}



//                                                                           //
///////////////////////////////////////////////////////////////////////////////
