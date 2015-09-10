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
 * @file testLinearizer.cpp
 * @ingroup Tests
 * @author Roland Denis (\c roland.denis@univ-smb.fr )
 *
 *
 * @date 2015/09/10
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testLinearizer.cpp <p>
 * Aim: Tests of linearization (point to index) et reverse process (index to point) for HyperRectDomain.
 */

#include <cstddef>
#include <cmath>

#include "DGtalCatch.h"

#include <DGtal/kernel/SpaceND.h>
#include <DGtal/kernel/domains/HyperRectDomain.h>
#include <DGtal/kernel/domains/Linearizer.h>

using namespace DGtal;

// Previous version of the linearizer, used here for reference results.
namespace
{

  /**
   * Class template for linearization of the coordinates of a Point.
   * This class template is to be specialized for efficiency for dimensions 1,
   * 2 and 3 to prevent the use of a loop in these cases.
   *
   * @tparam Domain an instance of HyperRectDomain
   * @tparam dimension domain dimension
   */
  template < typename Domain, int dimension>
  struct linearizer
  {

    typedef typename Domain::Point Point;
    typedef typename Domain::Size Size;

    /**
     * Compute the linearized offset of a point in a vector container.
     *
     * @param aPoint a point
     * @param lowerBound lower bound of the image domain.
     * @param extent extension of the image domain.
     *
     * @return the index
     */
    static Size apply( const Point & aPoint, const Point & lowerBound,
        const Point & extent )
    {
      Size pos = aPoint[ 0 ] - lowerBound[ 0 ] ;
      Size multiplier = 1;
      for (typename Domain::Dimension k = 1 ; k < dimension ; ++k)
      {
        multiplier *= extent[ k-1  ];
        pos += multiplier * ( aPoint[ k ] - lowerBound[ k ] );
      }
      return pos;
    }
  };

  /**
   * Specialization of the linearizer class for dimension 1.
   *
   */
  template < typename Domain >
  struct linearizer< Domain, 1 >
  {
    typedef typename Domain::Point Point;
    typedef typename Domain::Size Size;

    static Size apply( const Point & aPoint,
        const Point & lowerBound,
        const Point & /*extent*/ )
    {
      return aPoint[ 0 ] - lowerBound[ 0 ];
    }
  };

  /**
   * Specialization of the linearizer class for dimension 2.
   *
   */
  template < typename Domain >
  struct linearizer< Domain, 2 >
  {
    typedef typename Domain::Point Point;
    typedef typename Domain::Size Size;

    static Size apply( const Point & aPoint,
        const Point & lowerBound,
        const Point & extent )
    {
      return ( aPoint[ 0 ] - lowerBound[ 0 ] ) + extent[ 0 ] *
	(aPoint[ 1 ] - lowerBound[ 1 ] );
    }
  };

  /**
   * Specialization of the linearizer class for dimension 3.
   *
   */
  template < typename Domain >
  struct linearizer< Domain, 3 >
  {
    typedef typename Domain::Point Point;
    typedef typename Domain::Size Size;

    static Size apply( const Point & aPoint,
        const Point & lowerBound,
        const Point & extent )
    {
      Size res = aPoint[ 0 ] - lowerBound[ 0 ];
      Size multiplier = extent[ 0 ];
      res += multiplier * ( aPoint[ 1 ] - lowerBound[ 1 ] );
      multiplier *= extent[ 1 ];
      res += multiplier * ( aPoint[ 2 ] - lowerBound[ 2 ] );
      return res;
    }
  };
}

// Test for Linearizer with dimension parameter
template < DGtal::Dimension N >
class LinearizerTester
{
public:
  typedef SpaceND<N>              Space;
  typedef HyperRectDomain<Space>  Domain;
  typedef typename Space::Point   Point;

  typedef linearizer<Domain, N>   RefLinearizer;
  typedef Linearizer<Domain, ColMajorStorage>   NewLinearizer;

private:
  Domain myDomain;

public:
  /// Construct a test domain whose size is near the given size.
  LinearizerTester( std::size_t size = 1e5 )
    {
      Point lowerBound;
      for ( std::size_t i = 0 ; i < N ; ++i )
        lowerBound[i] = 1 + 7*i;

      std::size_t dim_size = std::size_t( std::pow( double(size), 1./N ) + 0.5 );
      Point upperBound;
      for ( std::size_t i = 0; i < N ; ++i )
        upperBound[i] = lowerBound[i] + dim_size + i;

      myDomain = Domain( lowerBound, upperBound );
    }

  /// Test getIndex( Point, Point, Extent ) syntax
  bool testGetIndexFromPPE()
    {
      const Point lowerBound = myDomain.lowerBound();
      const Point extent = myDomain.upperBound() - lowerBound + Point::diagonal(1);

      bool success = true;
      for ( typename Domain::ConstIterator it = myDomain.begin(), it_end = myDomain.end(); it != it_end && success ; ++it )
        {
          if ( RefLinearizer::apply( *it, lowerBound, extent ) != NewLinearizer::getIndex( *it, lowerBound, extent ) )
            {
              FAIL( "Index is different for " << *it );
              success = false;
            }
        }

      return success;
    }

  /// Test getIndex( Point, Extent ) syntax
  bool testGetIndexFromPE()
    {
      const Point lowerBound = myDomain.lowerBound();
      const Point extent = myDomain.upperBound() - lowerBound + Point::diagonal(1);

      bool success = true;
      for ( typename Domain::ConstIterator it = myDomain.begin(), it_end = myDomain.end(); it != it_end && success ; ++it )
        {
          if ( RefLinearizer::apply( *it, lowerBound, extent ) != NewLinearizer::getIndex( *it - lowerBound, extent ) )
            {
              FAIL( "Index is different for " << *it );
              success = false;
            }
        }

      return success;
    }

  /// Test getIndex( Point, Domain ) syntax
  bool testGetIndexFromPD()
    {
      const Point lowerBound = myDomain.lowerBound();
      const Point extent = myDomain.upperBound() - lowerBound + Point::diagonal(1);

      bool success = true;
      for ( typename Domain::ConstIterator it = myDomain.begin(), it_end = myDomain.end(); it != it_end && success ; ++it )
        {
          if ( RefLinearizer::apply( *it, lowerBound, extent ) != NewLinearizer::getIndex( *it, myDomain ) )
            {
              FAIL( "Index is different for " << *it );
              success = false;
            }
        }

      return success;
    }

  /// Test getPoint( Size, Point, Extent ) syntax
  bool testGetPointFromSPE()
    {
      const Point lowerBound = myDomain.lowerBound();
      const Point extent = myDomain.upperBound() - lowerBound + Point::diagonal(1);
      return true;

    }
};

/// Converter between col-major and row-major storage order.
template < typename StorageOrder >
struct PointConverter;

template <>
struct PointConverter<ColMajorStorage>
{
  template < typename TPoint >
  static inline
  TPoint apply( TPoint const& aPoint )
    {
      return aPoint;
    }
};

template <>
struct PointConverter<RowMajorStorage>
{
  template < typename TPoint >
  static inline
  TPoint apply( TPoint const& aPoint )
    {
      TPoint result;
      for ( std::size_t i = 0 ; i < TPoint::dimension ; ++i )
        result[i] = aPoint[ TPoint::dimension - i - 1 ];

      return result;
    }
};

#define TEST_LINEARIZER( N , ORDER ) \
TEST_CASE( "Testing Linearizer in dimension " #N " with " #ORDER, "[test][dim" #N "][" #ORDER "]" )\
{\
\
  typedef SpaceND<N>              Space;\
  typedef HyperRectDomain<Space>  Domain;\
  typedef typename Space::Point   Point;\
\
  typedef linearizer<Domain, N>   RefLinearizer;\
  typedef Linearizer<Domain, ORDER>   NewLinearizer;\
\
  typedef PointConverter<ORDER>   RefConverter;\
\
  std::size_t size = 1e3;\
\
  Point lowerBound;\
  for ( std::size_t i = 0 ; i < N ; ++i )\
    lowerBound[i] = 1 + 7*i;\
\
  std::size_t dim_size = std::size_t( std::pow( double(size), 1./N ) + 0.5 );\
  Point upperBound;\
  for ( std::size_t i = 0; i < N ; ++i )\
    upperBound[i] = lowerBound[i] + dim_size + i;\
\
  Domain domain( lowerBound, upperBound );\
  Point extent = upperBound - lowerBound + Point::diagonal(1);\
\
  Point refLowerBound = RefConverter::apply(lowerBound);\
  Point refExtent     = RefConverter::apply(extent);\
\
  SECTION( "Testing getIndex(Point, Point, Extent) syntax" )\
    {\
      for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
          REQUIRE( RefLinearizer::apply( RefConverter::apply(*it), refLowerBound, refExtent ) == NewLinearizer::getIndex( *it, lowerBound, extent ) );\
    }\
\
  SECTION( "Testing getIndex(Point, Extent) syntax" )\
    {\
      for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
          REQUIRE( RefLinearizer::apply( RefConverter::apply(*it), refLowerBound, refExtent ) == NewLinearizer::getIndex( *it - lowerBound, extent ) );\
    }\
\
  SECTION( "Testing getIndex(Point, Domain) syntax" )\
    {\
      for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
          REQUIRE( RefLinearizer::apply( RefConverter::apply(*it), refLowerBound, refExtent ) == NewLinearizer::getIndex( *it, domain ) );\
    }\
\
  SECTION( "Testing getPoint(Index, Point, Extent) syntax" )\
    {\
      for ( std::size_t i = 0; i < domain.size(); ++i )\
          REQUIRE( RefLinearizer::apply( RefConverter::apply( NewLinearizer::getPoint( i, lowerBound, extent ) ), refLowerBound, refExtent ) == i );\
    }\
\
  SECTION( "Testing getPoint(Index, Extent) syntax" )\
    {\
      for ( std::size_t i = 0; i < domain.size(); ++i )\
          REQUIRE( RefLinearizer::apply( RefConverter::apply( NewLinearizer::getPoint( i, extent ) + lowerBound ), refLowerBound, refExtent ) == i );\
    }\
\
  SECTION( "Testing getPoint(Index, Domain) syntax" )\
    {\
      for ( std::size_t i = 0; i < domain.size(); ++i )\
          REQUIRE( RefLinearizer::apply( RefConverter::apply( NewLinearizer::getPoint( i, domain ) ), refLowerBound, refExtent ) == i );\
    }\
}

#define BENCH_LINEARIZER( N , ORDER ) \
TEST_CASE( "Benchmarking Linearizer in dimension " #N " with " #ORDER, "[.bench][dim" #N "][" #ORDER "]" )\
{\
\
  typedef SpaceND<N>              Space;\
  typedef HyperRectDomain<Space>  Domain;\
  typedef typename Space::Point   Point;\
\
  typedef linearizer<Domain, N>   RefLinearizer;\
  typedef Linearizer<Domain, ORDER>   NewLinearizer;\
\
  typedef PointConverter<ORDER>   RefConverter;\
\
  std::size_t size = 1e8;\
\
  Point lowerBound;\
  for ( std::size_t i = 0 ; i < N ; ++i )\
    lowerBound[i] = 1 + 7*i;\
\
  std::size_t dim_size = std::size_t( std::pow( double(size), 1./N ) + 0.5 );\
  Point upperBound;\
  for ( std::size_t i = 0; i < N ; ++i )\
    upperBound[i] = lowerBound[i] + dim_size + i;\
\
  Domain domain( lowerBound, upperBound );\
  Point extent = upperBound - lowerBound + Point::diagonal(1);\
\
  Point refLowerBound = RefConverter::apply(lowerBound);\
  Point refExtent     = RefConverter::apply(extent);\
\
  std::size_t sum = 0;\
\
  for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
      sum += RefLinearizer::apply( RefConverter::apply(*it), refLowerBound, refExtent );\
  REQUIRE( sum > 0 );\
  sum = 0;\
\
  SECTION( "Benchmarking reference linearizer" )\
    {\
      for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
          sum += RefLinearizer::apply( RefConverter::apply(*it), refLowerBound, refExtent );\
    }\
\
  SECTION( "Benchmarking getIndex(Point, Point, Extent) syntax" )\
    {\
      for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
          sum += NewLinearizer::getIndex( *it, lowerBound, extent );\
    }\
\
  SECTION( "Benchmarking getIndex(Point, Extent) syntax" )\
    {\
      for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
          sum += NewLinearizer::getIndex( *it, extent );\
    }\
\
  SECTION( "Benchmarking getIndex(Point, Domain) syntax" )\
    {\
      for ( typename Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end ; ++it )\
          sum += NewLinearizer::getIndex( *it, domain );\
    }\
\
  SECTION( "Benchmarking getPoint(Index, Point, Extent) syntax" )\
    {\
      for ( std::size_t i = 0; i < domain.size(); ++i )\
          sum += NewLinearizer::getPoint( i, lowerBound, extent )[N-1];\
    }\
\
  SECTION( "Benchmarking getPoint(Index, Extent) syntax" )\
    {\
      for ( std::size_t i = 0; i < domain.size(); ++i )\
          sum += NewLinearizer::getPoint( i, extent )[N-1];\
    }\
\
  SECTION( "Benchmarking getPoint(Index, Domain) syntax" )\
    {\
      for ( std::size_t i = 0; i < domain.size(); ++i )\
          sum += NewLinearizer::getPoint( i, domain )[N-1];\
    }\
\
  REQUIRE( sum > 0 );\
}

TEST_LINEARIZER( 1, ColMajorStorage )
TEST_LINEARIZER( 2, ColMajorStorage )
TEST_LINEARIZER( 3, ColMajorStorage )
TEST_LINEARIZER( 4, ColMajorStorage )
TEST_LINEARIZER( 5, ColMajorStorage )

TEST_LINEARIZER( 1, RowMajorStorage )
TEST_LINEARIZER( 2, RowMajorStorage )
TEST_LINEARIZER( 3, RowMajorStorage )
TEST_LINEARIZER( 4, RowMajorStorage )
TEST_LINEARIZER( 5, RowMajorStorage )

BENCH_LINEARIZER( 1, ColMajorStorage )
BENCH_LINEARIZER( 2, ColMajorStorage )
BENCH_LINEARIZER( 3, ColMajorStorage )
BENCH_LINEARIZER( 4, ColMajorStorage )
BENCH_LINEARIZER( 5, ColMajorStorage )

BENCH_LINEARIZER( 1, RowMajorStorage )
BENCH_LINEARIZER( 2, RowMajorStorage )
BENCH_LINEARIZER( 3, RowMajorStorage )
BENCH_LINEARIZER( 4, RowMajorStorage )
BENCH_LINEARIZER( 5, RowMajorStorage )
