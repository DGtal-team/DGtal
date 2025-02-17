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
 * @file testHyperRectDomain.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * @author Guillaume Damiand
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testHyperRectDomain <p>
 * Aim: simple test of \ref HyperRectDomain
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <iterator>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/base/CConstBidirectionalRange.h"

#include "DGtalCatch.h"

using namespace DGtal;
using namespace std;


///////////////////////////////////////////////////////////////////////////////
// Simple test of HyperRectDomain construction.

TEST_CASE( "Simple HyperRectDomain", "[domain][4D]" )
{
  typedef SpaceND<4> Space4Type;
  typedef Space4Type::Point Point;
  typedef Space4Type::RealPoint RealPoint;

  Space4Type::Integer t [] = { 1, 2, 3 , 4};
  Point a ( t );
  Space4Type::Integer t2[] = { 5, 5, 3 , 4};
  Point b ( t2 );
  double td [] = { 1.1, 2.5, 3 , 4};
  RealPoint c ( td );
  double td2[] = { 4.9, 4.5, 3 , 4};
  RealPoint d ( td2 );

  trace.beginBlock ( "HyperRectDomain init" );

  // Checking that HyperRectDomain is a model of CDomain.
  typedef HyperRectDomain<Space4Type> HRDomain4;
  BOOST_CONCEPT_ASSERT(( concepts::CDomain< HRDomain4 > ));
  DGTAL_CONCEPT_CHECK(requires concepts::CConstBidirectionalRange<HRDomain4> );

  // Empty domain using the default constructor
  trace.info() << "Empty domain using the default constructor" << std::endl;
  HyperRectDomain<Space4Type> myEmptyDomain;

  trace.info() << "Empty domain = " << myEmptyDomain << std::endl;
  REQUIRE( myEmptyDomain.isValid() );
  REQUIRE( myEmptyDomain.isEmpty() );
  REQUIRE( myEmptyDomain.size() == 0 );
  REQUIRE( (myEmptyDomain.end() - myEmptyDomain.begin()) == 0 );
  REQUIRE( (myEmptyDomain.begin() - myEmptyDomain.end()) == 0 );
  REQUIRE( (myEmptyDomain.rend() - myEmptyDomain.rbegin()) == 0 );
  REQUIRE( (myEmptyDomain.rbegin() - myEmptyDomain.rend()) == 0 );

  // Domain characterized by points a and b
  trace.info() << "Domain characterized by points a and b" << std::endl;
  HyperRectDomain<Space4Type> myHyperRectDomain ( a, b );

  trace.info() << "Domain = " << myHyperRectDomain << std::endl;
  REQUIRE( myHyperRectDomain.isValid() );
  REQUIRE( myHyperRectDomain.lowerBound() == a );
  REQUIRE( myHyperRectDomain.upperBound() == b );

  trace.info() << "Domain size = " << myHyperRectDomain.size() << std::endl;
  REQUIRE( myHyperRectDomain.size() == 20 );
  REQUIRE( (myHyperRectDomain.end() - myHyperRectDomain.begin()) == 20 );
  REQUIRE( (myHyperRectDomain.begin() - myHyperRectDomain.end()) == -20 );
  REQUIRE( (myHyperRectDomain.rend() - myHyperRectDomain.rbegin()) == 20 );
  REQUIRE( (myHyperRectDomain.rbegin() - myHyperRectDomain.rend()) == -20 );

  // Domain initialized with RealPoint
  trace.info() << "Domain initialized with RealPoint" << std::endl;
  HyperRectDomain<Space4Type> myHyperRectDomain_rr ( c, d );
  trace.info() << "Domain = " << myHyperRectDomain_rr << std::endl;
  REQUIRE( myHyperRectDomain_rr.isValid() );
  REQUIRE( myHyperRectDomain_rr.lowerBound() == myHyperRectDomain.lowerBound() );
  REQUIRE( myHyperRectDomain_rr.upperBound() == myHyperRectDomain.upperBound() );

  HyperRectDomain<Space4Type> myHyperRectDomain_ir ( a, d );
  trace.info() << "Domain = " << myHyperRectDomain_ir << std::endl;
  REQUIRE( myHyperRectDomain_ir.isValid() );
  REQUIRE( myHyperRectDomain_ir.lowerBound() == myHyperRectDomain.lowerBound() );
  REQUIRE( myHyperRectDomain_ir.upperBound() == myHyperRectDomain.upperBound() );

  HyperRectDomain<Space4Type> myHyperRectDomain_ri ( c, b );
  trace.info() << "Domain = " << myHyperRectDomain_ri << std::endl;
  REQUIRE( myHyperRectDomain_ri.isValid() );
  REQUIRE( myHyperRectDomain_ri.lowerBound() == myHyperRectDomain.lowerBound() );
  REQUIRE( myHyperRectDomain_ri.upperBound() == myHyperRectDomain.upperBound() );

  trace.endBlock();

  // Test Copy Constructor
  trace.beginBlock("Test Copy Constructor");
  HyperRectDomain<Space4Type> myHyperRectDomainBis( myHyperRectDomain );
  trace.info() << "Domain = " << myHyperRectDomainBis << std::endl;
  trace.info() << "Domain size = " << myHyperRectDomainBis.size() << std::endl;
  REQUIRE( myHyperRectDomainBis.isValid() );
  REQUIRE( myHyperRectDomainBis.lowerBound() == myHyperRectDomain.lowerBound() );
  REQUIRE( myHyperRectDomainBis.upperBound() == myHyperRectDomain.upperBound() );
  REQUIRE( myHyperRectDomainBis.size() == 20 );
  trace.endBlock();

  // Test Assignement
  trace.beginBlock("Test Assignement");
  HyperRectDomain<Space4Type> myHyperRectDomainTer;
  myHyperRectDomainTer = myHyperRectDomain;
  trace.info() << "Domain = " << myHyperRectDomainTer << std::endl;
  trace.info() << "Domain size = " << myHyperRectDomainTer.size() << std::endl;
  REQUIRE( myHyperRectDomainTer.isValid() );
  REQUIRE( myHyperRectDomainTer.lowerBound() == myHyperRectDomain.lowerBound() );
  REQUIRE( myHyperRectDomainTer.upperBound() == myHyperRectDomain.upperBound() );
  REQUIRE( myHyperRectDomainTer.size() == 20 );
  trace.endBlock();
}

///////////////////////////////////////////////////////////////////////////////
// Checking iterators

/// Checking iterator given then span domain and dimensions order.
template <
  typename Iterator,
  typename Point,
  typename Domain,
  typename Dimension
>
void testIteratorHelperImpl(
    Iterator & it, Iterator const& it_begin, Iterator const& it_end,
    typename std::iterator_traits<Iterator>::difference_type & cnt_begin,
    Point & pt, Domain const& domain, std::vector<Dimension> const& dimensions, std::size_t id,
    bool forward)
{
  if (id == 0)
    {
      trace.warning() << *it << std::endl;
      REQUIRE( it != it_end );
      REQUIRE( pt == *it );
      REQUIRE( std::distance(it_begin, it) ==  cnt_begin );
      REQUIRE( std::distance(it, it_begin) == -cnt_begin );
      INFO( *(it_begin + cnt_begin) << " == " << *it );
      REQUIRE( it_begin + cnt_begin == it );
      INFO( *(it - cnt_begin) << " == " << *it_begin);
      REQUIRE( it - cnt_begin == it_begin );
      ++it;
      ++cnt_begin;
      return;
    }

  --id;

  const auto d = dimensions[id];

  if (forward)
    {
      while ( pt[d] <= domain.upperBound()[d] )
        {
          testIteratorHelperImpl(it, it_begin, it_end, cnt_begin, pt, domain, dimensions, id, forward);
          ++pt[d];
        }

      pt[d] = domain.lowerBound()[d];
    }
  else
    {
      while ( pt[d] >= domain.lowerBound()[d] )
        {
          testIteratorHelperImpl(it, it_begin, it_end, cnt_begin, pt, domain, dimensions, id, forward);
          --pt[d];
        }

      pt[d] = domain.upperBound()[d];
    }
}

template <
  typename Iterator,
  typename Point,
  typename Domain,
  typename Dimension
>
void testIteratorHelper(
    Iterator const& it_begin, Iterator const& it_end,
    Point pt, Domain const& domain, std::vector<Dimension> const& dimensions,
    bool forward = true)
{
  Iterator it = it_begin;
  typename std::iterator_traits<Iterator>::difference_type cnt_begin = 0;

  testIteratorHelperImpl(
    it, it_begin, it_end, cnt_begin,
    pt, domain, dimensions, dimensions.size(),
    forward);
  REQUIRE( it == it_end );
}

template <
  typename Iterator,
  typename Point,
  typename Domain
>
void testIteratorHelper(
    Iterator const& it_begin, Iterator const& it_end,
    Point pt, Domain const& domain, bool forward = true)
{
  std::vector<Dimension> dimensions(Domain::dimension);
  std::iota(dimensions.begin(), dimensions.end(), Dimension(0));

  testIteratorHelper(it_begin, it_end, pt, domain, dimensions, forward);
}

template <
  typename Point
>
void testIterator(Point const& a, Point const& b, Point const& c)
{
  using Space = SpaceND<Point::dimension, typename Point::Component>;
  using Domain = HyperRectDomain<Space>;

  Domain domain(a, b);
  trace.info() << "Domain = " << domain << std::endl;

  trace.emphase() << "Iterator" << std::endl;
  testIteratorHelper(domain.begin(), domain.end(), domain.lowerBound(), domain);

  trace.emphase() << "Reverse iterator" << std::endl;
  testIteratorHelper(domain.rbegin(), domain.rend(), domain.upperBound(), domain, false);

  trace.emphase() << "Iterator from starting point" << std::endl;
  testIteratorHelper(domain.begin(c), domain.end(), c, domain);

  trace.emphase() << "Reverse iterator from starting point" << std::endl;
  testIteratorHelper(domain.rbegin(c), domain.rend(), c, domain, false);

  trace.emphase() << "Iterator on reversed dimension order: " << std::endl;
  std::vector<Dimension> dimensions(Point::dimension);
  std::iota(dimensions.rbegin(), dimensions.rend(), Dimension(0));
  const auto range = domain.subRange(dimensions);
  testIteratorHelper(range.begin(), range.end(), domain.lowerBound(), domain, dimensions);

  trace.emphase() << "Reverse iterator on reversed dimension order: " << std::endl;
  testIteratorHelper(range.rbegin(), range.rend(), domain.upperBound(), domain, dimensions, false);

  trace.emphase() << "Iterator on reversed dimension order and from a starting point: " << std::endl;
  const auto range2 = domain.subRange(dimensions);
  testIteratorHelper(range2.begin(c), range2.end(), c, domain, dimensions);

  trace.emphase() << "Reverse iterator on reversed dimension order and from a starting point: " << std::endl;
  testIteratorHelper(range2.rbegin(c), range2.rend(), c, domain, dimensions, false);

  trace.emphase() << "Iterator along one dimension: " << std::endl;
  const auto range3 = domain.subRange({1});
  const std::vector<Dimension> one_dimension({1});
  testIteratorHelper(range3.begin(), range3.end(), domain.lowerBound(), domain, one_dimension);

  trace.emphase() << "Reverse iterator along one dimension: " << std::endl;
  auto upper_one_dim = domain.lowerBound();
  upper_one_dim.partialCopy(domain.upperBound(), one_dimension);
  testIteratorHelper(range3.rbegin(), range3.rend(), upper_one_dim, domain, one_dimension, false);

  trace.emphase() << "Iterator along one dimension and from a starting point: " << std::endl;
  const auto range4 = domain.subRange({1}, c);
  testIteratorHelper(range4.begin(c), range4.end(), c, domain, one_dimension);

  trace.emphase() << "Reverse iterator along one dimension and from a starting point: " << std::endl;
  testIteratorHelper(range4.rbegin(c), range4.rend(), c, domain, one_dimension, false);
}

TEST_CASE( "Iterator 2D", "[iterator][2D]" )
{
  using Space = SpaceND<2>;
  using Point = Space::Point;

  Point a (1, 1);
  Point b (4, 5);
  Point c (2, 2);

  trace.beginBlock( "Iterator 2D" );
  testIterator(a, b, c);
  trace.endBlock();
}

TEST_CASE( "Iterator 4D", "[iterator][4D]" )
{
  using Space = SpaceND<4>;
  using Point = Space::Point;

  Point a({1, 1, 1, 1});
  Point b({2, 3, 4, 5});
  Point c({1, 2, 3, 2});

  trace.beginBlock( "Iterator 4D" );
  testIterator(a, b, c);
  trace.endBlock();
}

#ifdef WITH_BIGINTEGER
TEST_CASE( "Iterator 4D GMP", "[iterator][4D][GMP]" )
{
  using Space = SpaceND<4, BigInteger>;
  using Point = Space::Point;

  Point a({1, 1, 1, 1});
  Point b({2, 3, 4, 5});
  Point c({1, 2, 3, 2});

  trace.beginBlock( "Iterator 4D using GMP" );
  testIterator(a, b, c);
  trace.endBlock();
}
#endif

TEST_CASE( "STL compatiblity", "[iterator][4D][STL]" )
{
  typedef SpaceND<4> TSpace4D;
  typedef TSpace4D::Point Point4D;
  TSpace4D::Integer t[] = {1, 1, 1, 1};
  Point4D a4D (t);
  TSpace4D::Integer t2[] = {3, 3, 3, 3};
  Point4D b4D (t2);

  trace.beginBlock ( "TestSTL Compatibility" );

  HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D, b4D );
  trace.emphase() << myHyperRectDomain4D << std::endl;

  std::copy ( myHyperRectDomain4D.begin(),
      myHyperRectDomain4D.end(),
      ostream_iterator<Point4D> ( trace.info(), " " ) );

  trace.info() << std::endl;
  trace.endBlock();
}

TEST_CASE( "Empty domain", "[domain][3D][empty]" )
{
  typedef SpaceND<3> TSpace;
  typedef TSpace::Point TPoint;
  typedef HyperRectDomain<TSpace> TDomain;

  const TDomain nonempty_domain( TPoint::diagonal(0), TPoint::diagonal(0) );
  REQUIRE( !nonempty_domain.isEmpty() );

  const TDomain default_domain;
  REQUIRE( default_domain.isEmpty() );

  const TDomain domain( TPoint::diagonal(1), TPoint::diagonal(0) );
  REQUIRE( domain.isEmpty() );
  REQUIRE( domain.size() == 0 );
  REQUIRE( domain.begin() == domain.end() );
  REQUIRE( domain.rbegin() == domain.rend() );

  TDomain::ConstSubRange range = domain.subRange( 0, 1, 2, domain.lowerBound()  );
  REQUIRE( range.begin() == range.end() );
  REQUIRE( range.rbegin() == range.rend() );

  range = domain.subRange( 2, 1, 0, domain.lowerBound()  );
  REQUIRE( range.begin() == range.end() );
  REQUIRE( range.rbegin() == range.rend() );

  range = domain.subRange( 0, 2, domain.lowerBound()  );
  REQUIRE( range.begin() == range.end() );
  REQUIRE( range.rbegin() == range.rend() );

  range = domain.subRange( 2, 0, domain.lowerBound()  );
  REQUIRE( range.begin() == range.end() );
  REQUIRE( range.rbegin() == range.rend() );

  range = domain.subRange( 1, domain.lowerBound()  );
  REQUIRE( range.begin() == range.end() );
  REQUIRE( range.rbegin() == range.rend() );
}

/** @ingroup Tests **/
