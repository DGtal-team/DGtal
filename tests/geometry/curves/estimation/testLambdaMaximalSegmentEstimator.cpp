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
 * @file
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2026/11/26
 *
 * Functions for testing class LambdaMaximalSegmentEstimator.
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "geometry/curves/SegmentComputerTestTools.h"

#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/estimation/SegmentComputerEstimators.h"
#include "DGtal/geometry/curves/estimation/LambdaMaximalSegmentEstimator.h"

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Tools
///////////////////////////////////////////////////////////////////////////////

/**
 * Value associated to an element of index @a i in a segment beginning
 * at index @a b and containing @a n elements, which identifies both the
 * segment and the element.
 */
double value(int b, int n, int i)
{
  return 100.0 * b + n + 0.01 * i;
}

/**
 * Segment computer estimator returning, for each element, the value
 * associated to the element and to the attached segment.
 */
template <typename TSC, typename TIterator>
struct ValueEstimator
{
  using SegmentComputer = TSC;
  using ConstIterator = typename TSC::ConstIterator;
  using Quantity = double;

  ValueEstimator() : myRange(nullptr) {}
  ValueEstimator(Range<TIterator> const& r) : myRange(&r) {}

  void init(const double, const ConstIterator&, const ConstIterator&) {}
  void attach(const SegmentComputer& sc) { mySC = sc; }
  bool isValid() const { return myRange != nullptr; }

  Quantity eval(const ConstIterator& it) const
  {
    return value( myRange->index( mySC.begin() ), (int) mySC.size(), myRange->index( it ) );
  }

  template <typename OutputIterator>
  OutputIterator eval(const ConstIterator& itb, const ConstIterator& ite,
                      OutputIterator result) const
  {
    if ( isNotEmpty(itb, ite) )
    {
      ConstIterator it = itb;
      do
      {
        *result++ = eval(it);
        ++it;
      } while (it != ite);
    }
    return result;
  }

  Range<TIterator> const* myRange;
  SegmentComputer mySC;
};

/**
 * Expected estimation at element @a i, computed by brute force from
 * the maximal segments passing through @a i.
 */
template <typename TPredicate, typename TLambda, typename TEccentricity>
double expectedValue(BruteForce<TPredicate> const& bf, int i)
{
  double weights = 0.0, weightedSum = 0.0, sum = 0.0;
  int count = 0;
  for (Segment const& s : bf.maximalSegments)
  {
    int idx = bf.position(i, s);
    if ( idx < 0 ) continue;
    double w = TLambda()( TEccentricity()( (std::size_t) idx, (std::size_t) s.size ) );
    double q = value( s.begin, s.size, i );
    weights += w;
    weightedSum += w * q;
    sum += q;
    ++count;
  }
  REQUIRE( count > 0 );
  return ( weights > 0.0 ) ? weightedSum / weights : sum / count;
}

/**
 * Checks the estimator on a sequence, at each element and on all the
 * subranges.
 */
template <typename TIterator, typename TPredicate, typename TCategory,
          typename TLambda, typename TEccentricity>
void checkEstimator(Sequence const& v)
{
  using SC = DummySegmentComputer<TIterator, TPredicate, TCategory>;
  using SCEstimator = ValueEstimator<SC, TIterator>;
  using Estimator = LambdaMaximalSegmentEstimator<SC, SCEstimator, TLambda, TEccentricity>;

  Range<TIterator> r{ v };
  BruteForce<TPredicate> bf( v, r.closed );
  //a segment covering the whole closed range has no well-defined begin
  if ( bf.wholeRange ) return;
  const int n = (int) v.size();
  INFO( toString(v, r.closed) << ", " << TPredicate::name() );

  Estimator e{ SC(), SCEstimator( r ) };
  e.init( r.begin(), r.end() );

  for (int i = 0; i < n; ++i)
  {
    INFO( "evaluation at " << i );
    CHECK( e.eval( r.at(i) ) == Approx( expectedValue<TPredicate, TLambda, TEccentricity>(bf, i) ) );
  }

  for (int s = 0; s < n; ++s)
    for (int t = s+1; t <= ( r.closed ? s+n : n ); ++t)
    {
      INFO( "evaluation on [" << s << ", " << t << ")" );
      const bool whole = r.closed && (t == s+n);
      std::vector<double> res;
      if ( whole )
        e.eval( r.at(s), r.at(s), BoundedBackInserter<double>{ &res, (std::size_t) 2*n } );
      else
        e.eval( r.at(s), r.at(t), BoundedBackInserter<double>{ &res, (std::size_t) 2*n } );
      REQUIRE( (int) res.size() == t-s );
      for (int i = s; i < t; ++i)
        CHECK( res[i-s] == Approx( expectedValue<TPredicate, TLambda, TEccentricity>(bf, i % n) ) );
    }
}

/// Checks the estimator on all the sequences, open and closed
template <typename TPredicate, typename TCategory, typename TLambda, typename TEccentricity>
void checkAllSequences()
{
  for (Sequence const& v : sequences)
  {
    checkEstimator<Iterator, TPredicate, TCategory, TLambda, TEccentricity>( v );
    checkEstimator<Circular, TPredicate, TCategory, TLambda, TEccentricity>( v );
  }
}

///////////////////////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////////////////////

TEST_CASE("Eccentricity and lambda functors")
{
  functors::PointEccentricity pe;
  CHECK( pe(0, 5) == 0.0 );
  CHECK( pe(2, 5) == 0.5 );
  CHECK( pe(4, 5) == 1.0 );
  CHECK( pe(0, 1) == 0.5 );
  functors::StepEccentricity se;
  CHECK( se(0, 4) == 0.125 );
  CHECK( se(3, 4) == 0.875 );
  CHECK( se(0, 1) == 0.5 );
  functors::LambdaTriangleFunction triangle;
  CHECK( triangle(0.0) == 0.0 );
  CHECK( triangle(0.25) == 0.5 );
  CHECK( triangle(0.5) == 1.0 );
  CHECK( triangle(1.0) == 0.0 );
}

TEMPLATE_TEST_CASE("LambdaMaximalSegmentEstimator on sequences", "",
                   ForwardSegmentComputer, BidirectionalSegmentComputer,
                   DynamicSegmentComputer, DynamicBidirectionalSegmentComputer)
{
  SECTION("Monotone subsequences, lambda64, points")
  {
    checkAllSequences<Monotone, TestType, functors::Lambda64Function, functors::PointEccentricity>();
  }
  SECTION("Monotone subsequences, triangle, steps")
  {
    checkAllSequences<Monotone, TestType, functors::LambdaTriangleFunction, functors::StepEccentricity>();
  }
  SECTION("Subsequences of amplitude at most 1, lambda64, points")
  {
    checkAllSequences<BoundedAmplitude<1>, TestType, functors::Lambda64Function, functors::PointEccentricity>();
  }
  SECTION("Subsequences of amplitude at most 3, triangle, steps")
  {
    checkAllSequences<BoundedAmplitude<3>, TestType, functors::LambdaTriangleFunction, functors::StepEccentricity>();
  }
}

TEST_CASE("LambdaMaximalSegmentEstimator on a digital curve")
{
  using Point = Z2i::Point;
  std::fstream fst( (testPath + "samples/contourS.fc").c_str(), std::ios::in );
  FreemanChain<int> fc( fst );
  std::vector<Point> points( fc.begin(), fc.end() );
  points.pop_back(); // the last point is equal to the first one
  using ConstCirculator = DGtal::Circulator<std::vector<Point>::const_iterator>;
  ConstCirculator c( points.cbegin(), points.cbegin(), points.cend() );

  using SegmentComputer = ArithmeticalDSSComputer<ConstCirculator, int, 4>;
  using SCEstimator = TangentFromDSSEstimator<SegmentComputer>;
  using Estimator = LambdaMaximalSegmentEstimator<SegmentComputer, SCEstimator>;

  Estimator e{ SegmentComputer(), SCEstimator() };
  e.init( c, c );
  std::vector<SCEstimator::Quantity> tangents;
  e.eval( c, c, std::back_inserter( tangents ) );
  REQUIRE( tangents.size() == points.size() );

  ConstCirculator it( c );
  for (std::size_t i = 0; i < points.size(); ++i, ++it)
  {
    INFO( "point " << i );
    auto t = e.eval( it );
    //weighted mean of unit vectors: a vector of norm at most 1
    CHECK( t.norm() <= 1.0 + 1e-9 );
    CHECK( t.norm() > 0.5 );
    CHECK( t[0] == Approx( tangents[i][0] ) );
    CHECK( t[1] == Approx( tangents[i][1] ) );
  }
}

/** @ingroup Tests **/
