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
 * @date 2026/11/20
 *
 * Unit tests of the functions of SegmentComputerHelpers.h.
 *
 * The functions are tested on sequences of integers with a dummy segment
 * computer, whose segments are the subsequences satisfying a simple
 * hereditary predicate (e.g. monotone subsequences). The expected results
 * are computed by brute force.
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdexcept>

#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/base/IteratorCirculatorTraits.h"
#include "DGtal/geometry/curves/SegmentComputerHelpers.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"
#include "DGtal/geometry/curves/GreedySegmentation.h"
#include "DGtal/geometry/curves/estimation/MostCenteredMaximalSegmentEstimator.h"
#include "DGtalCatch.h"
#include "geometry/curves/SegmentComputerTestTools.h"

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Checks
///////////////////////////////////////////////////////////////////////////////

/**
 * Modes of SaturatedSegmentation, with their names, which are used
 * by the brute-force computations (the three simple modes first).
 */
template <typename TSC>
std::vector<std::pair<std::string, typename SaturatedSegmentation<TSC>::Mode> >
saturatedModes()
{
  using Mode = typename SaturatedSegmentation<TSC>::Mode;
  return { { "First", Mode::First }, { "MostCentered", Mode::MostCentered },
           { "Last", Mode::Last }, { "First++", Mode::FirstPlusPlus },
           { "MostCentered++", Mode::MostCenteredPlusPlus },
           { "Last++", Mode::LastPlusPlus } };
}

/**
 * Modes of GreedySegmentation, with their names, which are used
 * by the brute-force computations.
 */
template <typename TSC>
std::vector<std::pair<std::string, typename GreedySegmentation<TSC>::Mode> >
greedyModes()
{
  using Mode = typename GreedySegmentation<TSC>::Mode;
  return { { "Truncate", Mode::Truncate }, { "Truncate+1", Mode::TruncatePlusOne },
           { "DoNotTruncate", Mode::DoNotTruncate } };
}


/**
 * Checks all the functions of SegmentComputerHelpers.h on a sequence.
 */
template <typename TIterator, typename TPredicate, typename TCategory>
void checkFunctions(Sequence const& v)
{
  using SC = DummySegmentComputer<TIterator, TPredicate, TCategory>;
  Range<TIterator> r{ v };
  BruteForce<TPredicate> bf( v, r.closed );
  const int n = (int) v.size();
  const int m = (int) bf.maximalSegments.size();
  INFO( toString(v, r.closed) << ", " << TPredicate::name() );

  //a segment covering the whole closed range has no well-defined begin
  auto check = [&](SC const& s, Segment const& expected)
  {
    if ( bf.wholeRange )
      CHECK( (int) s.size() == n );
    else
      CHECK( segment(s, r) == expected );
  };

  {
    INFO( "longestSegment" );
    for (int i = 0; i < n; ++i)
    {
      INFO( "i = " << i );
      SC s;
      longestSegment( s, r.at(i), r.end() );
      check( s, { i, bf.longest(i) } );
    }
  }

  {
    INFO( "firstMaximalSegment" );
    for (int i = 0; i < n; ++i)
    {
      INFO( "i = " << i );
      SC s;
      firstMaximalSegment( s, r.at(i), r.begin(), r.end() );
      check( s, bf.maximalSegments[ bf.first(i) ] );
    }
  }

  {
    INFO( "lastMaximalSegment" );
    for (int i = 0; i < n; ++i)
    {
      INFO( "i = " << i );
      SC s;
      lastMaximalSegment( s, r.at(i), r.begin(), r.end() );
      check( s, bf.maximalSegments[ bf.last(i) ] );
    }
  }

  {
    INFO( "mostCenteredMaximalSegment" );
    for (int i = 0; i < n; ++i)
    {
      INFO( "i = " << i );
      SC s;
      mostCenteredMaximalSegment( s, r.at(i), r.begin(), r.end() );
      if ( bf.wholeRange )
        CHECK( (int) s.size() == n );
      else
      {
        //the maximal segment containing the tangent at i
        //(definition 3 of Feschet and Tougne, 1999)
        CHECK( segment(s, r) == bf.maximalSegments[ bf.mostCentered(i) ] );
      }
    }
  }

  if ( bf.wholeRange ) return;

  {
    INFO( "nextMaximalSegment" );
    for (int k = 0; k < m; ++k)
    {
      if ( ! r.closed && k+1 == m ) break;  //no next segment
      INFO( "maximal segment " << bf.maximalSegments[k] );
      SC s = makeSegmentComputer<SC>( bf.maximalSegments[k], r );
      nextMaximalSegment( s, r.end() );
      check( s, bf.maximalSegments[ (k+1) % m ] );
    }
  }

  {
    INFO( "previousMaximalSegment" );
    for (int k = 0; k < m; ++k)
    {
      if ( ! r.closed && k == 0 ) continue;  //no previous segment
      INFO( "maximal segment " << bf.maximalSegments[k] );
      SC s = makeSegmentComputer<SC>( bf.maximalSegments[k], r );
      previousMaximalSegment( s, r.begin() );
      check( s, bf.maximalSegments[ (k+m-1) % m ] );
    }
  }

  {
    INFO( "SaturatedSegmentation" );
    for (auto const& [name, mode] : saturatedModes<SC>())
    {
      if ( name.find("++") != std::string::npos ) continue;
      INFO( "mode " << name );
      SaturatedSegmentation<SC> segmentation( r.begin(), r.end(), SC() );
      segmentation.setMode(mode);
      std::vector<Segment> res;
      for (auto it = segmentation.begin();
           (it != segmentation.end()) && ((int) res.size() <= m); ++it)
        res.push_back( segment(*it, r) );
      if ( r.closed )
      { //the first segment of a closed range is not necessarily the first one
        auto it = std::find( res.begin(), res.end(), bf.maximalSegments[0] );
        if ( it != res.end() )
          std::rotate( res.begin(), it, res.end() );
      }
      CHECK( res == bf.maximalSegments );
    }
  }
}

/**
 * Checks GreedySegmentation, SaturatedSegmentation (on subranges, in all
 * modes) and MostCenteredMaximalSegmentEstimator on a sequence.
 */
template <typename TIterator, typename TPredicate, typename TCategory>
void checkSegmentations(Sequence const& v)
{
  using SC = DummySegmentComputer<TIterator, TPredicate, TCategory>;
  Range<TIterator> r{ v };
  BruteForce<TPredicate> bf( v, r.closed );
  const int n = (int) v.size();
  const int m = (int) bf.maximalSegments.size();
  INFO( toString(v, r.closed) << ", " << TPredicate::name() );
  if ( bf.wholeRange ) return;

  //subranges [s, t): all of them for an open range, those of
  //length 1 to n (whole range) for a closed range
  std::vector<std::pair<int,int> > subranges;
  for (int s = 0; s < n; ++s)
    for (int t = s+1; t <= ( r.closed ? s+n : n ); ++t)
      subranges.push_back( { s, t } );

  auto segments = [&](auto const& segmentation)
  {
    std::vector<Segment> res;
    for (auto it = segmentation.begin();
         (it != segmentation.end()) && ((int) res.size() <= 2*n+2); ++it)
      res.push_back( segment(*it, r) );
    return res;
  };

  {
    INFO( "SaturatedSegmentation" );
    for (auto [s, t] : subranges)
      for (auto const& [name, mode] : saturatedModes<SC>())
      {
        const bool whole = ( r.closed ? (t == s+n) : ( (s == 0) && (t == n) ) );
        //the "++" modes are not tested for a whole closed range
        if ( ( name.find("++") != std::string::npos ) && r.closed && whole )
          continue;
        INFO( "subrange [" << s << ", " << t << "), mode " << name
              << ( bf.sameChosenSegment(s, t, name) ? " (same chosen segment)" : "" ) );
        SaturatedSegmentation<SC> segmentation( r.begin(), r.end(), SC() );
        if ( ! whole || r.closed ) segmentation.setSubRange( r.at(s), r.at(t) );
        segmentation.setMode(mode);
        CHECK( segments(segmentation) == bf.saturated(s, t, name) );
      }
  }

  {
    INFO( "GreedySegmentation" );
    for (auto [s, t] : subranges)
      for (auto const& [name, mode] : greedyModes<SC>())
      {
        const bool whole = ( r.closed ? (t == s+n) : ( (s == 0) && (t == n) ) );
        //no extension beyond the whole closed range
        if ( r.closed && whole && (name != "Truncate") ) continue;
        INFO( "subrange [" << s << ", " << t << "), mode " << name );
        GreedySegmentation<SC> segmentation( r.begin(), r.end(), SC() );
        if ( ! whole || r.closed ) segmentation.setSubRange( r.at(s), r.at(t) );
        segmentation.setMode(mode);
        CHECK( segments(segmentation) == bf.greedy(s, t, name) );
      }
  }

  {
    INFO( "MostCenteredMaximalSegmentEstimator" );
    using Estimator = MostCenteredMaximalSegmentEstimator<SC, SegmentEstimator<SC, TIterator> >;
    auto expected = [&](int i) { return bf.maximalSegments[ bf.mostCentered(i) ]; };
    Estimator e{ SC(), SegmentEstimator<SC, TIterator>( r ) };
    e.init( r.begin(), r.end() );

    //evaluation at each element
    for (int i = 0; i < n; ++i)
    {
      INFO( "evaluation at " << i );
      CHECK( e.eval( r.at(i) ) == expected(i) );
    }

    //evaluation on subranges
    for (auto [s, t] : subranges)
    {
      INFO( "evaluation on [" << s << ", " << t << ")" );
      const bool whole = ( r.closed ? (t == s+n) : ( (s == 0) && (t == n) ) );
      std::vector<Segment> res, exp;
      if ( whole && r.closed )
      { //the whole closed range can only be given by r.begin(), r.end()
        if ( s != 0 ) continue;
        CHECK_NOTHROW( e.eval( r.begin(), r.end(), BoundedBackInserter<Segment>{ &res, (std::size_t) 2*n } ) );
      }
      else
        CHECK_NOTHROW( e.eval( r.at(s), r.at(t), BoundedBackInserter<Segment>{ &res, (std::size_t) 2*n } ) );
      for (int i = s; i < t; ++i)
        exp.push_back( expected( ((i % n) + n) % n ) );
      CHECK( res == exp );
    }
  }
  (void) m;
}


/// Checks the functions on all the sequences, open and closed
template <typename TPredicate, typename TCategory>
void checkAllSequences()
{
  for (Sequence const& v : sequences)
  {
    checkFunctions<Iterator, TPredicate, TCategory>( v );
    checkFunctions<Circular, TPredicate, TCategory>( v );
  }
}

/// Checks the segmentations and the estimator on all the sequences, open and closed
template <typename TPredicate, typename TCategory>
void checkSegmentationsOnAllSequences()
{
  for (Sequence const& v : sequences)
  {
    checkSegmentations<Iterator, TPredicate, TCategory>( v );
    checkSegmentations<Circular, TPredicate, TCategory>( v );
  }
}

///////////////////////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////////////////////

TEST_CASE("Brute force computation of maximal segments")
{
  SECTION("Monotone subsequences")
  {
    BruteForce<Monotone> bf( { 3, 4, 5, 5, 5, 4, 3 }, false );
    REQUIRE( bf.maximalSegments == std::vector<Segment>{ { 0, 5 }, { 2, 5 } } );
    BruteForce<Monotone> bfc( { 3, 4, 5, 5, 5, 4, 3 }, true );
    REQUIRE( bfc.maximalSegments == std::vector<Segment>{ { 2, 6 }, { 6, 6 } } );
  }
  SECTION("Subsequences of bounded amplitude")
  {
    //each element belongs to (at most) K+1 maximal segments
    BruteForce<BoundedAmplitude<2> > bf( { 0, 1, 2, 3, 4, 5 }, false );
    REQUIRE( bf.maximalSegments == std::vector<Segment>{ { 0, 3 }, { 1, 3 }, { 2, 3 }, { 3, 3 } } );
    REQUIRE( bf.first(3) == 1 );
    REQUIRE( bf.last(3) == 3 );
  }
  SECTION("Tangent (Feschet and Tougne, 1999)")
  {
    BruteForce<Monotone> bf( { 1, 1, 2, 2, 2, 1, 1, 3, 3, 0, 0, 0, 5 }, false );
    //maximal segments: [0,5), [2,7), [5,9), [7,12), [9,13)
    REQUIRE( bf.symmetricTangent(6) == Segment{ 5, 3 } );
    REQUIRE( bf.maximalSegments[ bf.mostCentered(6) ] == Segment{ 5, 4 } );
    REQUIRE( bf.symmetricTangent(3) == Segment{ 2, 3 } );
    REQUIRE( bf.maximalSegments[ bf.mostCentered(3) ] == Segment{ 2, 5 } );
    REQUIRE( bf.symmetricTangent(0) == Segment{ 0, 1 } );
    REQUIRE( bf.maximalSegments[ bf.mostCentered(0) ] == Segment{ 0, 5 } );
  }
  SECTION("Whole closed range")
  {
    BruteForce<Monotone> bf( { 2, 2, 2 }, true );
    REQUIRE( bf.wholeRange );
  }
}

TEMPLATE_TEST_CASE("Functions of SegmentComputerHelpers.h", "",
                   ForwardSegmentComputer, BidirectionalSegmentComputer,
                   DynamicSegmentComputer, DynamicBidirectionalSegmentComputer)
{
  SECTION("Monotone subsequences")
  {
    checkAllSequences<Monotone, TestType>();
  }
  SECTION("Subsequences of constant values")
  {
    checkAllSequences<BoundedAmplitude<0>, TestType>();
  }
  SECTION("Subsequences of amplitude at most 1")
  {
    checkAllSequences<BoundedAmplitude<1>, TestType>();
  }
  SECTION("Subsequences of amplitude at most 3")
  {
    checkAllSequences<BoundedAmplitude<3>, TestType>();
  }
}

TEMPLATE_TEST_CASE("Segmentations and most centered maximal segment estimator", "",
                   ForwardSegmentComputer, BidirectionalSegmentComputer,
                   DynamicSegmentComputer, DynamicBidirectionalSegmentComputer)
{
  SECTION("Monotone subsequences")
  {
    checkSegmentationsOnAllSequences<Monotone, TestType>();
  }
  SECTION("Subsequences of constant values")
  {
    checkSegmentationsOnAllSequences<BoundedAmplitude<0>, TestType>();
  }
  SECTION("Subsequences of amplitude at most 1")
  {
    checkSegmentationsOnAllSequences<BoundedAmplitude<1>, TestType>();
  }
  SECTION("Subsequences of amplitude at most 3")
  {
    checkSegmentationsOnAllSequences<BoundedAmplitude<3>, TestType>();
  }
}

TEST_CASE("Modes of the segmentations given as strings")
{
  using SC = DummySegmentComputer<Iterator, BoundedAmplitude<1>, DynamicBidirectionalSegmentComputer>;
  const Sequence v = sequences[3];
  Range<Iterator> r{ v };
  const int n = (int) v.size();
  auto segments = [&](auto const& segmentation)
  {
    std::vector<Segment> res;
    for (auto it = segmentation.begin(); it != segmentation.end(); ++it)
      res.push_back( segment(*it, r) );
    return res;
  };

  SECTION("SaturatedSegmentation")
  {
    for (auto const& [name, mode] : saturatedModes<SC>())
    {
      INFO( "mode " << name );
      SaturatedSegmentation<SC> s1( r.begin(), r.end(), SC() ), s2( r.begin(), r.end(), SC() );
      s1.setSubRange( r.at(2), r.at(n-3) );
      s2.setSubRange( r.at(2), r.at(n-3) );
      s1.setMode(mode);
      s2.setMode(name);
      CHECK( segments(s1) == segments(s2) );
    }
    SaturatedSegmentation<SC> segmentation( r.begin(), r.end(), SC() );
    CHECK_THROWS_AS( segmentation.setMode("Unknown"), InputException );
    CHECK_THROWS_AS( segmentation.setMode("Truncate"), InputException );
  }

  SECTION("GreedySegmentation")
  {
    for (auto const& [name, mode] : greedyModes<SC>())
    {
      INFO( "mode " << name );
      GreedySegmentation<SC> s1( r.begin(), r.end(), SC() ), s2( r.begin(), r.end(), SC() );
      s1.setSubRange( r.at(2), r.at(n-3) );
      s2.setSubRange( r.at(2), r.at(n-3) );
      s1.setMode(mode);
      s2.setMode(name);
      CHECK( segments(s1) == segments(s2) );
    }
    GreedySegmentation<SC> segmentation( r.begin(), r.end(), SC() );
    CHECK_THROWS_AS( segmentation.setMode("Unknown"), InputException );
    CHECK_THROWS_AS( segmentation.setMode("First"), InputException );
  }
}

/** @ingroup Tests **/
