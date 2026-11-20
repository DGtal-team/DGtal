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

#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/base/IteratorCirculatorTraits.h"
#include "DGtal/geometry/curves/SegmentComputerHelpers.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"
#include "DGtalCatch.h"

using namespace std;
using namespace DGtal;

using Sequence = std::vector<int>;
using Iterator = Sequence::const_iterator;
using Circular = DGtal::Circulator<Iterator>;

///////////////////////////////////////////////////////////////////////////////
// Predicates
///////////////////////////////////////////////////////////////////////////////

/**
 * Monotone (i.e. non-increasing or non-decreasing) sequences.
 */
struct Monotone
{
  static std::string name() { return "monotone"; }
  bool operator()(Sequence const& v) const
  {
    bool increasing = true, decreasing = true;
    for (std::size_t i = 1; i < v.size(); ++i)
    {
      increasing = increasing && (v[i-1] <= v[i]);
      decreasing = decreasing && (v[i-1] >= v[i]);
    }
    return increasing || decreasing;
  }
};

/**
 * Sequences whose amplitude (max - min) is at most K.
 */
template <int K>
struct BoundedAmplitude
{
  static std::string name() { return "amplitude <= " + std::to_string(K); }
  bool operator()(Sequence const& v) const
  {
    if (v.empty()) return true;
    auto [mi, ma] = std::minmax_element(v.begin(), v.end());
    return (*ma - *mi) <= K;
  }
};

///////////////////////////////////////////////////////////////////////////////
// Dummy segment computer
///////////////////////////////////////////////////////////////////////////////

/**
 * Segment computer whose segments are the non-empty subranges whose
 * values satisfy a hereditary predicate. Its implementation is
 * deliberately naive: the predicate is evaluated on the whole segment
 * at each call. The segment is stored as a begin iterator and a size,
 * so that a segment covering a whole circular range is well defined.
 *
 * @tparam TIterator type of iterator or circulator on integers
 * @tparam TPredicate predicate on sequences of integers
 * @tparam TCategory category of segment computer
 */
template <typename TIterator, typename TPredicate, typename TCategory>
class DummySegmentComputer
{
public:
  using ConstIterator = TIterator;
  using Self = DummySegmentComputer<TIterator, TPredicate, TCategory>;
  using Reverse = DummySegmentComputer<ReverseIterator<TIterator>, TPredicate, TCategory>;
  using SegmentComputerCategory = TCategory;

  DummySegmentComputer() : myBegin(), myEnd(), mySize(0) {}

  void init(const ConstIterator& it)
  {
    myBegin = it; myEnd = it; ++myEnd; mySize = 1;
  }

  Self getSelf() const { return Self(); }
  Reverse getReverse() const { return Reverse(); }

  ConstIterator begin() const { return myBegin; }
  ConstIterator end() const { return myEnd; }
  std::size_t size() const { return mySize; }

  bool operator==(const Self& other) const
  {
    return (myBegin == other.myBegin) && (myEnd == other.myEnd) && (mySize == other.mySize);
  }
  bool operator!=(const Self& other) const { return !(*this == other); }

  bool isExtendableFront() const
  {
    Sequence v = values(); v.push_back( *myEnd );
    return TPredicate()(v);
  }
  bool extendFront()
  {
    if ( ! isExtendableFront() ) return false;
    ++myEnd; ++mySize;
    return true;
  }

  bool isExtendableBack() const
  {
    ConstIterator it( myBegin ); --it;
    Sequence v = values(); v.insert( v.begin(), *it );
    return TPredicate()(v);
  }
  bool extendBack()
  {
    if ( ! isExtendableBack() ) return false;
    --myBegin; ++mySize;
    return true;
  }

  bool retractBack()
  {
    if ( mySize <= 1 ) return false;
    ++myBegin; --mySize;
    return true;
  }
  bool retractFront()
  {
    if ( mySize <= 1 ) return false;
    --myEnd; --mySize;
    return true;
  }

private:
  Sequence values() const
  {
    Sequence v;
    ConstIterator it( myBegin );
    for (std::size_t i = 0; i < mySize; ++i, ++it)
      v.push_back( *it );
    return v;
  }

  ConstIterator myBegin;
  ConstIterator myEnd;
  std::size_t mySize;
};

///////////////////////////////////////////////////////////////////////////////
// Brute force computation of the maximal segments
///////////////////////////////////////////////////////////////////////////////

/**
 * A segment of a sequence, given by the index of its first element
 * and its number of elements.
 */
struct Segment
{
  int begin;
  int size;
  bool operator==(const Segment& other) const = default;
};

std::ostream& operator<<(std::ostream& out, const Segment& s)
{
  return out << "[" << s.begin << ", +" << s.size << ")";
}

/**
 * Sequence of integers, either open (range) or closed (circular range),
 * with the maximal segments for a given predicate.
 */
template <typename TPredicate>
struct BruteForce
{
  Sequence values;
  bool closed;
  int n;
  std::vector<Segment> maximalSegments;  // sorted by first element
  bool wholeRange;                       // true if the whole (closed) range is a segment

  BruteForce(Sequence const& v, bool c)
    : values(v), closed(c), n( (int) v.size() ), wholeRange(false)
  {
    if ( closed && longest(0) == n )
    {
      wholeRange = true;
      maximalSegments.push_back( { 0, n } );
      return;
    }
    for (int i = 0; i < n; ++i)
    {
      // the segment starting at i is maximal iff it is not
      // included in the longest segment starting at i-1
      bool isMaximal;
      if ( i == 0 && ! closed )
        isMaximal = true;
      else
      {
        int prev = (i == 0) ? n-1 : i-1;
        int prevEnd = (i == 0) ? (prev + longest(prev) - n) : (prev + longest(prev));
        isMaximal = ( prevEnd < i + longest(i) );
      }
      if ( isMaximal )
        maximalSegments.push_back( { i, longest(i) } );
    }
  }

  /// value at a given index (modulo n if closed)
  int at(int i) const { return values[ ((i % n) + n) % n ]; }

  /// number of elements of the longest segment starting at i
  int longest(int i) const
  {
    int limit = closed ? n : n - i;
    int l = 1;
    while ( l < limit )
    {
      Sequence v;
      for (int k = 0; k <= l; ++k) v.push_back( at(i+k) );
      if ( ! TPredicate()(v) ) break;
      ++l;
    }
    return l;
  }

  /// position of i in segment s, or -1 if i is not in s
  int position(int i, Segment const& s) const
  {
    int d = closed ? ( ((i - s.begin) % n) + n ) % n : i - s.begin;
    return ( (d >= 0) && (d < s.size) ) ? d : -1;
  }

  /// index in maximalSegments of the first maximal segment containing i
  int first(int i) const
  {
    int res = -1, best = -1;
    for (int k = 0; k < (int) maximalSegments.size(); ++k)
    {
      int d = position(i, maximalSegments[k]);
      if (d > best) { best = d; res = k; }
    }
    return res;
  }

  /// index in maximalSegments of the last maximal segment containing i
  int last(int i) const
  {
    int res = -1, best = n+1;
    for (int k = 0; k < (int) maximalSegments.size(); ++k)
    {
      int d = position(i, maximalSegments[k]);
      if ( (d >= 0) && (d < best) ) { best = d; res = k; }
    }
    return res;
  }
};

///////////////////////////////////////////////////////////////////////////////
// Tools for open and closed ranges
///////////////////////////////////////////////////////////////////////////////

/**
 * Open or closed range on a sequence of integers.
 * @tparam TIterator either Iterator (open range) or Circular (closed range)
 */
template <typename TIterator>
struct Range;

template <>
struct Range<Iterator>
{
  static constexpr bool closed = false;
  Sequence const& v;
  Iterator begin() const { return v.cbegin(); }
  Iterator end() const { return v.cend(); }
  Iterator at(int i) const { return v.cbegin() + i; }
  int index(Iterator const& it) const { return (int) (it - v.cbegin()); }
};

template <>
struct Range<Circular>
{
  static constexpr bool closed = true;
  Sequence const& v;
  Circular begin() const { return Circular( v.cbegin(), v.cbegin(), v.cend() ); }
  Circular end() const { return begin(); }
  Circular at(int i) const { return Circular( v.cbegin() + i, v.cbegin(), v.cend() ); }
  int index(Circular const& it) const { return (int) (it.base() - v.cbegin()); }
};

/// Returns the segment computed by a segment computer
template <typename TSC, typename TIterator>
Segment segment(TSC const& s, Range<TIterator> const& r)
{
  return { r.index(s.begin()), (int) s.size() };
}

/// Builds a segment computer on a given segment
template <typename TSC, typename TIterator>
TSC makeSegmentComputer(Segment const& seg, Range<TIterator> const& r)
{
  TSC s;
  s.init( r.at(seg.begin) );
  for (int k = 1; k < seg.size; ++k)
  {
    bool ok = s.extendFront();
    REQUIRE( ok );
  }
  return s;
}

/// Returns a string describing a sequence, for the messages
std::string toString(Sequence const& v, bool closed)
{
  std::stringstream ss;
  ss << (closed ? "closed" : "open") << " sequence (";
  for (std::size_t i = 0; i < v.size(); ++i) ss << (i ? ", " : "") << v[i];
  ss << ")";
  return ss.str();
}

///////////////////////////////////////////////////////////////////////////////
// Checks
///////////////////////////////////////////////////////////////////////////////

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
        //a maximal segment containing i, between the first and the last ones
        Segment res = segment(s, r);
        auto it = std::find( bf.maximalSegments.begin(), bf.maximalSegments.end(), res );
        CHECK( it != bf.maximalSegments.end() );
        CHECK( bf.position(i, res) >= 0 );
        CHECK( bf.position(i, res) <= bf.position(i, bf.maximalSegments[ bf.first(i) ]) );
        CHECK( bf.position(i, res) >= bf.position(i, bf.maximalSegments[ bf.last(i) ]) );
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
    for (std::string mode : { "First", "MostCentered", "Last" })
    {
      INFO( "mode " << mode );
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

/// Sequences used in the tests
const std::vector<Sequence> sequences = {
  { 3, 4, 5, 5, 5, 4, 3 },
  { 0, 1, 0, 1, 0, 1, 0, 1 },
  { 1, 1, 2, 2, 2, 1, 1, 3, 3, 0, 0, 0, 5 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1 },
  { 3, 1, 4, 1, 5, 9, 2, 6, 5, 3, 5, 8, 9, 7, 9 },
  { 2, 2, 2, 2 },
  { 7, 8 },
  { 7 }
};

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

/** @ingroup Tests **/
