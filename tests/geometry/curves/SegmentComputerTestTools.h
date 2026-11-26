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

#pragma once

/**
 * @file
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2026/11/26
 *
 * Tools for testing segment computers, segmentations and estimators on
 * sequences of integers: a dummy segment computer, whose segments are the
 * subsequences satisfying a simple hereditary predicate (e.g. monotone
 * subsequences), brute-force computations of the maximal segments, and
 * a segment computer estimator returning the segments.
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
#include "DGtalCatch.h"

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
  bool operator<(const Segment& other) const
  { return (begin < other.begin) || ( (begin == other.begin) && (size < other.size) ); }
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

  /// true if the l elements starting at index b form a segment
  bool isSegment(int b, int l) const
  {
    Sequence v;
    for (int k = 0; k < l; ++k) v.push_back( at(b+k) );
    return TPredicate()(v);
  }

  /**
   * Symmetric tangent at i (Definition 2 of Feschet and Tougne, 1999):
   * the segment [i-k, i+k] such that [i-k-1, i+k+1] is not a segment
   * (or exceeds the range).
   */
  Segment symmetricTangent(int i) const
  {
    int k = 0;
    while ( ( closed ? (2*k+3 <= n) : ( (i-k-1 >= 0) && (i+k+1 < n) ) )
            && isSegment(i-k-1, 2*k+3) )
      ++k;
    return { i-k, 2*k+1 };
  }

  /**
   * Index in maximalSegments of the maximal segment containing the
   * tangent at i (Definition 3 of Feschet and Tougne, 1999): the symmetric
   * tangent at i is first extended forward as much as possible, then
   * backward as much as possible.
   */
  int mostCentered(int i) const
  {
    Segment t = symmetricTangent(i);
    int b = t.begin, e = t.begin + t.size; //[b,e)
    while ( ( closed ? (e-b < n) : (e < n) ) && isSegment(b, e-b+1) ) ++e;
    while ( ( closed ? (e-b < n) : (b > 0) ) && isSegment(b-1, e-b+1) ) --b;
    Segment res{ closed ? ( (b % n) + n ) % n : b, e-b };
    for (int k = 0; k < (int) maximalSegments.size(); ++k)
      if ( maximalSegments[k] == res ) return k;
    return -1;
  }

  /// index in maximalSegments of the maximal segment chosen for i in a given mode
  int chosen(int i, std::string const& mode) const
  {
    if ( mode.rfind("First", 0) == 0 ) return first(i);
    if ( mode.rfind("Last", 0) == 0 ) return last(i);
    return mostCentered(i);
  }

  /**
   * True if the maximal segment chosen for s is also chosen for t
   * and contains [s, t].
   */
  bool sameChosenSegment(int s, int t, std::string const& mode) const
  {
    int k = chosen(s, mode);
    return ( chosen(t % n, mode) == k )
      && ( position(t % n, maximalSegments[k]) > position(s, maximalSegments[k]) );
  }

  /**
   * Expected saturated segmentation of the subrange [s, t) in a given mode.
   * For a closed range, t may be greater than n (t = s+n for the whole range).
   * The first maximal segment is the one chosen for s; the last one is the
   * one chosen for t (included in the "++" modes only), or the one chosen
   * for the last element of an open range ending at n. If the same maximal
   * segment is chosen for s and t (and contains [s, t]), the segmentation
   * is empty in the simple modes, and reduced to this segment in the "++"
   * modes.
   */
  std::vector<Segment> saturated(int s, int t, std::string const& mode) const
  {
    const int m = (int) maximalSegments.size();
    const bool plus = ( mode.find("++") != std::string::npos );
    int kFirst = chosen(s, mode);
    int kLast;
    if ( ! closed && t == n )
      kLast = chosen(n-1, mode);
    else
      kLast = plus ? chosen(t, mode) : chosen(t, mode) - 1;
    //if the same maximal segment is chosen for s and t,
    //it is the only one of the segmentation
    const bool wholeClosed = closed && (t == s+n);
    const bool endOfOpen = ( ! closed ) && (t == n);
    std::vector<Segment> res;
    //a closed range going round: the last maximal segment, equal to
    //the first one, is retrieved again after a whole turn
    const bool turn = closed && plus && ! wholeClosed
      && ( chosen(t % n, mode) == kFirst ) && ! sameChosenSegment(s, t, mode);
    if ( ! wholeClosed && ! endOfOpen && sameChosenSegment(s, t, mode) )
    { //this maximal segment is both the first one and the (excluded) last one
      if ( plus ) res.push_back( maximalSegments[kFirst] );
      return res;
    }
    if ( closed )
    {
      kLast = ( kLast + m ) % m;
      for (int k = kFirst; ; k = (k+1) % m)
      {
        res.push_back( maximalSegments[k] );
        if ( (k == kLast) && ( ! turn || res.size() > 1 ) ) break;
      }
    }
    else
      for (int k = kFirst; k <= kLast; ++k)
        res.push_back( maximalSegments[k] );
    return res;
  }

  /**
   * Expected greedy segmentation of the subrange [s, t) in a given mode
   * ("Truncate", "Truncate+1" or "DoNotTruncate"). For a closed range,
   * t may be greater than n (t = s+n for the whole range).
   * Each segment is the longest segment starting at a given element,
   * bounded by t; the next segment starts at the last element of the
   * previous one if these two elements form a segment, at the following
   * element otherwise.
   */
  std::vector<Segment> greedy(int s, int t, std::string const& mode) const
  {
    std::vector<Segment> res;
    int pos = s;
    while ( true )
    {
      int e = pos+1;
      while ( (e < t) && isSegment(pos, e-pos+1) ) ++e;
      if ( e == t )
      { //last segment
        if ( closed || (t < n) )
        {
          if ( mode == "Truncate+1" )
          {
            if ( isSegment(pos, e-pos+1) ) ++e;
          }
          else if ( mode == "DoNotTruncate" )
          { //extension until the end of the (closed) range
            while ( isSegment(pos, e-pos+1) )
            {
              ++e;
              if ( closed ? (e % n == 0) : (e == n) ) break;
            }
          }
        }
        res.push_back( { closed ? pos % n : pos, e-pos } );
        break;
      }
      res.push_back( { closed ? pos % n : pos, e-pos } );
      pos = isSegment(e-1, 2) ? e-1 : e;
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
  Circular at(int i) const
  {
    const int n = (int) v.size();
    return Circular( v.cbegin() + ( ((i % n) + n) % n ), v.cbegin(), v.cend() );
  }
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

/**
 * Segment computer estimator that returns, for each element, the
 * segment to which it is attached, as a pair (index of the first
 * element, number of elements).
 */
template <typename TSC, typename TIterator>
struct SegmentEstimator
{
  using SegmentComputer = TSC;
  using ConstIterator = typename TSC::ConstIterator;
  using Quantity = Segment;

  SegmentEstimator() : myRange(nullptr) {}
  SegmentEstimator(Range<TIterator> const& r) : myRange(&r) {}

  void init(const double, const ConstIterator&, const ConstIterator&) {}
  void attach(const SegmentComputer& sc) { mySC = sc; }
  bool isValid() const { return myRange != nullptr; }

  Quantity eval(const ConstIterator&) const
  {
    return Segment{ myRange->index( mySC.begin() ), (int) mySC.size() };
  }

  template <typename OutputIterator>
  OutputIterator eval(const ConstIterator& itb, const ConstIterator& ite,
                      OutputIterator result) const
  {
    // do-while loop to deal with the case of a whole circular range,
    // as in the estimators of SegmentComputerEstimators.h
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
 * Output iterator appending values to a vector, which throws an
 * exception when more than a given number of values are written,
 * so that an estimator that does not stop is detected.
 */
template <typename T>
struct BoundedBackInserter
{
  using iterator_category = std::output_iterator_tag;
  using value_type = void;
  using difference_type = std::ptrdiff_t;
  using pointer = void;
  using reference = void;

  std::vector<T>* v;
  std::size_t max;
  BoundedBackInserter& operator=(const T& x)
  {
    if ( v->size() >= max ) throw std::runtime_error("too many values");
    v->push_back(x);
    return *this;
  }
  BoundedBackInserter& operator*() { return *this; }
  BoundedBackInserter& operator++() { return *this; }
  BoundedBackInserter operator++(int) { return *this; }
};

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
