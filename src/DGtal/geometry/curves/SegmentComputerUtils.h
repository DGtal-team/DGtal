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
 * @file SegmentComputerUtils.h
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/07/25
 *
 * Header file for module SegmentComputerUtils.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SegmentComputerUtils_RECURSES)
#error Recursive header files inclusion detected in SegmentComputerUtils.h
#else // defined(SegmentComputerUtils_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SegmentComputerUtils_RECURSES

#if !defined SegmentComputerUtils_h
/** Prevents repeated inclusion of headers. */
#define SegmentComputerUtils_h


#include <type_traits>
#include "DGtal/base/Circulator.h"

namespace DGtal
{

////////////////////////////////////////////
// Categories
/**
 * Tags describing the capabilities of a segment computer that the
 * functions of this file may exploit. They form a hierarchy, like
 * standard iterator tags: a tag is convertible to any tag requiring
 * fewer capabilities.
 *
 * - ForwardSegmentComputer: model of CForwardSegmentComputer
 * - BidirectionalSegmentComputer: model of CBidirectionalSegmentComputer
 * - DynamicSegmentComputer: model of CDynamicSegmentComputer
 * - DynamicBidirectionalSegmentComputer: model of CDynamicBidirectionalSegmentComputer
 */
   struct ForwardSegmentComputer{};
   struct BidirectionalSegmentComputer : virtual ForwardSegmentComputer {};
   struct DynamicSegmentComputer : virtual ForwardSegmentComputer {};
   struct DynamicBidirectionalSegmentComputer
     : BidirectionalSegmentComputer, DynamicSegmentComputer {};

/**
 * True if a segment computer whose capabilities are described by
 * @a TMaxCategory may be used as a segment computer of category
 * @a TCategory, i.e. if @a TCategory does not require more capabilities
 * than @a TMaxCategory.
 *
 * Segment computers that can be used with several categories take the
 * chosen category as a template parameter and check it with this
 * constant, e.g.:
 * @code
 * static_assert( isSegmentComputerCategoryCompatible<TCategory, DynamicBidirectionalSegmentComputer>,
 *                "TCategory exceeds the capabilities of this segment computer" );
 * @endcode
 *
 * @tparam TCategory the chosen category
 * @tparam TMaxCategory the category describing all the capabilities
 */
template <typename TCategory, typename TMaxCategory>
inline constexpr bool isSegmentComputerCategoryCompatible =
  std::is_base_of_v<ForwardSegmentComputer, TCategory> &&
  std::is_base_of_v<TCategory, TMaxCategory>;


/////////////////////////////////////////////////////////////////////////////
/**
 * Description of template class 'SegmentComputerTraits' <p>
 * \brief Aim:
 *  Provides the category of the segment computer
 * {ForwardSegmentComputer,BidirectionalSegmentComputer,
 * DynamicSegmentComputer, DynamicBidirectionalSegmentComputer},
 * which is used by the functions of this file to select the
 * most suitable algorithm.
 *
 * The category is the nested type `SC::SegmentComputerCategory`
 * if it exists, ForwardSegmentComputer otherwise. It may also be
 * set by specializing this class.
 *
 * @tparam SC any segment computer
 */

//default
template <typename SC>
struct SegmentComputerTraits {
    typedef  ForwardSegmentComputer Category;
};

//segment computers declaring their category
template <typename SC>
  requires requires { typename SC::SegmentComputerCategory; }
struct SegmentComputerTraits<SC> {
    typedef typename SC::SegmentComputerCategory Category;
};



////////////////////////////////////////////
// Useful functions for segment computers


/**
 * Specialization for random access category
 */
template<typename IC>
IC getMiddleIterator(const IC& itb, const IC& ite, RandomAccessCategory)
{
//how to compute this with circulators ?
//return itb + ((ite-itb)/2);
//does not work
  return getMiddleIterator(itb, ite, BidirectionalCategory() );
}

/**
 * Specialization for bidirectional category
 * NB: in O(ite-itb)
 */
template<typename IC>
IC getMiddleIterator(const IC& itb, const IC& ite, BidirectionalCategory)
{
  IC b( itb );
  IC f( ite );
  bool flag = true;
  while (b != f) {
    if (flag) {
      --f;
      flag = false;
    } else {
      ++b;
      flag = true;
    }
  }
  return b;
}

/**
 * Specialization for forward category
 * NB: in O(ite-itb)
 */
template<typename IC>
IC getMiddleIterator(const IC& itb, const IC& ite, ForwardCategory)
{
  IC i( itb );

  unsigned int c = 0;
  while (i != ite) {
    ++i;
    ++c;
  }
  unsigned int k = c/2;

  c = 0;
  i = itb;
  while (c != k) {
    ++i;
    ++c;
  }

  return i;
}
/**
 * Computes the middle iterator of a given range,
 * i.e. itb + (ite-itb)/2)
 * @param itb begin iterator of a range
 * @param ite end iterator of a range
 * @return the middle iterator of the range [itb,ite)
 * @tparam IC iterator or circulator
 */
template<typename IC>
IC getMiddleIterator(const IC& itb, const IC& ite) {
  typedef typename IteratorCirculatorTraits<IC>::Category Category;
  return getMiddleIterator(itb, ite, Category() );
}

///////////////////////////////////////////////////////////
/////////////////////////////////////// extension functions


/**
 * Specialization for Iterator type
 */
template <typename SC>
void maximalExtension(SC& s, const typename SC::ConstIterator& end, IteratorType ) {
  //stop if s.end() == end
  while ( (s.end() != end)
       && (s.extendFront()) ) {}
}

/**
 * Specialization for Circulator type
 */
template <typename SC>
void maximalExtension(SC& s, const typename SC::ConstIterator& /*end*/, CirculatorType )
{
  //stop if the segment is the whole range
  const typename SC::ConstIterator newEnd( s.begin() );
  while ( (s.extendFront())
    && (s.end() != newEnd) ) {}
}
/**
 * Calls s.extendFront() while possible
 * @param s any instance of segment computer
 * @param end any ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void maximalExtension(SC& s, const typename SC::ConstIterator& end) {
  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type;
  maximalExtension( s, end, Type() );
}


/**
 * Specialization for Iterator type
 */
template <typename SC>
void oppositeEndMaximalExtension(SC& s, const typename SC::ConstIterator& begin, IteratorType ) {
  //extend one more time if s.begin() == begin
  while ( (s.begin() != begin)
       && (s.extendBack()) ) {}
  if (s.begin() == begin) s.extendBack();
}

/**
 * Specialization for Circulator type
 */
template <typename SC>
void oppositeEndMaximalExtension(SC& s, const typename SC::ConstIterator& begin, CirculatorType )
{
  boost::ignore_unused_variable_warning( begin );
  //stop if the segment is the whole range
  const typename SC::ConstIterator newBegin( s.end() );
  while ( (s.extendBack())
       && (s.begin() != newBegin) ) {}
}

/**
 * Calls s.extendBack() while possible
 * @param s any instance of (bidirectional) segment computer
 * @param begin any ConstIterator
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void oppositeEndMaximalExtension(SC& s, const typename SC::ConstIterator& begin) {
  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type;
  oppositeEndMaximalExtension( s, begin, Type() );
}



/**
 * Specialization for Iterator type
 */
template <typename SC>
bool maximalSymmetricExtension(SC& s,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  IteratorType ) {

  bool flagOk = true;
  bool flagForward = true;
  //while the extension is possible
  //at the front and (then) at the back
  while (flagOk)  {
    if (flagForward) {
      flagForward = false;
      if ( s.end() != end ) flagOk = s.extendFront();
      else flagOk = false;
    } else {
      flagForward = true;
      if ( s.begin() != begin ) flagOk = s.extendBack();
      else flagOk = false;
    }
  }
  //extend one more time if s.begin() == begin
  if (s.begin() != begin ) {
    if (s.extendBack()) return !s.extendFront();
    else return false;
  } else {
    return !flagForward;
  }

}

/**
 * Specialization for Circulator type
 */
template <typename SC>
bool maximalSymmetricExtension(SC& s,
                               const typename SC::ConstIterator& begin,
                               const typename SC::ConstIterator& end,
                               CirculatorType )
{
  boost::ignore_unused_variable_warning( begin );
  boost::ignore_unused_variable_warning( end );

  bool flagOk = true;
  bool flagForward = true;
  //while the extensions are possible and
  //the segment does not correspond to the whole range
  while ( (flagOk) && ( s.end() != s.begin() ) )  {
    if (flagForward) {
      flagForward = false;
      flagOk = s.extendFront();
    } else {
      flagForward = true;
      flagOk = s.extendBack();
    }
  }
  return !flagForward;
}

/**
 * Calls alternatively s.extendFront() and s.extendBack()
 * while it is possible
 * @param s any instance of (bidirectional)segment computer
 * @param begin begin iterator of a range
 * @param end end iterator of a range
 * @return 'true' if the extension at the front fails first
 * and 'false' if the extension at the back fails first
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
bool maximalSymmetricExtension(SC& s,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end) {

  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type;
  return maximalSymmetricExtension( s, begin, end, Type() );

}


/**
 * Calls s.retractBack() while s.isExtendableFront()
 * returns false
 * @param s any instance of segment computer
 * @param end any ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void maximalRetraction(SC& s, const typename SC::ConstIterator& end)
{
  if ( isNotEmpty<typename SC::ConstIterator>(s.end(),end) ) {
    while ( (! s.isExtendableFront() )
          &&(s.retractBack() ) )  {}
  } else {
    while ( s.retractBack() ) {}
  }
}

/**
 * Calls s.retractFront()
 * while s.isExtendableBack() returns false
 * @param s any instance of segment computer
 * @param begin any ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void oppositeEndMaximalRetraction(SC& s, const typename SC::ConstIterator& begin)
{
  if ( isNotEmpty<typename SC::ConstIterator>(s.begin(),begin) ) {
    while ( (! s.isExtendableBack() )
          &&(s.retractFront() ) ) {}
  } else {
    while ( s.retractFront() ) {}
  }
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////// longest segment


/**
 * Specialization for Iterator type
 */
template <typename SC>
void longestSegment(SC& s,
                   const typename SC::ConstIterator& i,
                   const typename SC::ConstIterator& end,
                   IteratorType )
 {
  if (i != end) {
    s.init(i);
    maximalExtension(s, end, IteratorType() );
  }
}

/**
 * Specialization for Circulator type
 */
template <typename SC>
void longestSegment(SC& s,
                   const typename SC::ConstIterator& i,
                   const typename SC::ConstIterator& end,
                   CirculatorType )
{
  s.init(i);
  maximalExtension(s, end, CirculatorType() );
}

/**
 * Computes the longest possible segment from [i]
 * @param s any instance of segment computer
 * @param i a given ConstIterator
 * @param end any end ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void longestSegment(SC& s,
                   const typename SC::ConstIterator& i,
                   const typename SC::ConstIterator& end)
{
  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type;
  longestSegment( s, i, end, Type() );
}


////////////////////////////////////////////////////////////////
////////////////////////////////////////// first maximal segment

/**
 * Computes the first maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CForwardSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::ForwardSegmentComputer )
{

  typedef typename SC::ConstIterator ConstIterator;
  typedef typename SC::Reverse ReverseSegmentComputer;
  typedef typename ReverseSegmentComputer::ConstIterator ConstReverseIterator;

  if ( isNotEmpty<ConstIterator>(i,end) ) {

    //backward extension
    ConstIterator it( i ); ++it;
    ConstReverseIterator rit( it );
    ConstReverseIterator rend( begin );
    ReverseSegmentComputer r( s.getReverse() );
    longestSegment(r, rit, rend);

    //forward extension
    ConstIterator it2( r.end().base() );
    longestSegment(s, it2, end);

  }

}

/**
 * Computes the first maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::BidirectionalSegmentComputer)
{
  s.init(i);

  oppositeEndMaximalExtension(s, begin);
  maximalExtension(s, end);
}

/**
 * Computes the first maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CDynamicSegmentComputer
 * @note calls the function dedicated to ForwardSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::DynamicSegmentComputer)
{
  firstMaximalSegment(s,i,begin,end,DGtal::ForwardSegmentComputer() );
}

/**
 * Computes the first maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to BidirectionalSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::DynamicBidirectionalSegmentComputer)
{
  firstMaximalSegment(s,i,begin,end,DGtal::BidirectionalSegmentComputer() );
}

/**
 * Computes the first maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of segment computer
 */
template <typename SC>
void firstMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end )
{
  firstMaximalSegment<SC>(s, i, begin, end,
typename DGtal::SegmentComputerTraits<SC>::Category() );
}

////////////////////////////////////////////////////////////////
/////////////////////////////////// most centered maximal segment

/**
 * Computes the most centered maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CForwardSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::ForwardSegmentComputer )
{

  typedef typename SC::ConstIterator ConstIterator;
  typedef typename SC::Reverse ReverseSegmentComputer;
  typedef typename ReverseSegmentComputer::ConstIterator ConstReverseIterator;

 //get the first maximal segment passing through i

  firstMaximalSegment( s, i, begin, end, DGtal::ForwardSegmentComputer() );

 //get the next maximal segment while i is not at the middle of
 //the current maximal segment.

  ConstIterator k( s.begin() );
  while ( k != i ) {

    if ( isNotEmpty<ConstIterator>(s.end(),end) ) {

      //backward extension
      ConstIterator it( s.end() ); ++it;
      ConstReverseIterator rit( it );
      ConstReverseIterator rend( s.begin() );
      ReverseSegmentComputer r( s.getReverse() );
      longestSegment(r, rit, rend);
      ConstIterator newBegin = r.end().base();
      ASSERT( newBegin != s.begin() );

      while ( ( k != getMiddleIterator(newBegin, s.end() ) )
            &&( k != i ) ) {
        ++k;
      }
      if ( k != i ) {

        //get the next maximal segment
        longestSegment(s, newBegin, end);

      }

    } else {
      k = i;
    }
  }
}

/**
 * Computes the most centered maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::BidirectionalSegmentComputer)
{

  if ( (isNotEmpty(i,end)) || (isNotEmpty(i,begin)) ) {

    s.init(i);

    //symmetric extension
    if ( (isNotEmpty(i,end)) && (isNotEmpty(i,begin)) ) {
      maximalSymmetricExtension(s, begin, end);
    }

  //forward extension
  maximalExtension(s, end);

  //backward extension
  oppositeEndMaximalExtension(s, begin);

  }

}

/**
 * Computes the most centered maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CDynamicSegmentComputer
 * @note calls the function dedicated to ForwardSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::DynamicSegmentComputer)
{
  mostCenteredMaximalSegment(s,i,begin,end,DGtal::ForwardSegmentComputer() );
}

/**
 * Computes the most centered maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to BidirectionalSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::DynamicBidirectionalSegmentComputer)
{
  mostCenteredMaximalSegment(s,i,begin,end,DGtal::BidirectionalSegmentComputer() );
}

/**
 * Computes the most centered maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of segment computer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end )
{
   mostCenteredMaximalSegment<SC>(s, i, begin, end,
typename DGtal::SegmentComputerTraits<SC>::Category() );
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////// last maximal segment

/**
 * Computes the last maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CForwardSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::ForwardSegmentComputer )
{

  typedef typename SC::ConstIterator ConstIterator;
  typedef typename SC::Reverse ReverseSegmentComputer;
  typedef typename ReverseSegmentComputer::ConstIterator ConstReverseIterator;

  //forward extension
  ConstIterator j( i );
  longestSegment(s, j, end);

  //backward extension
  ConstIterator it( s.end() );
  ConstReverseIterator rit( it );
  ConstReverseIterator rend( begin );
  ReverseSegmentComputer r( s.getReverse() );
  longestSegment(r, rit, rend);

  //forward extension
  ConstIterator it2( r.end().base() );
  longestSegment(s, it2, end);
}

/**
 * Computes the last maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  BidirectionalSegmentComputer)
{
  s.init(i);

  maximalExtension(s, end);
  oppositeEndMaximalExtension(s, begin);
}

/**
 * Computes the last maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CDynamicSegmentComputer
 * @note calls the function dedicated to ForwardSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::DynamicSegmentComputer)
{
  lastMaximalSegment(s,i,begin,end,DGtal::ForwardSegmentComputer() );
}

/**
 * Computes the last maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to BidirectionalSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  DGtal::DynamicBidirectionalSegmentComputer)
{
  lastMaximalSegment(s,i,begin,end,DGtal::BidirectionalSegmentComputer() );
}

/**
 * Computes the last maximal segment passing through i
 * @param s any instance of segment computer
 * @param i any ConstIterator
 * @param begin any begin ConstIterator bounding a range
 * @param end   any end   ConstIterator bounding a range
 * @tparam SC any model of segment computer
 */
template <typename SC>
void lastMaximalSegment(SC& s,
  const typename SC::ConstIterator& i,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end )
{
   lastMaximalSegment<SC>(s, i, begin, end,
typename DGtal::SegmentComputerTraits<SC>::Category() );
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////// next maximal segment

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param end any end ConstIterator
 * @tparam SC any model of CForwardSegmentComputer
 * @note firstMaximalSegment of s.end()
 */
template <typename SC>
void nextMaximalSegment(SC& s,
  const typename SC::ConstIterator& end,
  DGtal::ForwardSegmentComputer )
{
  firstMaximalSegment(s, s.end(), s.begin(), end, ForwardSegmentComputer() );
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param end end ConstIterator
 * @tparam SC any model of CBidirectionalSegmentComputer
 * @note firstMaximalSegment of s.end()
 */
template <typename SC>
void nextMaximalSegment(SC& s,
  const typename SC::ConstIterator& end,
  DGtal::BidirectionalSegmentComputer)
{
  firstMaximalSegment(s, s.end(), s.begin(), end, DGtal::BidirectionalSegmentComputer() );
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param end end ConstIterator
 * @tparam SC any model of CDynamicSegmentComputer
 */
template <typename SC>
void nextMaximalSegment(SC& s,
  const typename SC::ConstIterator& end,
  DGtal::DynamicSegmentComputer)
{
  typedef typename SC::ConstIterator ConstIterator;

  //rectract
  maximalRetraction(s, end);

  //if the intersection between the two consecutive maximal segments is
  //empty, i.e. if s has been retracted to a single element that cannot
  //be extended, the next maximal segment begins at s.end()
  if ( isNotEmpty<ConstIterator>(s.end(), end)
       && ( ! s.isExtendableFront() ) ) {
    ConstIterator i( s.end() );
    s.init(i);
  }

  //extend
  maximalExtension(s, end);
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param end end ConstIterator
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to DynamicSegmentComputer
 */
template <typename SC>
void nextMaximalSegment(SC& s,
  const typename SC::ConstIterator& end,
  DGtal::DynamicBidirectionalSegmentComputer)
{
  nextMaximalSegment(s, end, DGtal::DynamicSegmentComputer() );
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param end any end ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void nextMaximalSegment(SC& s,
  const typename SC::ConstIterator& end )
{
   nextMaximalSegment<SC>(s, end,
typename DGtal::SegmentComputerTraits<SC>::Category() );
}

////////////////////////////////////////////////////////////////
/////////////////////////////////////// previous maximal segment

/**
 * Computes the previous maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param begin any begin ConstIterator
 * @tparam SC any model of CForwardSegmentComputer
 * @note lastMaximalSegment of --s.begin()
 */
template <typename SC>
void previousMaximalSegment(SC& s,
  const typename SC::ConstIterator& begin,
  DGtal::ForwardSegmentComputer )
{
  if ( isNotEmpty<typename SC::ConstIterator>(s.begin(),begin) )
    lastMaximalSegment(s, --s.begin(), begin, s.end(), DGtal::ForwardSegmentComputer() );
}

/**
 * Computes the previous maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param begin any begin ConstIterator
 * @tparam SC any model of CBidirectionalSegmentComputer
 * @note lastMaximalSegment of --s.begin()
 */
template <typename SC>
void previousMaximalSegment(SC& s,
  const typename SC::ConstIterator& begin,
  DGtal::BidirectionalSegmentComputer)
{
  if ( isNotEmpty<typename SC::ConstIterator>(s.begin(),begin) )
    lastMaximalSegment(s, --s.begin(), begin, s.end(), DGtal::BidirectionalSegmentComputer() );
}

/**
 * Computes the previous maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param begin any begin ConstIterator
 * @tparam SC any model of CDynamicSegmentComputer
 */
template <typename SC>
void previousMaximalSegment(SC& s,
  const typename SC::ConstIterator& begin,
  DGtal::DynamicSegmentComputer)
{

  typedef typename SC::ConstIterator ConstIterator;

  //rectract
  oppositeEndMaximalRetraction(s, begin);

  //if the intersection between the two consecutive maximal segments is
  //empty, i.e. if s has been retracted to a single element that cannot
  //be extended, the previous maximal segment ends at s.begin()
  if ( isNotEmpty<ConstIterator>(s.begin(), begin)
       && ( ! s.isExtendableBack() ) ) {
    ConstIterator i( s.begin() ); --i;
    s.init(i);
  }

  //extend
  oppositeEndMaximalExtension(s, begin);

}

/**
 * Computes the previous maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param end end ConstIterator
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to DynamicSegmentComputer
 */
template <typename SC>
void previousMaximalSegment(SC& s,
  const typename SC::ConstIterator& end,
  DGtal::DynamicBidirectionalSegmentComputer)
{
  previousMaximalSegment(s, end, DGtal::DynamicSegmentComputer() );
}

/**
 * Computes the previous maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param begin any begin ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void previousMaximalSegment(SC& s,
  const typename SC::ConstIterator& begin )
{
  previousMaximalSegment(s, begin,
typename DGtal::SegmentComputerTraits<SC>::Category() );
}

} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SegmentComputerUtils_h

#undef SegmentComputerUtils_RECURSES
#endif // else defined(SegmentComputerUtils_RECURSES)
