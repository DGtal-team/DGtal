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


#include "DGtal/base/Circulator.h"

namespace DGtal
{

////////////////////////////////////////////
// Categories
   struct ForwardSegmentComputer{}; 
   struct BidirectionalSegmentComputer{}; 
   struct DynamicSegmentComputer{}; 
   struct DynamicBidirectionalSegmentComputer{}; 


////////////////////////////////////////////
// Useful functions

/**
 * Computes the middle iterator of a given range, 
 * i.e. itb + (ite-itb)/2)
 * @param itb, ite, begin and end iterators of a range
 * @return the middle iterator of the range [itb,ite)
 * @tparam any iterator or circulator
 */
template<typename IC>
IC getMiddleIterator(const IC& itb, const IC& ite) {
  typedef typename IteratorCirculatorTagTraits<IC>::Category Category; 
  getMiddleIterator(itb, ite, Category() ); 
} 

/**
 * Specialization for random access category
 */
template<typename IC>
IC getMiddleIterator(const IC& itb, const IC& ite, RandomAccessCategory)
{
  return itb + ((ite-itb)/2);    
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

///////////////////////////////////////////////////////////
/////////////////////////////////////// extension functions

/**
 * Calls s.extend() while possible
 * @param s any instance of SegmentComputer
 * @param end any ConstIterator
 * @tparam SC any model of CSegmentComputer
 */
template <typename SC>
void segmentComputerMaximalExtension(SC& s, const typename SC::ConstIterator& end) {
  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type; 
  segmentComputerMaximalExtension( s, end, Type() ); 
}

/**
 * Specialization for Iterator type
 */
template <typename SC>
void segmentComputerMaximalExtension(SC& s, const typename SC::ConstIterator& end, IteratorType ) {
  //stop if s.end() == end
  while ( (s.end() != end)
	     && (s.extend()) ) {}
}

/**
 * Specialization for Circulator type
 */
template <typename SC>
void segmentComputerMaximalExtension(SC& s, const typename SC::ConstIterator& end, CirculatorType ) {
  //stop if the segment is the whole range
  const typename SC::ConstIterator newEnd( s.begin() ); 
  while ( (s.extend())
       && (s.end() != newEnd) ) {}
}

/**
 * Calls s.extendOppositeEnd() while possible
 * @param s any instance of (bidirectional)SegmentComputer
 * @param begin any ConstIterator
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void segmentComputerMaximalOppositeExtension(SC& s, const typename SC::ConstIterator& begin) {
  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type; 
  segmentComputerMaximalOppositeExtension( s, begin, Type() ); 
}

/**
 * Specialization for Iterator type
 */
template <typename SC>
void segmentComputerMaximalOppositeExtension(SC& s, const typename SC::ConstIterator& begin, IteratorType ) {
  //extend one more time if s.begin() == begin
  while ( (s.begin() != begin)
	     && (s.extendOppositeEnd()) ) {}
  if (s.begin() == begin) s.extendOppositeEnd();
}

/**
 * Specialization for Circulator type
 */
template <typename SC>
void segmentComputerMaximalOppositeExtension(SC& s, const typename SC::ConstIterator& begin, CirculatorType ) {
  //stop if the segment is the whole range
  const typename SC::ConstIterator newBegin( s.end() ); 
  while ( (s.extendOppositeEnd())
       && (s.begin() != newBegin) ) {}
}

/**
 * Calls alternatively s.extend() and s.extendOppositeEnd() 
 * while it is possible
 * @param s any instance of (bidirectional)SegmentComputer
 * @param begin, end, begin and end iterator of a range
 * @return 'true' if the extension at the front fails first
 * and 'false' if the extension at the back fails first
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
bool segmentComputerMaximalSymmetricExtension(SC& s, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end) {

  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type; 
  return segmentComputerMaximalSymmetricExtension( s, begin, end, Type() ); 

}

/**
 * Specialization for Iterator type
 */
template <typename SC>
bool segmentComputerMaximalSymmetricExtension(SC& s, 
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
      if ( s.end() != end ) flagOk = s.extend();
      else flagOk = false; 
    } else {
      flagForward = true; 
      if ( s.begin() != begin ) flagOk = s.extendOppositeEnd();
      else flagOk = false; 
    } 
  }
  //extend one more time if s.begin() == begin
  if (s.begin() != begin ) {
    if (s.extendOppositeEnd()) return !s.extend(); 
    else return false; 
  } else {
    return !flagForward; 
  }

}

/**
 * Specialization for Circulator type
 */
template <typename SC>
bool segmentComputerMaximalSymmetricExtension(SC& s, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  CirculatorType ) {

  bool flagOk = true; 
  bool flagForward = true; 
  //while the extensions are possible and
  //the segment does not correspond to the whole range
  while ( (flagOk) && ( s.end() != s.begin() ) )  {
    if (flagForward) {
      flagForward = false; 
      flagOk = s.extend(); 
    } else {
      flagForward = true; 
      flagOk = s.extendOppositeEnd(); 
    } 
  }
  return !flagForward; 
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////// longest segment

/**
 * Computes the longest possible segment from [i]
 * @param s any instance of SegmentComputer
 * @param i, a given ConstIterator
 * @param end, any end ConstIterator
 * @tparam SC any model of CSegmentComputer
 */
template <typename SC>
void longestSegment(SC& s, 
                   const typename SC::ConstIterator& i, 
                   const typename SC::ConstIterator& end) 
{
  typedef typename IteratorCirculatorTraits<typename SC::ConstIterator>::Type Type; 
  longestSegment( s, i, end, Type() ); 
}

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
    segmentComputerMaximalExtension(s, end, IteratorType() );
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
  segmentComputerMaximalExtension(s, end, CirculatorType() );
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////// first maximal segment
/**
 * Computes the first maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end ) 
{
  	firstMaximalSegment(s, i, begin, end, 
                      typename SC::Category() );
}

/**
 * Computes the first maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CForwardSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  ForwardSegmentComputer ) 
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
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  BidirectionalSegmentComputer) 
{
  s.init(i);

  segmentComputerMaximalOppositeExtension(s, begin);
  segmentComputerMaximalExtension(s, end);
}

/**
 * Computes the first maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CDynamicSegmentComputer
 * @note calls the function dedicated to ForwardSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  DynamicSegmentComputer) 
{
  firstMaximalSegment(s,i,begin,end,ForwardSegmentComputer() );
}

/**
 * Computes the first maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to BidirectionalSegmentComputer
 */
template <typename SC>
void firstMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  DynamicBidirectionalSegmentComputer) 
{
  firstMaximalSegment(s,i,begin,end,BidirectionalSegmentComputer() );
}

////////////////////////////////////////////////////////////////
/////////////////////////////////// most centered maximal segment
/**
 * Computes the most centered maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end ) 
{
  	mostCenteredMaximalSegment(s, i, begin, end, 
                      typename SC::Category() );
}

/**
 * Computes the most centered maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CForwardSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  ForwardSegmentComputer ) 
{

  typedef typename SC::ConstIterator ConstIterator; 
  typedef typename SC::Reverse ReverseSegmentComputer; 
  typedef typename ReverseSegmentComputer::ConstIterator ConstReverseIterator; 

 //get the first maximal segment passing through i

  firstMaximalSegment( s, i, begin, end, ForwardSegmentComputer() );

 //get the next maximal segment while i is not the middle of 
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
      ConstIterator begin = r.end().base(); 
      ConstIterator end = r.begin().base(); 

      while ( ( k != getMiddleIterator(begin, end) )&&( k != i ) )
        ++k; 
      if ( k != i ) {
   
        //get the next maximal segment
        firstMaximalSegment( s, s.end(), begin, end, ForwardSegmentComputer() );

      }

    } else {
      k = i; 
    }
  }
}

/**
 * Computes the most centered maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  BidirectionalSegmentComputer) 
{
  
  if ( (isNotEmpty(i,end)) || (isNotEmpty(i,begin)) ) { 

    s.init(i);

    //symmetric extension
    if ( (isNotEmpty(i,end)) && (isNotEmpty(i,begin)) ) { 
      segmentComputerMaximalSymmetricExtension(s, begin, end); 
    }

  //forward extension
  segmentComputerMaximalExtension(s, end);
      
  //backward extension
  segmentComputerMaximalOppositeExtension(s, begin);

  }

}

/**
 * Computes the most centered maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CDynamicSegmentComputer
 * @note calls the function dedicated to ForwardSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  DynamicSegmentComputer) 
{
  mostCenteredMaximalSegment(s,i,begin,end,ForwardSegmentComputer() );
}

/**
 * Computes the most centered maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to BidirectionalSegmentComputer
 */
template <typename SC>
void mostCenteredMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  DynamicBidirectionalSegmentComputer) 
{
  mostCenteredMaximalSegment(s,i,begin,end,BidirectionalSegmentComputer() );
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////// last maximal segment
/**
 * Computes the last maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end ) 
{
  	lastMaximalSegment(s, i, begin, end, 
                      typename SC::Category() );
}

/**
 * Computes the last maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CForwardSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  ForwardSegmentComputer ) 
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
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
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

  segmentComputerMaximalExtension(s, end);
  segmentComputerMaximalOppositeExtension(s, begin);
}

/**
 * Computes the last maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CDynamicSegmentComputer
 * @note calls the function dedicated to ForwardSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  DynamicSegmentComputer) 
{
  lastMaximalSegment(s,i,begin,end,ForwardSegmentComputer() );
}

/**
 * Computes the last maximal segment passing through i
 * @param i any ConstIterator
 * @param begin, end, any pair of ConstIterators bounding a range
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to BidirectionalSegmentComputer
 */
template <typename SC>
void lastMaximalSegment(SC& s, 
  const typename SC::ConstIterator& i, 
  const typename SC::ConstIterator& begin, 
  const typename SC::ConstIterator& end, 
  DynamicBidirectionalSegmentComputer) 
{
  lastMaximalSegment(s,i,begin,end,BidirectionalSegmentComputer() );
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////// next maximal segment
/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param end, any end ConstIterator
 * @tparam SC any model of CSegmentComputer
 */
template <typename SC>
void nextMaximalSegment(SC& s, 
  const typename SC::ConstIterator& end ) 
{
  	nextMaximalSegment(s, end, 
                      typename SC::Category() );
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param end, any end ConstIterator
 * @tparam SC any model of CForwardSegmentComputer
 * @note fistMaximalSegment of s.end()
 */
template <typename SC>
void nextMaximalSegment(SC& s, 
  const typename SC::ConstIterator& end, 
  ForwardSegmentComputer ) 
{
  firstMaximalSegment(s, s.end(), s.begin(), end, ForwardSegmentComputer() );
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param i any ConstIterator
 * @param end, end ConstIterator
 * @tparam SC any model of CBidirectionalSegmentComputer
 * @note fistMaximalSegment of s.end()
 */
template <typename SC>
void nextMaximalSegment(SC& s, 
  const typename SC::ConstIterator& end, 
  BidirectionalSegmentComputer) 
{
  firstMaximalSegment(s, s.end(), s.begin(), end, BidirectionalSegmentComputer() );
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param end, end ConstIterator
 * @tparam SC any model of CDynamicSegmentComputer
 */
template <typename SC>
void nextMaximalSegment(SC& s, 
  const typename SC::ConstIterator& end, 
  DynamicSegmentComputer) 
{
  while ( ( (s.end() == end )
          ||(! s.isExtendable() ) )
        &&(s.retract() ) ) {} 

  segmentComputerMaximalExtension(s, end);
}

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param end, end ConstIterator
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to DynamicSegmentComputer
 */
template <typename SC>
void nextMaximalSegment(SC& s, 
  const typename SC::ConstIterator& end, 
  DynamicBidirectionalSegmentComputer) 
{ 
  nextMaximalSegment(s, end, DynamicSegmentComputer() ); 
}

} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SegmentComputerUtils_h

#undef SegmentComputerUtils_RECURSES
#endif // else defined(SegmentComputerUtils_RECURSES)
