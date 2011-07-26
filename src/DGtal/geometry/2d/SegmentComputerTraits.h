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
 * @file SegmentComputerTraits.h
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/07/25
 *
 * Header file for module SegmentComputerTraits.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SegmentComputerTraits_RECURSES)
#error Recursive header files inclusion detected in SegmentComputerTraits.h
#else // defined(SegmentComputerTraits_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SegmentComputerTraits_RECURSES

#if !defined SegmentComputerTraits_h
/** Prevents repeated inclusion of headers. */
#define SegmentComputerTraits_h


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

} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SegmentComputerTraits_h

#undef SegmentComputerTraits_RECURSES
#endif // else defined(SegmentComputerTraits_RECURSES)
