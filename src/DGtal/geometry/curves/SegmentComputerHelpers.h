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
 * @file SegmentComputerHelpers.h
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/07/25
 *
 * @brief Categories of segment computers and functions computing
 * maximal segments (used by the segmentation algorithms)
 *
 * This file is part of the DGtal library.
 */

#if defined(SegmentComputerHelpers_RECURSES)
#error Recursive header files inclusion detected in SegmentComputerHelpers.h
#else // defined(SegmentComputerHelpers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SegmentComputerHelpers_RECURSES

#if !defined SegmentComputerHelpers_h
/** Prevents repeated inclusion of headers. */
#define SegmentComputerHelpers_h


#include <type_traits>
#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/IteratorFunctions.h"

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


///////////////////////////////////////////////////////////
/////////////////////////////////////// extension functions

/**
 * Specialization for Iterator type
 */
template <typename SC>
void maximalExtension(SC& s, const typename SC::ConstIterator& end, IteratorType );

/**
 * Specialization for Circulator type
 */
template <typename SC>
void maximalExtension(SC& s, const typename SC::ConstIterator& /*end*/, CirculatorType );

/**
 * Calls s.extendFront() while possible
 * @param s any instance of segment computer
 * @param end any ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void maximalExtension(SC& s, const typename SC::ConstIterator& end);

/**
 * Specialization for Iterator type
 */
template <typename SC>
void oppositeEndMaximalExtension(SC& s, const typename SC::ConstIterator& begin, IteratorType );

/**
 * Specialization for Circulator type
 */
template <typename SC>
void oppositeEndMaximalExtension(SC& s, const typename SC::ConstIterator& begin, CirculatorType );

/**
 * Calls s.extendBack() while possible
 * @param s any instance of (bidirectional) segment computer
 * @param begin any ConstIterator
 * @tparam SC any model of CBidirectionalSegmentComputer
 */
template <typename SC>
void oppositeEndMaximalExtension(SC& s, const typename SC::ConstIterator& begin);

/**
 * Specialization for Iterator type
 */
template <typename SC>
bool maximalSymmetricExtension(SC& s,
  const typename SC::ConstIterator& begin,
  const typename SC::ConstIterator& end,
  IteratorType );

/**
 * Specialization for Circulator type
 */
template <typename SC>
bool maximalSymmetricExtension(SC& s,
                               const typename SC::ConstIterator& begin,
                               const typename SC::ConstIterator& end,
                               CirculatorType );

/**
 * Calls alternatively s.extendFront() and s.extendBack()
 * while it is possible, beginning with s.extendFront().
 *
 * If s is initialized with a single element, the extensions are
 * symmetric around this element as long as they are possible on both
 * sides (symmetric tangent, definition 2 of Feschet and Tougne, 1999,
 * see mostCenteredMaximalSegment). Since the extension at the front
 * is tried first, the resulting segment is this symmetric tangent,
 * possibly extended by one element at the front.
 *
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
  const typename SC::ConstIterator& end);

/**
 * Calls s.retractBack() while s.isExtendableFront()
 * returns false
 * @param s any instance of segment computer
 * @param end any ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void maximalRetraction(SC& s, const typename SC::ConstIterator& end);

/**
 * Calls s.retractFront()
 * while s.isExtendableBack() returns false
 * @param s any instance of segment computer
 * @param begin any ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void oppositeEndMaximalRetraction(SC& s, const typename SC::ConstIterator& begin);
////////////////////////////////////////////////////////////////
////////////////////////////////////////// longest segment

/**
 * Specialization for Iterator type
 */
template <typename SC>
void longestSegment(SC& s,
                   const typename SC::ConstIterator& i,
                   const typename SC::ConstIterator& end,
                   IteratorType );

/**
 * Specialization for Circulator type
 */
template <typename SC>
void longestSegment(SC& s,
                   const typename SC::ConstIterator& i,
                   const typename SC::ConstIterator& end,
                   CirculatorType );

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
                   const typename SC::ConstIterator& end);
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
  DGtal::ForwardSegmentComputer );

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
  DGtal::BidirectionalSegmentComputer);

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
  DGtal::DynamicSegmentComputer);

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
  DGtal::DynamicBidirectionalSegmentComputer);

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
  const typename SC::ConstIterator& end );
////////////////////////////////////////////////////////////////
/////////////////////////////////// most centered maximal segment

/**
 * Computes the most centered maximal segment passing through i
 * (see the function without category below for the definition).
 *
 * This version goes through the maximal segments containing i,
 * beginning with the first one, until i is not before the middle
 * of the intersection with the next maximal segment (as in the
 * algorithm of Feschet and Tougne, 1999).
 *
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
  DGtal::ForwardSegmentComputer );

/**
 * Computes the most centered maximal segment passing through i
 * (see the function without category below for the definition).
 *
 * This version follows the definition: the symmetric tangent at i is
 * computed by maximalSymmetricExtension, then extended at the front and
 * at the back.
 *
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
  DGtal::BidirectionalSegmentComputer);

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
  DGtal::DynamicSegmentComputer);

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
  DGtal::DynamicBidirectionalSegmentComputer);

/**
 * Computes the most centered maximal segment passing through i.
 *
 * It is not exactly the most centered one, but the maximal segment
 * containing the tangent at i defined by Feschet and Tougne
 * (definition 3 of the paper below, where the extension is first
 * tried on the right):
 * - the symmetric tangent at i is the segment [i-k, i+k] such that
 *   [i-k-1, i+k+1] is not a segment (or exceeds the range);
 * - the tangent at i is the symmetric tangent extended as much as
 *   possible at the front, then at the back if the extension at the
 *   front is not possible.
 *
 * Several maximal segments may contain the symmetric tangent; since the
 * extension is first tried at the front, the one with the largest index
 * (i.e. the last one) is returned. The returned segment is the maximal
 * segment obtained by extending the tangent as much as possible at the
 * front, then at the back.
 *
 * F. Feschet and L. Tougne, Optimal Time Computation of the Tangent
 * of a Discrete Curve: Application to the Curvature, DGCI 1999,
 * LNCS 1568, pp. 31-40.
 *
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
  const typename SC::ConstIterator& end );
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
  DGtal::ForwardSegmentComputer );

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
  BidirectionalSegmentComputer);

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
  DGtal::DynamicSegmentComputer);

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
  DGtal::DynamicBidirectionalSegmentComputer);

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
  const typename SC::ConstIterator& end );
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
  DGtal::ForwardSegmentComputer );

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
  DGtal::BidirectionalSegmentComputer);

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
  DGtal::DynamicSegmentComputer);

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
  DGtal::DynamicBidirectionalSegmentComputer);

/**
 * Computes the next maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param end any end ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void nextMaximalSegment(SC& s,
  const typename SC::ConstIterator& end );
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
  DGtal::ForwardSegmentComputer );

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
  DGtal::BidirectionalSegmentComputer);

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
  DGtal::DynamicSegmentComputer);

/**
 * Computes the previous maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param begin any begin ConstIterator
 * @tparam SC any model of CDynamicBidirectionalSegmentComputer
 * @note calls the function dedicated to DynamicSegmentComputer
 */
template <typename SC>
void previousMaximalSegment(SC& s,
  const typename SC::ConstIterator& begin,
  DGtal::DynamicBidirectionalSegmentComputer);

/**
 * Computes the previous maximal segment of s
 * (s is assumed to be maximal)
 * @param s any instance of segment computer
 * @param begin any begin ConstIterator
 * @tparam SC any model of segment computer
 */
template <typename SC>
void previousMaximalSegment(SC& s,
  const typename SC::ConstIterator& begin );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/SegmentComputerHelpers.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SegmentComputerHelpers_h

#undef SegmentComputerHelpers_RECURSES
#endif // else defined(SegmentComputerHelpers_RECURSES)
