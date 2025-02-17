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
 * @file CBidirectionalRangeWithWritableIteratorFromPoint.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/27
 *
 * Header file for concept CBidirectionalRangeWithWritableIteratorFromPoint.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CBidirectionalRangeWithWritableIteratorFromPoint_RECURSES)
#error Recursive header files inclusion detected in CBidirectionalRangeWithWritableIteratorFromPoint.h
#else // defined(CBidirectionalRangeWithWritableIteratorFromPoint_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBidirectionalRangeWithWritableIteratorFromPoint_RECURSES

#if !defined CBidirectionalRangeWithWritableIteratorFromPoint_h
/** Prevents repeated inclusion of headers. */
#define CBidirectionalRangeWithWritableIteratorFromPoint_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/base/CBidirectionalRangeWithWritableIterator.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace concepts
  {

    /////////////////////////////////////////////////////////////////////////////
    // class CBidirectionalRangeWithWritableIteratorFromPoint
    /**
       DescriptionDescription of \b concept '\b CBidirectionalRangeWithWritableIteratorFromPoint' <p>
       @ingroup Concepts
       @brief Aim: refined concept of  single pass range with an routputIterator() method from a point.

       # Refinement of CBidirectionalRangeWithWritableIterator

       # Associated types

       # Notation
       - X : A type that is a model of CBidirectionalRangeWithWritableIteratorFromPoint
       - x,  y : object of type X
       - Point: A type of Point


       # Definitions

       # Valid expressions and semantics


       | Name                    | Expression                           | Type requirements    | Return type          | Precondition | Semantics                                                         | Post condition | Complexity |
       |-------------------------|--------------------------------------|----------------------|----------------------|--------------|-------------------------------------------------------------------|----------------|------------|
       | reverse output iterator | routputIterator(const Point &aPoint) | aPoint of type Point | ReveseOutputIterator |              | Returns a reverse output iterator on the range at point \a aPoint |                |            |

       # Invariants

       # Models

       ImageContainerBySTLVector::Range

       # Notes

       @tparam T the type that should be a model of CBidirectionalRangeWithWritableIteratorFromPoint.
       @tparam Value the type of object t in (*it) = t.
    */
    template <typename T, typename Value>
    concept CBidirectionalRangeWithWritableIteratorFromPoint = 
      CBidirectionalRangeWithWritableIterator<T, Value> && requires(T myX, typename T::Point myPoint) {
        { myX.routputIterator( myPoint) } -> std::same_as<typename T::ReverseOutputIterator>;
      };
  } // namespace concepts

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CBidirectionalRangeWithWritableIteratorFromPoint_h

#undef CBidirectionalRangeWithWritableIteratorFromPoint_RECURSES
#endif // else defined(CBidirectionalRangeWithWritableIteratorFromPoint_RECURSES)
