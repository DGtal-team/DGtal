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
 * @file CBidirectionalOutputRangeFromPoint.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/27
 *
 * Header file for concept CBidirectionalOutputRangeFromPoint.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CBidirectionalOutputRangeFromPoint_RECURSES)
#error Recursive header files inclusion detected in CBidirectionalOutputRangeFromPoint.h
#else // defined(CBidirectionalOutputRangeFromPoint_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBidirectionalOutputRangeFromPoint_RECURSES

#if !defined CBidirectionalOutputRangeFromPoint_h
/** Prevents repeated inclusion of headers. */
#define CBidirectionalOutputRangeFromPoint_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CBidirectionalOutputRange.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CBidirectionalOutputRangeFromPoint
  /**
DescriptionDescription of \b concept '\b CBidirectionalOutputRangeFromPoint' <p>
@ingroup Concepts
@brief Aim: refined concept of  single pass range with an routputIterator() method from a point.

### Refinement of CBidirectionalOutputRange

### Associated types :

### Notation
- X : A type that is a model of CBidirectionalOutputRangeFromPoint
- x,  y : object of type X
- Point: A type of Point


### Definitions

### Valid expressions and semantics


| Name                    | Expression                           | Type requirements    | Return type          | Precondition | Semantics                                                         | Post condition | Complexity |
|-------------------------|--------------------------------------|----------------------|----------------------|--------------|-------------------------------------------------------------------|----------------|------------|
| reverse output iterator | routputIterator(const Point &aPoint) | aPoint of type Point | ReveseOutputIterator |              | Returns a reverse output iterator on the range at point \a aPoint |                |            |

### Invariants

### Models

ImageContainerBySTLVector::Range

### Notes

@tparam T the type that should be a model of CBidirectionalOutputRangeFromPoint.
@tparam Value the type of object t in (*it) = t.
      */
  template <typename T, typename Value>
  struct CBidirectionalOutputRangeFromPoint:
    CBidirectionalOutputRange<T,Value>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::Point Point;
    typedef typename T::ReverseOutputIterator ReverseOutputIterator;

    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CBidirectionalOutputRangeFromPoint )
    {
       ConceptUtils::sameType( myIt, myX.routputIterator( myPoint ) );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // do not require T to be default constructible.
    Point myPoint;
    ReverseOutputIterator myIt;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CBidirectionalOutputRangeFromPoint

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CBidirectionalOutputRangeFromPoint_h

#undef CBidirectionalOutputRangeFromPoint_RECURSES
#endif // else defined(CBidirectionalOutputRangeFromPoint_RECURSES)
