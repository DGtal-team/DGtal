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
 * @file CSinglePassOutputRangeFromPoint.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/27
 *
 * Header file for concept CSinglePassOutputRangeFromPoint.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CSinglePassOutputRangeFromPoint_RECURSES)
#error Recursive header files inclusion detected in CSinglePassOutputRangeFromPoint.h
#else // defined(CSinglePassOutputRangeFromPoint_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSinglePassOutputRangeFromPoint_RECURSES

#if !defined CSinglePassOutputRangeFromPoint_h
/** Prevents repeated inclusion of headers. */
#define CSinglePassOutputRangeFromPoint_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CSinglePassOutputRange.h"
#include "DGtal/base/CConstSinglePassRangeFromPoint.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CSinglePassOutputRangeFromPoint
  /**
     Description of \b concept '\b CSinglePassOutputRangeFromPoint' <p>
     @ingroup Concepts
     @brief Aim: refined concept of single pass range with a outputIterator() method from a point.

     ### Refinement of CConstSinglePassRangeFromPoint and CSinglePassOutputRange

     ### Associated types :

     ### Notation
     - X : A type that is a model of CSinglePassOutputRangeFromPoint
     - x,  y : object of type X
     - Point: A type of Point

     ### Definitions

     ### Valid expressions and semantics

     | Name  | Expression                 | Type requirements    | Return type   | Precondition | Semantics                                           | Post condition | Complexity |
     |-------|----------------------------|----------------------|---------------|--------------|-----------------------------------------------------|----------------|------------|
     | output iterator | outputIterator(const Point &aPoint) | aPoint of type Point | OutputIterator |              | Returns an output iterator on the range first element |                |            |

     ### Invariants

     ### Models
     - ImageContainerBySTLVector::Range

     ### Notes

     @tparam T the type that should be a model of CSinglePassOutputRangeFromPoint.
     @tparam Value the type of object t in (*it) = t.

   */
  template <typename T, typename Value>
  struct CSinglePassOutputRangeFromPoint:
    CConstSinglePassRangeFromPoint<T,Value>,
    CSinglePassOutputRange<T,Value>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::OutputIterator OutputIterator;
    typedef typename T::Point Point;

    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CSinglePassOutputRangeFromPoint )
    {
       ConceptUtils::sameType( myIt, myX.begin( myPoint ) );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // do not require T to be default constructible.
    Point myPoint;
    OutputIterartor myIt;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CSinglePassOutputRangeFromPoint

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSinglePassOutputRangeFromPoint_h

#undef CSinglePassOutputRangeFromPoint_RECURSES
#endif // else defined(CSinglePassOutputRangeFromPoint_RECURSES)
