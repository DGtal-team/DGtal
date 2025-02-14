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
 * @file CStack.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/12/07
 *
 * Header file for concept CStack.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CStack_RECURSES)
#error Recursive header files inclusion detected in CStack.h
#else // defined(CStack_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CStack_RECURSES

#if !defined CStack_h
/** Prevents repeated inclusion of headers. */
#define CStack_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace concepts
  {
    /////////////////////////////////////////////////////////////////////////////
    // class CStack
    /**
       Description of \b concept '\b CStack' <p>
       @ingroup Concepts
       @brief Aim: This concept gathers classes that provide 
       a stack interface

       # Refinement of copy constructible and assignable

       # Inner types
       - Value type of element stored in the stack
       - Size type that represents the number of elements

       # Notation
       - \e X : A type that is a model of CStack
       - \e x : object of type X
       - \e v : object of type Value

       # Definitions

       # Valid expressions and semantics

    | Name  | Expression | Type requirements | Return type            | Precondition | Semantics                          | Post condition | Complexity |
    |-------+------------+-------------------+------------------------+--------------+------------------------------------+----------------+------------|
    | size  | x.size()   |                   | Size                   |              | returns the number of elements     |                | O(1)       |
    | empty | x.empty()  |                   | bool                   |              | 'true' is empty, 'false' otherwise |                | O(1)       |
    | top   | x.top()    |                   | Value& or const Value& |              | returns the top element            |                | O(1)       |
    | push  | x.push(v)  |                   | void                   |              | inserts a new element at the top   |                | O(1)       |
    | pop   | x.pop()    |                   | void                   |              | removes the top element            |                | O(1)       |


       # Models

       std::stack, BackInsertionSequenceToStackAdapter, FrontInsertionSequenceToStackAdapter are the usual models of CStack.

       @tparam T the type that should be a model of CStack.
    */
    template <typename T>
    concept CStack = 
      std::is_copy_constructible_v<T> &&
      std::is_copy_assignable_v<T>  &&
      requires (T myX, typename T::Value v ) {
        { myX.empty() } -> std::same_as<bool>;
        { myX.size()  } -> std::same_as<typename T::size_type>;

        ConceptUtils::sameType(v, myX.top());
        myX.pop();
        myX.push(v);
      };
  }//namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CStack_h

#undef CStack_RECURSES
#endif // else defined(CStack_RECURSES)
