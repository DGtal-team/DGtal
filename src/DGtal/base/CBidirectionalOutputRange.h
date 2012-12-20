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
 * @file CBidirectionalOutputRange.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/27
 *
 *
 * This file is part of the DGtal library.
 */

#if defined(CBidirectionalOutputRange_RECURSES)
#error Recursive header files inclusion detected in CBidirectionalOutputRange.h
#else // defined(CBidirectionalOutputRange_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBidirectionalOutputRange_RECURSES

#if !defined CBidirectionalOutputRange_h
/** Prevents repeated inclusion of headers. */
#define CBidirectionalOutputRange_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CSinglePassOutputRange.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CBidirectionalOutputRange
  /**
Description of \b concept '\b CBidirectionalOutputRange'
     @ingroup Concepts
     @brief Aim: refined concept of bidirectional range which require that a reverse output iterator exists.


### Refinement of CSinglePassOutputRange

### Associated types :
- OutputIterator: type of output iterator on the range.

### Notation
- \a X : A type that is a model of CBidirectionalOutputRange
- \a x, \a y : object of type X


### Definitions


| Name| Expression       | Type requirements | Return type    | Precondition | Semantics                                          | Post condition | Complexity |
|----------|------------------|-------------------|----------------|--------------|----------------------------------------------------|----------------|------------|
| creation | routputIterator() |                   | ReverseOutputIterator |              | Returns a reverse output iterator on the range first element |                |            |
|          |                  |                   |                |              |                                                    |                |            |


### Invariants###

### Models###

ImageContainerBySTLVector::Range

### Notes###

@tparam T the type that should be a model of CBidirectionalOutputRange.
@tparam Value the type of object t in (*it) = t.

   */
  template <typename T, typename Value>
  struct CBidirectionalOutputRange : CSinglePassOutputRange<T, Value>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::ReverseOutputIterator  ReverseOutputIterator;
    // possibly check these types so as to satisfy a concept with
    //BOOST_CONCEPT_ASSERT(( CConcept< InnerType > ));

    BOOST_CONCEPT_USAGE( CBidirectionalOutputRange )
    {
      ConceptUtils::sameType( myOutput, myX.routputIterator( ) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // do not require T to be default constructible.
    ReverseOutputIterator myOutput;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CBidirectionalOutputRange

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CBidirectionalOutputRange_h

#undef CBidirectionalOutputRange_RECURSES
#endif // else defined(CBidirectionalOutputRange_RECURSES)
