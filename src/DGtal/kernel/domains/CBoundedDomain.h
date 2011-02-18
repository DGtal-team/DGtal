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
 * @file CBoundedDomain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 * @date 2011/02/17
 *
 * Header file for concept CBoundedDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CBoundedDomain_RECURSES)
#error Recursive header files inclusion detected in CBoundedDomain.h
#else // defined(CBoundedDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBoundedDomain_RECURSES

#if !defined CBoundedDomain_h
/** Prevents repeated inclusion of headers. */
#define CBoundedDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
#include "DGtal/utils/ConceptUtils.h"
#include "DGtal/kernel/domains/CDomain.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CBoundedDomain
  /**
   * Description of \b concept '\b CBoundedDomain' <p> \brief Aim: This
   * concept represents a bounded digital domain, i.e. a non mutable subset of
   * points of the given digital space.
   * 
   * <p> Refinement of concept CDomain
   *
   * <p> Associated types :
   * - Domain : the type itself of the CBoundedDomain model.
   * - Space : the embedding digital space.
   * - Point : the point type of the space
   * - SizeType : the type used for counting elements of the space.
   * - Vector : the vector type of the space
   * - Predicate : the type of the predicate returning true for exactly the points of this domain.  
   * - ConstIterator : the type used for iterating/visiting the points of the domain.
   * 
   * 
   * <p> Notation
   * - \t X : A type that is a model of CBoundedDomain
   * - \t x	: Object of type X
   * - \t p	: Object of type Point
   * - \t it	: Object of type ConstIterator
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr> 
   * <td> Const Iterator \c begin</td> <td> it = x.begin() </td> 
   * <td> </td> <td> const ConstIterator &</td>
   * <td> </td> <td> return the iterator pointing on the first element of the domain.</td> <td> </td> <td> O(1) </td>
   * </tr>
   * <tr> 
   * <td> Const Iterator \c end</td> <td> it = x.end() </td> 
   * <td> </td> <td> const ConstIterator &</td>
   * <td> </td> <td> return the iterator pointing after the last element of the domain.</td> <td> </td> <td> O(1) </td>
   * </tr>
   * <tr> 
   * <td> lower bound</td> <td> x.lowerBound() </td> 
   * <td> </td> <td> const Point &</td>
   * <td> </td> <td> return the infimum of all points of the domain.</td> <td> </td> <td> O(1) </td>
   * </tr>
   * <tr> 
   * <td> upper bound</td> <td> x.upperBound() </td> 
   * <td> </td> <td> const Point &</td>
   * <td> </td> <td> return the supremum of all points of the domain.</td> <td> </td> <td> O(1) </td>
   * </tr>
   * <tr> 
   * <td> inside domain test</td> <td> x.isInside( p ) </td> 
   * <td> </td> <td> \c bool </td>
   * <td> </td> <td> return 'true' whenever \c p is in the domain.</td> <td> </td> <td>  </td>
   * </tr>
   * <tr> 
   * <td> inside domain predicate object</td> <td> x.predicate() </td> 
   * <td> </td> <td> const Predicate & </td>
   * <td> </td> <td> return a reference to the predicate object equivalent to the isinside(p) test.</td> <td> </td> <td>  </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   * HyperRectDomain 
   *
   * <p> Notes <br>
   *
   * @todo Complete domain checking.
   */
  template <typename T>
  struct CBoundedDomain: CDomain<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    BOOST_CONCEPT_USAGE( CBoundedDomain )
    {
      // Bounded Domain should have a TagTrue tag to IsBounded type.
      ConceptUtils::checkTrue( myIsBounded );
    }

  private:
    typename T::IsBounded myIsBounded;


   }; // end of concept CBoundedDomain
  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////

#endif // !defined CBoundedDomain_h

#undef CBoundedDomain_RECURSES
#endif // else defined(CBoundedDomain_RECURSES)
