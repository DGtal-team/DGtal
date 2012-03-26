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
 * @file CPredicate.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/11/26
 *
 * Defines the concept checking class CPredicate.
 *
 * This file is part of the DGtal library.
 */

#if defined(CPredicate_RECURSES)
#error Recursive header files inclusion detected in CPredicate.h
#else // defined(CPredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CPredicate_RECURSES

#if !defined CPredicate_h
/** Prevents repeated inclusion of headers. */
#define CPredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CPredicate
  /**
     Description of \b concept '\b CPredicate' <p>
     @ingroup Concepts
     \brief Aim: Defines a predicate function, ie. a functor mapping a domain into the set of booleans.

     @tparam T the type that should be a model of this predicate
     @tparam TELement the type of an element of the predicate domain.

     <p> Refinement of  boost::Assignable<T>
    
     <p> Associated types :
    
     <p> Notation
     - \t X : A type that is a model of CPredicate
     - \t x : Object of type \t X
     - \t p : Object of type TElement
    
     <p> Definitions
    
     <p> Valid expressions and semantics <br>
     <table> 
     <tr> 
     <td class=CName> \b Name </td> 
     <td class=CExpression> \b Expression </td>
     <td class=CRequirements> \b Type requirements </td> 
     <td class=CReturnType> \b Return type </td>
     <td class=CPrecondition> \b Precondition </td> 
     <td class=CSemantics> \b Semantics </td> 
     <td class=CPostCondition> \b Postcondition </td> 
     <td class=CComplexity> \b Complexity </td>
     </tr>
     <tr> 
     <td class=CName>            Apply predicate </td>
     <td class=CExpression>      \t x( \t p ) </td> 
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \c bool</td>
     <td class=CPrecondition>    </td> 
     <td class=CSemantics>       the value of the predicate \t x at element \t p</td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>
    
     - specializations: CPointPredicate, CVertexPredicate
    
     <p> Notes <br>

     CPredicate allows to factor codes when writing concepts for new
     kinds of predicates.
   */
  template <typename T, typename TElement>
    struct CPredicate : boost::Assignable<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef TElement Element;

    BOOST_CONCEPT_USAGE( CPredicate )
    {
      checkConstConstraints();
    }

    void checkConstConstraints() const
    {
      // x( p ) returns bool.
      ConceptUtils::sameType( myBool, myPred.operator() ( myElement ) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myPred;
    Element myElement;
    bool myBool;
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CPredicate
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CPredicate_h

#undef CPredicate_RECURSES
#endif // else defined(CPredicate_RECURSES)
