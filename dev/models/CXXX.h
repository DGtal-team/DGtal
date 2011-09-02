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
 * @file XXX.h
 * @author AUTHOR (\c EMAIL )
 * INSTITUTION
 *
 * @date 2000/??/??
 *
 * Header file for concept XXX.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(XXX_RECURSES)
#error Recursive header files inclusion detected in XXX.h
#else // defined(XXX_RECURSES)
/** Prevents recursive inclusion of headers. */
#define XXX_RECURSES

#if !defined XXX_h
/** Prevents repeated inclusion of headers. */
#define XXX_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "YYY/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace YYY
{

  /////////////////////////////////////////////////////////////////////////////
  // class XXX
  /**
     Description of \b concept '\b XXX' <p>
     @ingroup Concepts
     @brief Aim:
     
     <p> Refinement of
    
     <p> Associated types :
    
     <p> Notation
     - \t X : A type that is a model of XXX
     - \t x, \t y : object of type X
    
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
        <td class=CName>            </td> 
        <td class=CExpression>      </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType>      </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics>       </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>      </td>
      </tr>
    
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>

     A dummy model (for concept checking) is CXXXArchetype.

     <p> Notes <br>

     @tparam T the type that should be a model of XXX.
   */
  template <typename T> 
  struct XXX // : CoarserConcept<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::InnerType InnerType;
    // possibly check these types so as to satisfy a concept with
    BOOST_CONCEPT_ASSERT(( CConcept< InnerType > ));
    // To test if two types A and Y are equals, use
    BOOST_STATIC_ASSERT( ConceptUtils::sameType<A,X>::value );    
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( XXX )
    {
      // Static members of type A can be tested with
      ConceptUtils::sameType( myA, T::staticMember );
      // Method dummy should take parameter myA of type A and return
      // something of type B
      ConceptUtils::sameType( myB, myX.dummy( myA ) );
      // look at CInteger.h for testing tags.
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // only if T is default constructible.
    A myA;
    B myB;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept XXX
  
} // namespace YYY

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined XXX_h

#undef XXX_RECURSES
#endif // else defined(XXX_RECURSES)
