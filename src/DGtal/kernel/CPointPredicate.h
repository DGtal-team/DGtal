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
 * @file CPointPredicate.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/10
 *
 * Header file for concept CPointPredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CPointPredicate_RECURSES)
#error Recursive header files inclusion detected in CPointPredicate.h
#else // defined(CPointPredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CPointPredicate_RECURSES

#if !defined CPointPredicate_h
/** Prevents repeated inclusion of headers. */
#define CPointPredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CPointPredicate
  /**
     Description of \b concept '\b CPointPredicate' <p>
     @ingroup Concepts
     \brief Aim: Defines a predicate on a point.
     
     Associates a boolean to points.
    
     <p> Refinement of Assignable
    
     <p> Associated types :
    
     - Point : specifies the type for a point (inner type).
    
     <p> Notation
     - \t X : A type that is a model of CPointPredicate
     - \t x : Object of type \t X
     - \t p : Object of type Point
    
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
     <td class=CSemantics>       the value of the predicate \t x at point \t p</td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>
    
     - basic models: ConstantPointPredicate, TruePointPredicate, FalsePointPredicate, IsUpperPointPredicate, IsLowerPointPredicate, IsWithinPointPredicate
     - complex predicate constructor: BinaryPointPredicate
     - others: DomainPredicate,SetPredicate
    
     <p> Notes <br>
   */
  template <typename T>
  struct CPointPredicate /*: boost::Assignable<T>*/
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::Point Point;

    BOOST_CONCEPT_USAGE( CPointPredicate )
    {
      // x( p ) returns bool.
      ConceptUtils::sameType( myBool, myPred.operator() ( myPoint ) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myPred;
    Point myPoint;
    bool myBool;
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CPointPredicate
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CPointPredicate_h

#undef CPointPredicate_RECURSES
#endif // else defined(CPointPredicate_RECURSES)
