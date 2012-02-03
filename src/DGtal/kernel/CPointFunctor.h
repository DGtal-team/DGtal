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
 * @file CPointFunctor.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/02
 *
 * Header file for concept CPointFunctor.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CPointFunctor_RECURSES)
#error Recursive header files inclusion detected in CPointFunctor.h
#else // defined(CPointFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CPointFunctor_RECURSES

#if !defined CPointFunctor_h
/** Prevents repeated inclusion of headers. */
#define CPointFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CPointFunctor
  /**
     Description of \b concept '\b CPointFunctor' <p>
     @ingroup Concepts
     \brief Aim: Defines a functor on points.
     
     Associates values to points.
    
     <p> Refinement of Assignable
    
     <p> Associated types :
    
     - Point : specifies the type for a point (inner type).
     - Point : specifies the type for a value (inner type).
    
     <p> Notation
     - \t X : A type that is a model of CPointFunctor
     - \t x : Object of type \t X
     - \t p : Object of type Point
     - \t v : Object of type Value
    
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
     <td class=CName>            Apply function </td>
     <td class=CExpression>      \t x( \t p ) </td> 
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \c v </td>
     <td class=CPrecondition>    </td> 
     <td class=CSemantics>       the value of the function \t x at point \t p</td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>
    
     - Shapes and images are models of (refinements of) this concept  
    
     <p> Notes <br>
   */
  template <typename T>
  struct CPointFunctor : boost::Assignable<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::Point Point;
    typedef typename T::Value Value;

    BOOST_CONCEPT_USAGE( CPointFunctor )
    {
      // x( p ) returns myV.
      ConceptUtils::sameType( myV, myF.operator() ( myPoint ) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myF;
    Point myPoint;
    Value myV;
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CPointFunctor
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CPointFunctor_h

#undef CPointFunctor_RECURSES
#endif // else defined(CPointFunctor_RECURSES)
