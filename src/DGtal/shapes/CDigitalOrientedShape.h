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
 * @file CDigitialOrientedShape.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/10/20
 *
 * Header file for concept CDigitalOrientedShape.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDigitalOrientedShape_RECURSES)
#error Recursive header files inclusion detected in CDigitalOrientedShape.h
#else // defined(CDigitalOrientedShape_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDigitalOrientedShape_RECURSES

#if !defined CDigitalOrientedShape_h
/** Prevents repeated inclusion of headers. */
#define CDigitalOrientedShape_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#inclide "DGtal/base/CCommutativeRing.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalOrientedShape
  /**
     Description of \b concept '\b CDigitalOrientedShape' <p>
     @ingroup Concepts
     @brief Aim: characterze models of digital oriented shapes. For example,
     models should provide an orientation method  for  points on a SpaceND.
     
     <p> Refinement of
    
     <p> Associated types :
     - Orientation: scalar and signed type for orientation values
     (model of CCommutativeRing). 
     - Point: type for digital points.

     <p> Notation
     - \t X : A type that is a model of CDigitalOrientedShape
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
        <td class=CName> Orientation method           </td> 
        <td class=CExpression>  x.orientation( aPoint)    </td>
        <td class=CRequirements> aPoint of type const Point &   </td> 
        <td class=CReturnType> Orientation     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> return the orientation of a point @c
     aPoint according to the shape. Negative return value means that
     the point is inside the shape (or on the negative side of the
     shape).  Positive return value means that
     the point is outside the shape (or on the positive side of the
     shape). Zero value means that the point is on the shape.      </Td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>      </td>
      </tr>
    
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>

      Parametric and implicit shapes in the DGtal shape factory, 

     <p> Notes <br>

     @tparam T the type that should be a model of CDigitalOrientedShape.
   */
  template <typename T> 
  struct CDigitalOrientedShape
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::Orientation Orientation;
    typedef typename T::Point Point;
   
    // possibly check these types so as to satisfy a concept with
    BOOST_CONCEPT_ASSERT(( CCommutativeRing< Orientation > ));
  
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CDigitalOrientedShape )
    {
      ConceptUtils::sameType( myA, myX.orientation( p ));
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // only if T is default constructible.
    Orientation myA;
    Point p;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CDigitalOrientedShape
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDigitalOrientedShape_h

#undef CDigitalOrientedShape_RECURSES
#endif // else defined(CDigitalOrientedShape_RECURSES)
