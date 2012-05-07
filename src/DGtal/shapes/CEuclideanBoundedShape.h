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
 * @file CEuclideanBoundedShape.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/22
 *
 * Header file for concept CEuclideanBoundedShape.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CEuclideanBoundedShape_RECURSES)
#error Recursive header files inclusion detected in CEuclideanBoundedShape.h
#else // defined(CEuclideanBoundedShape_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CEuclideanBoundedShape_RECURSES

#if !defined CEuclideanBoundedShape_h
/** Prevents repeated inclusion of headers. */
#define CEuclideanBoundedShape_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"

#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/CSpace.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CEuclideanBoundedShape
  /**
   * DescriptionDescription of \b concept '\b CEuclideanBoundedShape' <p>
   * @ingroup Concepts
   * Aim: designs the concept of bounded shapes in DGtal (shape for
   * which upper and lower bounding bounds are available).
   * 
   * <p> Refinement of 
   *
   * <p> Associated types :
   *   - RealPoint: type for real points (to characterize upper/lower bounds).
   *
   * <p> Notation
   * - \t X : A type that is a model of CEuclideanBoundedShape
   * - \t x, \t y  : Object of type X
   *
   * <p> Definitions
   *
   * <p> Valid expressions and 
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   <tr> 
        <td class=CName> Lower bound        </td> 
        <td class=CExpression> x.getLowerBound()      </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> RealPoint     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> compute and return the lower bound of
   * the shape bounding box      </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>      </td>
      </tr>
  <tr> 
        <td class=CName> Upper bound        </td> 
        <td class=CExpression> x.getUpperBound()      </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> Point     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> compute and return the upper bound of
   * the shape bounding box      </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>      </td>
      </tr>
    
      </table>
   *
   * <p> Invariants###
   *
   * <p> Models###
   * 
   *   Implicit and Parametric shapes of the shape factory.
   *
   *
   * <p> Notes###
   */
  template <typename TShape>
  struct CEuclideanBoundedShape
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    typedef typename TShape::RealPoint RealPoint;
    
     BOOST_CONCEPT_USAGE( CEuclideanBoundedShape )
    {
      // Shape should have a getUpperBound() returning a Point.
      ConceptUtils::sameType( myP, myT.getUpperBound() );
      // Shape should have a getLowerBound() returning a Point.
      ConceptUtils::sameType( myP, myT.getLowerBound() );    
    }

    // ------------------------- Private Datas --------------------------------
  private:
    TShape myT;
    RealPoint myP;
    
  }; // end of concept CEuclideanBoundedShape
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CEuclideanBoundedShape_h

#undef CEuclideanBoundedShape_RECURSES
#endif // else defined(CEuclideanBoundedShape_RECURSES)
