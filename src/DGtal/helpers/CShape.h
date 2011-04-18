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
 * @file CShape.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/22
 *
 * Header file for concept CShape.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CShape_RECURSES)
#error Recursive header files inclusion detected in CShape.h
#else // defined(CShape_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CShape_RECURSES

#if !defined CShape_h
/** Prevents repeated inclusion of headers. */
#define CShape_h

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
  // class CShape
  /**
   * Description of \b concept '\b CShape' <p>
   * @ingroup Concepts
   * Aim: designs the concept of constructible shapes in DGtal.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CShape
   * - \t x, \t y	: Object of type X
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
   * <td> </td> <td> </td> <td> </td> <td> </td>
   * <td> </td> <td> </td> <td> </td> <td> </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   *
   * All models of CShape are specified in the ShapeFactory.
   *
   * <p> Notes <br>
   */
  template <typename TShape>
  struct CShape
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    typedef typename TShape::Space::Point Point;
    typedef typename TShape::Space Space;
    
    BOOST_CONCEPT_ASSERT((CSpace<Space>));

 
    BOOST_CONCEPT_USAGE( CShape )
    {
      // Shape should have a getUpperBound() returning a Point.
      ConceptUtils::sameType( myP, myT.getUpperBound() );
      // Shape should have a getLowerBound() returning a Point.
      ConceptUtils::sameType( myP, myT.getLowerBound() );
      // Shape should have a operator() returning a double.
      //    ConceptUtils::sameType( aDouble, myT(myP) );
      // Shape should have an isInside() function returning a bool.
      ConceptUtils::sameType( aBool, myT.isInside(myP) );
      
    }

    // ------------------------- Private Datas --------------------------------
  private:
    TShape myT;
    Point myP;
    bool aBool;
    double aDouble;
    
  }; // end of concept CShape
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CShape_h

#undef CShape_RECURSES
#endif // else defined(CShape_RECURSES)
