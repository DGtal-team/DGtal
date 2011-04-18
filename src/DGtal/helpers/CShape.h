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
 * @file CImplicitShape.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/22
 *
 * Header file for concept CImplicitShape.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CImplicitShape_RECURSES)
#error Recursive header files inclusion detected in CImplicitShape.h
#else // defined(CImplicitShape_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CImplicitShape_RECURSES

#if !defined CImplicitShape_h
/** Prevents repeated inclusion of headers. */
#define CImplicitShape_h

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
  // class CImplicitShape
  /**
   * Description of \b concept '\b CImplicitShape' <p>
   * @ingroup Concepts
   * Aim: designs the concept if implicit functions constructible in DGtal.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CImplicitShape
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
   * All models of CImplicitShape are specified in the ImplicitShapeFactory.
   *
   * <p> Notes <br>
   */
  template <typename TShape>
  struct CImplicitShape
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    typedef typename TShape::Space::Point Point;
    typedef typename TShape::Space Space;
    
    BOOST_CONCEPT_ASSERT((CSpace<Space>));

 
    BOOST_CONCEPT_USAGE( CImplicitShape )
    {
      // ImplicitShape should have a getUpperBound() returning a Point.
      ConceptUtils::sameType( myP, myT.getUpperBound() );
      // ImplicitShape should have a getLowerBound() returning a Point.
      ConceptUtils::sameType( myP, myT.getLowerBound() );
      // ImplicitShape should have a operator() returning a double.
      //    ConceptUtils::sameType( aDouble, myT(myP) );
      // ImplicitShape should have an isInside() function returning a bool.
      ConceptUtils::sameType( aBool, myT.isInside(myP) );
      
    }

    // ------------------------- Private Datas --------------------------------
  private:
    TShape myT;
    Point myP;
    bool aBool;
    double aDouble;
    
  }; // end of concept CImplicitShape
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CImplicitShape_h

#undef CImplicitShape_RECURSES
#endif // else defined(CImplicitShape_RECURSES)
