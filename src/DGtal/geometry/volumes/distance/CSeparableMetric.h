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
 * @file CSeparableMetric.h
 * @brief Specifies what are the classes that implement a model of separable metrics.
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/11/24
 *
 * Header file for concept CSeparableMetric.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CSeparableMetric_RECURSES)
#error Recursive header files inclusion detected in CSeparableMetric.h
#else // defined(CSeparableMetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSeparableMetric_RECURSES

#if !defined CSeparableMetric_h
/** Prevents repeated inclusion of headers. */
#define CSeparableMetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CSeparableMetric
  /**
   * Description of \b concept '\b CSeparableMetric' <p>
   * @ingroup Concepts
   * Aim: The concept CSeparableMetric specifies what are the classes
   * that implement a model of separable metrics.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CSeparableMetric
   * - \t x, \t y  : Object of type X
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
   *    l_0, l_1 and l_2 metrics defined in the SeparableMetricHelper.H
   * <p> Notes <br>
   */
  template <typename T>
  struct CSeparableMetric
  {

    typedef typename T::InternalValue InternalValue;
    typedef typename T::Value Value;
    typedef typename T::Abscissa Abscissa;
    
    // ----------------------- Concept checks ------------------------------
  public:
    BOOST_CONCEPT_USAGE( CSeparableMetric )
    {
      //SeparableMetric  model should have a F(Abscissa, Abscissa, InternalValue) returing an InternalValue
      ConceptUtils::sameType( h, myT.F(a,a,h) );
      //SeparableMetric  model should have a Sep(Abscissa,InternalValue, Abscissa,InternalValue) returing an Value  
      ConceptUtils::sameType( a, myT.Sep(a,h,a,h) );
    }
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    T myT;
    Abscissa a;
    InternalValue h;
    
      
  }; // end of concept CSeparableMetric
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSeparableMetric_h

#undef CSeparableMetric_RECURSES
#endif // else defined(CSeparableMetric_RECURSES)
