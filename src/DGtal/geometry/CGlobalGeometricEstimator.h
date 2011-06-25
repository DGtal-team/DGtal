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
 * @file CGlobalGeometricEstimator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/06/25
 *
 * Header file for concept CGlobalGeometricEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CGlobalGeometricEstimator_RECURSES)
#error Recursive header files inclusion detected in CGlobalGeometricEstimator.h
#else // defined(CGlobalGeometricEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CGlobalGeometricEstimator_RECURSES

#if !defined CGlobalGeometricEstimator_h
/** Prevents repeated inclusion of headers. */
#define CGlobalGeometricEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CGlobalGeometricEstimator
  /**
   * Description of \b concept '\b CGlobalGeometricEstimator' <p>
   * @ingroup Concepts
   * Aim: Speicify the concept of global geometric estimator. 
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t T : A type that is a model of CGlobalGeometricEstimator
   * - \t x, \t y	: Object of type T
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
   * <td> init</td> <td> x.init(h,aSet)</td><td> aSet of type T::Set, h of
   * type double </td><td>void</td>
   * <td> </td> <td>Only called once, init the proces with a grid step
   * h and a given set</td> <td> </td> <td> O(1) </td>
   * </tr>
    * <tr> 
   * <td> eval</td> <td> x.eval()</td><td></td><td>a value of type T::Quantity</td>
   * <td>T.init(h,aSet) should have been called before </td> <td>Evaluate the  global estimator specified during
   * the init()</td> <td> </td> <td> Algorithm dependent </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   *
   * <p> Notes <br>
   */
  template <typename T>
  struct CGlobalGeometricEstimator
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    ///Models must define a Quantity type
    typedef typename T::Quantity Quantity;
    ///Models must define a Set type
    typedef typename T::Set Set;
    
    BOOST_CONCEPT_USAGE(CGlobalGeometricEstimator)
    {
      aGlobalEstimator.init(aH,aSet);
      ConceptUtils::sameType( aQuantity, aGlobalEstimator.eval());
    }
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    T aGlobalEstimator;
    Set *aSet;
    double aH;
    Quantity aQuantity;
    

  }; // end of concept CGlobalGeometricEstimator
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CGlobalGeometricEstimator_h

#undef CGlobalGeometricEstimator_RECURSES
#endif // else defined(CGlobalGeometricEstimator_RECURSES)
