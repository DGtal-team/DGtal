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
 * @file CQuantity.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/07
 *
 * Header file for concept CQuantity.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CQuantity_RECURSES)
#error Recursive header files inclusion detected in CQuantity.h
#else // defined(CQuantity_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CQuantity_RECURSES

#if !defined CQuantity_h
/** Prevents repeated inclusion of headers. */
#define CQuantity_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CLabel.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CQuantity
  /**
Description of \b concept '\b CQuantity' <p>
     @ingroup Concepts
     @brief Aim: defines the concept of quantity in DGtal.

###  Refinement of CLabel and boost::LessThanComparable

###  Associated types :

###  Notation
     - \t X : A type that is a model of CQuantity
     - \t x, \t y : object of type X

###  Definitions

###  Valid expressions and semantics
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

###  Invariants

###  Models


###  Notes

@tparam T the type that should be a model of CQuantity.
   */
  template <typename T>
  struct CQuantity : CLabel<T>, boost::LessThanComparable<T>
  {


  }; // end of concept CQuantity

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CQuantity_h

#undef CQuantity_RECURSES
#endif // else defined(CQuantity_RECURSES)
