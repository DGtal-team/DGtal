*/**
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
 * @file CValue.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/01
 *
 * Header file for concept CValue.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CValue_RECURSES)
#error Recursive header files inclusion detected in CValue.h
#else // defined(CValue_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CValue_RECURSES

#if !defined CValue_h
/** Prevents repeated inclusion of headers. */
#define CValue_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
#include "DGtal/images/CQuantity.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CValue
  /**
   * Description of \b concept '\b CValue' <p>
   * @ingroup Concepts
   *
   * Aim: Represents a Value.
   * 
   * <p> Refinement of CQuantity and boost::DefaultConstructible
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CValue
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
   *
   * <p> Notes <br>
   */
  template <typename T>
  struct CValue  :  CQuantity<T>, boost::DefaultConstructible<T>
    {
      // ----------------------- Concept checks ------------------------------
    public:
    
      // ------------------------- Private Datas --------------------------------
    private:
    
      // ------------------------- Internals ------------------------------------
    private:
    
    }; // end of concept CValue
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CValue_h

#undef CValue_RECURSES
#endif // else defined(CValue_RECURSES)
