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
 * @file XXX.h
 * @author AUTHOR (\c EMAIL )
 * INSTITUTION
 *
 * @date 2000/??/??
 *
 * Header file for concept XXX.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(XXX_RECURSES)
#error Recursive header files inclusion detected in XXX.h
#else // defined(XXX_RECURSES)
/** Prevents recursive inclusion of headers. */
#define XXX_RECURSES

#if !defined XXX_h
/** Prevents repeated inclusion of headers. */
#define XXX_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "YYY/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace YYY
{

  /////////////////////////////////////////////////////////////////////////////
  // class XXX
  /**
   * Description of \b concept '\b XXX' <p>
   * @ingroup Concepts
   * Aim:
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of XXX
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
   * <p> Notes <br>
   */
  template <typename T>
  struct XXX
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept XXX
  
} // namespace YYY


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "YYY/ZZZ/XXX.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined XXX_h

#undef XXX_RECURSES
#endif // else defined(XXX_RECURSES)
