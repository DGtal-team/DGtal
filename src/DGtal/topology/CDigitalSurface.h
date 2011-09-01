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
 * @file CDigitalSurface.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/05/17
 *
 * Header file for concept CDigitalSurface.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDigitalSurface_RECURSES)
#error Recursive header files inclusion detected in CDigitalSurface.h
#else // defined(CDigitalSurface_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDigitalSurface_RECURSES

#if !defined CDigitalSurface_h
/** Prevents repeated inclusion of headers. */
#define CDigitalSurface_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalSurface
  /**
   * Description of \b concept '\b CDigitalSurface' <p>
   * @ingroup Concepts
   * Aim: Describes what is a digital surface in cellular topology.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CDigitalSurface
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
  struct CDigitalSurface
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CDigitalSurface
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/CDigitalSurface.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDigitalSurface_h

#undef CDigitalSurface_RECURSES
#endif // else defined(CDigitalSurface_RECURSES)
