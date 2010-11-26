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
 * @file CValueType.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/01
 *
 * Header file for concept CValueType.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CValueType_RECURSES)
#error Recursive header files inclusion detected in CValueType.h
#else // defined(CValueType_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CValueType_RECURSES

#if !defined CValueType_h
/** Prevents repeated inclusion of headers. */
#define CValueType_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CValueType
  /**
   * Description of \b concept '\b CValueType' <p>
   * @ingroup Concepts
   *
   * Aim: Represents a ValueType.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CValueType
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
    struct CValueType  : boost::Assignable<T>, boost::EqualityComparable<T>, boost::DefaultConstructible<T>
    {
      // ----------------------- Concept checks ------------------------------
    public:
    
      // ------------------------- Private Datas --------------------------------
    private:
    
      // ------------------------- Internals ------------------------------------
    private:
    
    }; // end of concept CValueType
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/images/CValueType.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CValueType_h

#undef CValueType_RECURSES
#endif // else defined(CValueType_RECURSES)
