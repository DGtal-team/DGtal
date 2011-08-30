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
 * @file CBoundedInteger.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/01
 *
 * Header file for concept CBoundedInteger.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CBoundedInteger_RECURSES)
#error Recursive header files inclusion detected in CBoundedInteger.h
#else // defined(CBoundedInteger_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CBoundedInteger_RECURSES

#if !defined CBoundedInteger_h
/** Prevents repeated inclusion of headers. */
#define CBoundedInteger_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/utils/ConceptUtils.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/NumberTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CBoundedInteger
  /**
   * Description of \b concept '\b CBoundedInteger' <p>     
   * @ingroup Concepts
   *
   * \brief Aim: The concept CBoundedInteger specifies what are the bounded 
   * integer numbers. Hence, it is a refinement of CInteger Concept
   * ensuring that the numbers are bounded.
   * 
   * <p> Refinement of CInteger<T>
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CBoundedInteger
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table>
   * <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr> 
   * <td> \c X should be tagged \b true in \c NumberTraits for \c IsBounded.</td>
   * <td> typename NumberTraits<X>::IsBounded </td> 
   * <td> TagTrue </td>
   * <td> </td>
   * <td> </td> 
   * <td> </td>
   * <td> </td> 
   * <td> </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br> 
   * 
   * short, int, unsigned int, long long, unsigned long long,
   * uint16_t, uint32_t, uint64_t, int16_t, int32_t, int64_t.
   *
   * <p> Notes <br>
   *
   * @tparam T the type that is checked. T should be a model of
   * CBoundedInteger.
   */
  template <typename T>
  struct CBoundedInteger : public CInteger<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    BOOST_CONCEPT_USAGE(CBoundedInteger)
    {
      
      // Bounded Integer should have a TagTrue tag to IsBounded type.
      ConceptUtils::checkTrue( myIsBounded );
  
    }
      
    // ------------------------- Private Datas --------------------------------
  private:
    T myX;
    typename NumberTraits<T>::IsBounded myIsBounded;
   
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CBoundedInteger
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CBoundedInteger_h

#undef CBoundedInteger_RECURSES
#endif // else defined(CBoundedInteger_RECURSES)
