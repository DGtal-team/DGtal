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
 * @file CSpace.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/02
 *
 * Header file for concept CSpace.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CSpace_RECURSES)
#error Recursive header files inclusion detected in CSpace.h
#else // defined(CSpace_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSpace_RECURSES

#if !defined CSpace_h
/** Prevents repeated inclusion of headers. */
#define CSpace_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/utils/ConceptUtils.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CUnsignedInteger.h"
#include "DGtal/kernel/IntegerTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CSpace
  /**
   * Description of \b concept '\b CSpace' <p>
   * @ingroup Concepts
   *
   * \brief Aim: Defines the concept describing a digital space, ie a
   * cartesian product of integer lines.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CSpace
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
  struct CSpace
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::Integer Integer;
    BOOST_CONCEPT_ASSERT(( CInteger< Integer > ));
    typedef typename T::Space Space;
    typedef typename T::Point Point;
    typedef typename T::Vector Vector;
    typedef typename T::Dimension Dimension;
    BOOST_CONCEPT_ASSERT(( CUnsignedInteger< Dimension > ));
    typedef typename T::Size Size;
    BOOST_CONCEPT_USAGE( CSpace )
    {
      // Should have a dimension() method.
      myX.dimension();
      // This method shoud return the correct type.
      ConceptUtils::sameType( myDim, myX.dimension() );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myX;
    Dimension myDim;

    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CSpace
  
} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSpace_h

#undef CSpace_RECURSES
#endif // else defined(CSpace_RECURSES)
