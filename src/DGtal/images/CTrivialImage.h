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
 * @file CTrivialImage.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CConstImageRECURSES)
#error Recursive header files inclusion detected in CTrivialImage.h
#else // defined(CConstImageRECURSES)
/** Prevents recursive inclusion of headers. */
#define CConstImageRECURSES

#if !defined CTrivialImage_h
/** Prevents repeated inclusion of headers. */
#define CTrivialImage_h

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

#include "DGtal/images/CTrivialConstImage.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // struct CTrivialImage
  /**
   * Description of \b concept '\b CTrivialImage' <p>
   *
   * @ingroup Concepts
   * Aim: Defines the concept describing an image without extra ranges, 
   * which is a refinement of CTrivialConstImage. 
   *
###  Refinement of CTrivialConstImage
   *
###  Associated types : the same as CTrivialConstImage
   *
###  Notation
   * - \t X : A type that is a model of CTrivialImage
   * - \t x, \t y  : Object of type X
   * - \t p, \t v : Objects of type Point and Value
   *
###  Definitions
   *
###  Valid expressions and 
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
        <td class=CName> set value           </td> 
        <td class=CExpression>  x.setValue(p,v)   </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType>      </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics>  set a given value v to a given point p     </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>      </td>
      </tr>


    </table>   

   *
###  Invariants
   *
###  Models
   * ImageContainerBySTLVector, ImageContainerBySTLMap, ImageContainerByITKImage, ImageContainerByHashTree
   *
   */

  template <typename I>
  struct CTrivialImage: CTrivialConstImage<I>
  {

   public:

    BOOST_CONCEPT_USAGE(CTrivialImage)
    {
      myI.setValue( myPoint, myValue ); 
    }

   private:
    I myI;
    typename I::Point myPoint; 
    typename I::Value myValue; 
    
  };
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CTrivialImage_h

#undef CConstImageRECURSES
#endif // else defined(CConstImageRECURSES)
