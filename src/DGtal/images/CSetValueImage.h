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
 * @file CSetValueImage.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CSetValueImageRECURSES)
#error Recursive header files inclusion detected in CSetValueImage.h
#else // defined(CSetValueImageRECURSES)
/** Prevents recursive inclusion of headers. */
#define CSetValueImageRECURSES

#if !defined CSetValueImage_h
/** Prevents repeated inclusion of headers. */
#define CSetValueImage_h

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/images/CConstImage.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // struct CSetValueImage
  /**
   * Description of \b concept '\b CSetValueImage' <p>
   *
   * @ingroup Concepts
   * Aim: Defines the concept describing a read/write image, 
   * which is a refinement of a read-only image. 
   *
   * <p> Refinement of CConstImage
   *
   * <p> Associated types : the same as CConstImage
   *
   * <p> Notation
   * - \t X : A type that is a model of CSetValueImage
   * - \t x, \t y  : Object of type X
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
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
      <td class=CName> Set a value           </td> 
      <td class=CExpression>  x.setValue(@c aPoint, @c aValue)    </td>
      <td class=CRequirements> @c aPoint of type Point and @c aValue of
      type Value   </td> 
      <td class=CReturnType>  void    </td>
      <td class=CPrecondition> @c aPoint must be valid (inside the image domain)  </td> 
      <td class=CSemantics>  associate the value @c aValue with the
      point  @aPoint     </td> 
      <td class=CPostCondition>   </td> 
      <td class=CComplexity>  Container dependent    </td>
      </tr>
        

    </table>   

   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   * ImageContainerBySTLVector, ImageContainerBySTLMap, ImageContainerByITKImage
   * <p> Notes <br>
   *
   */

  template <typename I>
  struct CSetValueImage: CConstImage<I>
  {

  public:
  
    BOOST_CONCEPT_USAGE(CSetValueImage)
    {
      myI.setValue(myP, myV);  //set a value v at p
    }

  private:
    I myI;
    typename I::Value myV;
    typename I::Point myP;
  };
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSetValueImage_h

#undef CSetValueImageRECURSES
#endif // else defined(CSetValueImageRECURSES)
