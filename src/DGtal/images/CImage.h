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
 * @file CImage.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CImageRECURSES)
#error Recursive header files inclusion detected in CImage.h
#else // defined(CImageRECURSES)
/** Prevents recursive inclusion of headers. */
#define CImageRECURSES

#if !defined CImage_h
/** Prevents repeated inclusion of headers. */
#define CImage_h

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/images/CConstImage.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // struct CImage
  /**
   * Description of \b concept '\b CImage' <p>
   *
   * @ingroup Concepts
   * Aim: Defines the concept describing a read/write image, 
   * having an output iterator. 
   *
   * <p> Refinement of CSetValueImage
   *
   * <p> Associated types : the same as CSetValueImage +
   * - \t OutputIterator : type of the output iterator
   *
   * <p> Notation
   * - \t X : A type that is a model of CImage
   * - \t x : Object of type X
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
      <td class=CName> Provide an output iterator           </td> 
      <td class=CExpression>  x.outputIterator()    </td>
      <td class=CRequirements>        </td> 
      <td class=CReturnType>  an instance of OutputIterator    </td>
      <td class=CPrecondition>   </td> 
      <td class=CSemantics>       </td> 
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
  struct CImage: CConstImage<I>
  {

  typedef typename I::OutputIterator O; 

  public:
  
    BOOST_CONCEPT_USAGE(CImage)
    {
      ConceptUtils::sameType( myO, myI.outputIterator() ); //output iterator 
    }

  private:
    I myI;
    O myO; 
  };
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CImage_h

#undef CImageRECURSES
#endif // else defined(CImageRECURSES)
