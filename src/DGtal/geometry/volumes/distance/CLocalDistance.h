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
 * @file CLocalDistance.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/01/23
 *
 * Header file for concept CLocalDistance.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CLocalDistance_RECURSES)
#error Recursive header files inclusion detected in CLocalDistance.h
#else // defined(CLocalDistance_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLocalDistance_RECURSES

#if !defined CLocalDistance_h
/** Prevents repeated inclusion of headers. */
#define CLocalDistance_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/images/CImage.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CLocalDistance
  /**
     Description of \b concept '\b CLocalDistance' <p>
     @ingroup Concepts
     @brief Aim: Defines the concept describing a local distance 
    mapping, which can compute the distance at a given point, 
    knowing the distance value of some points in its neighborhood.   

     <p> Refinement : 
     - \t boost::Assignable
   
     <p> Nested types : 
     - \t Image : a model of CImage storing the known distance values 
  
     <p> Notation
     - \t X : a model of CLocalDistance
     - \t x : object of type X
  
     <p> Definitions
    
     <p> Valid expressions and semantics <br>
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
        <td class=CName> main operator  </td> 
        <td class=CExpression> x(image, set, point)     </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> Image::Value     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> 
Computes the distance value at 'point' (of type Image::Point), 
knowing the distance values stored in 'image' (of type Image)
of the neighborhood points belonging to 'set' (an instance of 
any model of CDigitalSet whose inner type Point is equal to 
Image::Point) </td> 
        <td class=CPostCondition>       </td> 
        <td class=CComplexity>   </td>
      </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>
     L2FirstOrderLocalDistance, L1FirstOrderLocalDistance, LInfFirstOrderLocalDistance
    
     <p> Notes <br>

     @tparam X the type that should be a model of CLocalDistance.
   */
  template <typename X> 
  struct CLocalDistance : boost::Assignable<X> 
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // Inner types
    typedef typename X::Image Image;
    BOOST_CONCEPT_ASSERT(( CImage <Image> ));
 
    // Methods
    BOOST_CONCEPT_USAGE( CLocalDistance )
    {
      ConceptUtils::sameType( myV, myX.operator()(myImg, mySet, myPoint) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    X myX;
    Image myImg;  
    typename Image::Point myPoint; 
    typename Image::Value myV; 
    DigitalSetBySTLSet<typename Image::Domain> mySet; 
  
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CLocalDistance
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLocalDistance_h

#undef CLocalDistance_RECURSES
#endif // else defined(CLocalDistance_RECURSES)
