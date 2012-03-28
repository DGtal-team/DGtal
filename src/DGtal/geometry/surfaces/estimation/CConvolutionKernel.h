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
 * @file CConvolutionKernel.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/06
 *
 * Header file for concept CConvolutionKernel.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CConvolutionKernel_RECURSES)
#error Recursive header files inclusion detected in CConvolutionKernel.h
#else // defined(CConvolutionKernel_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CConvolutionKernel_RECURSES

#if !defined CConvolutionKernel_h
/** Prevents repeated inclusion of headers. */
#define CConvolutionKernel_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CConvolutionKernel
  /**
Description of \b concept '\b CConvolutionKernel' <p>
     @ingroup Concepts
     @brief Aim: defines models of centered convolution kernel used for normal vector integration for instance.

     CConvolutionKernel models are functor mappings displacement vectors to real values.
     
     
 ### Refinement of CopyConstructible, Assignable
    
 ### Associated types : Vector
    
 ### Notation
     - \t X : A type that is a model of CConvolutionKernel
     - \t x, \t y : object of type X
    
 ### Definitions
    
 ### Valid expressions and 
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
        <td class=CName> Apply function           </td> 
        <td class=CExpression>  x(v)     </td>
        <td class=CRequirements> v of type const  Vector&    </td> 
        <td class=CReturnType> double     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics>  the value of the kernel at @e v     </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity> O(1)     </td>
      </tr>
    
     </table>
    
 ### Invariants###
    
 ### Models###

     ConstantConvolutionKernel, GaussianConvolutionKernel

 ### Notes###

@tparam T the type that should be a model of CConvolutionKernel.
   */
  template <typename T> 
  struct CConvolutionKernel:  boost::CopyConstructible<T>, boost::Assignable<T>
  // Use derivation for coarser concepts, like
  // : CoarserConcept<T>
  // Think to boost::CopyConstructible<T>, boost::DefaultConstructible<T>, ...
  // http://www.boost.org/doc/libs/1_49_0/libs/concept_check/reference.htm
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::Vector Vector;
   
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CConvolutionKernel )
    {

      ConceptUtils::sameType( myB, myX( myA ) );
    }
  
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // do not require T to be default constructible.
    typename T::Vector myA;
    double myB;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CConvolutionKernel
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CConvolutionKernel_h

#undef CConvolutionKernel_RECURSES
#endif // else defined(CConvolutionKernel_RECURSES)
