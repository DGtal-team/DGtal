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
 * @file CCellularConvolutionKernel.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/06
 *
 * Header file for concept CCellularConvolutionKernel.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CCellularConvolutionKernel_RECURSES)
#error Recursive header files inclusion detected in CCellularConvolutionKernel.h
#else // defined(CCellularConvolutionKernel_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CCellularConvolutionKernel_RECURSES

#if !defined CCellularConvolutionKernel_h
/** Prevents repeated inclusion of headers. */
#define CCellularConvolutionKernel_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CCellularConvolutionKernel
  /**
     Description of \b concept '\b CCellularConvolutionKernel' <p>
     @ingroup Concepts
     @brief Aim: defines models of centered convolution kernel used for normal vector integration for instance.

     CCellularConvolutionKernel models are functor mappings displacement vectors to real values.
     
     
 ### Refinement of CopyConstructible, Assignable
    
 ### Associated types : 
  
- Quantity: type for convolution values
- KSpace: type of underlying cellular space
- ConstIterator: iterator on cellular support
    
 ### Notation
     - \a X : A type that is a model of CCellularConvolutionKernel
     - \a x, \a y : object of type X
      - \a quantity: object of type Quantity
 ### Valid expressions and semantics
    | Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
    |-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
    |       |            |                   |               |              |           |                |            |
	
get value | quantity = x( const SCell &aScell) | return the value functor at \a aScell  
            KSpace::Spel x.center() | returns the support center (
            x.begin()
            x.end()
	      
 ### Invariants 
    
 ### Models 
 
 ### Notes
 
     @tparam T the type that should be a model of CCellularConvolutionKernel.
   */
  template <typename T> 
  struct CCellularConvolutionKernel
  // Use derivation for coarser concepts, like
  // : CoarserConcept<T>
  // Think to boost::CopyConstructible<T>, boost::DefaultConstructible<T>, ...
  // http://www.boost.org/doc/libs/1_49_0/libs/concept_check/reference.htm
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::Quantity Quantity;
    typedef typename T::KSpace KSpace;
    typedef typename T::ConstIterator ConstIterator;
    
    
    
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CCellularConvolutionKernel )
    {
      ConceptUtils::sameType( myQ, myX( mySCell ) );
    }
  
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // do not require T to be default constructible.
    typename KSpace::SCell mySCell;
    Quantity myQ;
    ConstIterator myIt;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CCellularConvolutionKernel
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CCellularConvolutionKernel_h

#undef CCellularConvolutionKernel_RECURSES
#endif // else defined(CCellularConvolutionKernel_RECURSES)
