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
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CUnsignedInteger.h"
#include "DGtal/kernel/NumberTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CSpace
  /**
Description of \b concept '\b CSpace' <p>
     @ingroup Concepts
    
     \brief Aim: Defines the concept describing a digital space, ie a
     cartesian product of integer lines.
     
 ### Refinement of
    
 ### Provided types :

     - Space: the type itself.
     - Integer: the type for the components or coordinates of the
       space. Must be a model of CInteger.
     - Point: the type for a point in this space.
     - Vector: the type for a vector in this space.
     - Dimension: the type for the dimension in this space. Must be a
       model of CUnsignedInteger
     - Size: the type for measuring distances or counting elements in
       this space. Must be a model of CUnsignedInteger

 ### Notation
     - \t X : A type that is a model of CSpace
     - \t x, \t y  : Object of type X
    
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
     <td class=CName>            \t X should have a static \c dimension. </td>
     <td class=CExpression>      \t x.dimension </td> 
     <td class=CRequirements>    static member has type \t Dimension </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td> 
     <td class=CSemantics>       </td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     </table>
    
 ### Invariants###
    
 ### Models###
    
 ### Notes###

@tparam T the type that is checked. T should be a model of
     CSpace.

   */
  template <typename T>
  struct CSpace
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::Space Space;
    typedef typename T::Integer Integer;
    BOOST_CONCEPT_ASSERT(( CInteger< Integer > ));
    typedef typename T::Point Point;
    typedef typename T::Vector Vector;
    typedef typename T::Dimension Dimension;
    BOOST_CONCEPT_ASSERT(( CUnsignedInteger< Dimension > ));
    typedef typename T::Size Size;
    BOOST_CONCEPT_ASSERT(( CUnsignedInteger< Size > ));
    BOOST_CONCEPT_USAGE( CSpace )
    {
      //Should have a static dimension.
      ConceptUtils::sameType( myDim, T::dimension );
    }

    // ------------------------- Private Datas --------------------------------
  private:
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
