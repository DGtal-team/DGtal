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
 * @file CForwardSegmentComputer.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/08/31
 *
 * Header file for concept CForwardSegmentComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CForwardSegmentComputer_RECURSES)
#error Recursive header files inclusion detected in CForwardSegmentComputer.h
#else // defined(CForwardSegmentComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CForwardSegmentComputer_RECURSES

#if !defined CForwardSegmentComputer_h
/** Prevents repeated inclusion of headers. */
#define CForwardSegmentComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/representation/CTrivialSegmentComputer.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CForwardSegmentComputer
  /**
Description of \b concept '\b CForwardSegmentComputer' <p>
     @ingroup Concepts
     @brief Aim: Defines the concept describing a forward segment computer,  
    ie. a model of CSegment that can extend itself (in the direction that is relative to 
    the underlying iterator). 
     
 ### Refinement of CTrivialSegmentComputer 
    
 ### Associated types : the same as CTrivialSegmentComputer +
    - Reverse, same as Self but using std::reverse_iterator<Self::ConstIterator>
    instead of Self::ConstIterator as the underlying iterator
  
 ### Notation
     - \t X : A type that is a model of CForwardSegmentComputer
     - \t x : object of type X
     - \t r : object of type X::Reverse
  
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
        <td class=CName> conversion  </td> 
        <td class=CExpression> x.getReverse()     </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> X::Reverse     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> returns an instance of Reverse, which is constructed from the same input parameters used to construct x (if any) </td> 
        <td class=CPostCondition>       </td> 
        <td class=CComplexity>      </td>
      </tr>
     </table>
    
 ### Invariants###
    
 ### Models###
    
 ### Notes###

@tparam T the type that should be a model of CForwardSegmentComputer.
   */
  template <typename T> 
  struct CForwardSegmentComputer : CTrivialSegmentComputer<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // Inner types
    typedef typename T::Reverse Reverse;
    // Methods
    BOOST_CONCEPT_USAGE( CForwardSegmentComputer )
    {
      ConceptUtils::sameType( myRx, myX.getReverse() );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // only if T is default constructible.
    Reverse myRx; 
  
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CForwardSegmentComputer
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CForwardSegmentComputer_h

#undef CForwardSegmentComputer_RECURSES
#endif // else defined(CForwardSegmentComputer_RECURSES)
