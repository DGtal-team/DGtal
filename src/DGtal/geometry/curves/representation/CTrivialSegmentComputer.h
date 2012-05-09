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
 * @file CTrivialSegmentComputer.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/08/31
 *
 * Header file for concept CTrivialSegmentComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CTrivialSegmentComputer_RECURSES)
#error Recursive header files inclusion detected in CTrivialSegmentComputer.h
#else // defined(CTrivialSegmentComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CTrivialSegmentComputer_RECURSES

#if !defined CTrivialSegmentComputer_h
/** Prevents repeated inclusion of headers. */
#define CTrivialSegmentComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/representation/CSegment.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CTrivialSegmentComputer
  /**
Description of \b concept '\b CTrivialSegmentComputer' <p>
     @ingroup Concepts
     @brief Aim: Defines the concept describing the most trivial segment computer,  
    ie. a model of CSegment that can extend itself. 
     
 ### Refinement of CSegment 
    
 ### Associated types : 
    the same as CSegment
  
 ### Notation
     - \t X : A type that is a model of CTrivialSegmentComputer
     - \t x, \t y : object of type X
     - \t i : object of type X::ConstIterator
  
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
        <td class=CName> initialization  </td> 
        <td class=CExpression> x.init(i)     </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> void     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> set a segment to i      </td> 
        <td class=CPostCondition>       </td> 
        <td class=CComplexity> O(1)     </td>
      </tr>
      <tr> 
        <td class=CName> extension test  </td> 
        <td class=CExpression> x.isExtendableForward()     </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> bool     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> checks whether x can be extended to x.end() or not </td> 
        <td class=CPostCondition>       </td> 
        <td class=CComplexity>      </td>
      </tr>
      <tr> 
        <td class=CName> extension </td> 
        <td class=CExpression> x.extendForward()     </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> bool     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> check whether x can be extended to x.end() or not, extends if true </td> 
        <td class=CPostCondition>       </td> 
        <td class=CComplexity>     </td>
      </tr>
     </table>
    
 ### Invariants###
    
 ### Models###
    
 ### Notes###

@tparam T the type that should be a model of CTrivialSegmentComputer.
   */
  template <typename T> 
  struct CTrivialSegmentComputer : CSegment<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:

    // Methods
    BOOST_CONCEPT_USAGE( CTrivialSegmentComputer )
    {
      typename T::ConstIterator i(myI);
      myX.init(myI);     

      ConceptUtils::sameType( myB, myX.isExtendableForward() );
      ConceptUtils::sameType( myB, myX.extendForward() );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // only if T is default constructible.
    typename T::ConstIterator myI;
    bool myB; 
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CTrivialSegmentComputer
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CTrivialSegmentComputer_h

#undef CTrivialSegmentComputer_RECURSES
#endif // else defined(CTrivialSegmentComputer_RECURSES)
