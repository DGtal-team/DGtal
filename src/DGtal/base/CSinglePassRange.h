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
 * @file CSinglePassRange.h
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/08/31
 *
 * Header file for concept CSinglePassRange
 *
 * This file is part of the DGtal library.
 */

#if defined(CSinglePassRange_RECURSES)
#error Recursive header files inclusion detected in CSinglePassRange.h
#else // defined(CSinglePassRange_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSinglePassRange_RECURSES

#if !defined CSinglePassRange_h
/** Prevents repeated inclusion of headers. */
#define CSinglePassRange_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CSinglePassRange
  /**
     Description of \b concept '\b CSinglePassRange' <p>
     @ingroup Concepts
    
     \brief Aim: Defines the concept describing a range.
     
     <p> Refinement of CSinglePassConstRange
    
     <p> Provided types:

     - Iterator: the iterator type, a model of iterator concept
          (see boost concept SinglePassIteratorConcept).

     <p> Notation:

     - x an object of a model of CSinglePassConstRange.
     
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
     <td class=CName>            \t begin </td>
     <td class=CExpression>      \t x.begin() </td> 
     <td class=CRequirements>    </td>
     <td class=CReturnType>      Iterator</td>
     <td class=CPrecondition>    </td> 
     <td class=CSemantics>       </td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     <tr> 
     <td class=CName>            \t end </td>
     <td class=CExpression>      \t x.end() </td> 
     <td class=CRequirements>    </td>
     <td class=CReturnType>      Iterator</td>
     <td class=CPrecondition>    </td> 
     <td class=CSemantics>       </td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>
    
     <p> Notes <br>

     @tparam T the type that is checked. T should be a model of CSinglePassRange.

   */
  template <typename T>
  struct CSinglePassRange : public CSinglePassConstRange<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
   typedef typename T::Iterator Iterator;

    BOOST_CONCEPT_ASSERT(( boost_concepts::SinglePassIteratorConcept<Iterator> ));
 
    BOOST_CONCEPT_USAGE(CSinglePassRange)
    {
      Iterator it=i.begin();
      it2=i.end();
    };

  private:
    T i;
  }; // end of concept CSinglePassRange
  
} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSinglePassRange_h

#undef CSinglePassRange_RECURSES
#endif // else defined(CSinglePassRange_RECURSES)
