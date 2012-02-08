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
 * @file CConstSinglePassRange.h
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/08/31
 *
 * Header file for concept CConstSinglePassRange
 *
 * This file is part of the DGtal library.
 */

#if defined(CConstSinglePassRange_RECURSES)
#error Recursive header files inclusion detected in CConstSinglePassRange.h
#else // defined(CConstSinglePassRange_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CConstSinglePassRange_RECURSES

#if !defined CConstSinglePassRange_h
/** Prevents repeated inclusion of headers. */
#define CConstSinglePassRange_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CConstSinglePassRange
  /**
     Description of \b concept '\b CConstSinglePassRange' <p>
     @ingroup Concepts
    
     \brief Aim: Defines the concept describing a const range.
     
     <p> Refinement of
    
     <p> Provided types :

     - ConstIterator: the const iterator type, a model of const iterator
          concept (see boost concept SinglePassIteratorConcept).

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
     <td class=CReturnType>      ConstIterator</td>
     <td class=CPrecondition>    </td> 
     <td class=CSemantics>       </td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     <tr> 
     <td class=CName>            \t end </td>
     <td class=CExpression>      \t x.end() </td> 
     <td class=CRequirements>    </td>
     <td class=CReturnType>      ConstIterator</td>
     <td class=CPrecondition>    </td> 
     <td class=CSemantics>       </td> 
     <td class=CPostCondition>   </td> 
     <td class=CComplexity>      </td>
     </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>    

   */
  template <typename T>
  struct CConstSinglePassRange
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::ConstIterator ConstIterator;
    
    BOOST_CONCEPT_ASSERT(( boost_concepts::SinglePassIteratorConcept<ConstIterator> ));

    BOOST_CONCEPT_USAGE(CConstSinglePassRange)
    {
      ConstIterator it=i.begin();
      it=i.end();
    };

  private:
    T i;
  }; // end of concept CConstSinglePassRange
  
} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CConstSinglePassRange_h

#undef CConstSinglePassRange_RECURSES
#endif // else defined(CConstSinglePassRange_RECURSES)
