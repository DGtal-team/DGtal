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
 * @file CRange.h
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/08/31
 *
 * Header file for concept CRange
 *
 * This file is part of the DGtal library.
 */

#if defined(CRange_RECURSES)
#error Recursive header files inclusion detected in CRange.h
#else // defined(CRange_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CRange_RECURSES

#if !defined CRange_h
/** Prevents repeated inclusion of headers. */
#define CRange_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CRange
  /**
     Description of \b concept '\b CRange' <p>
     @ingroup Concepts
    
     \brief Aim: Defines the concept describing a const range.
     
     <p> Refinement of CConstRange
    
     <p> Provided types :

     - Iterator: the iterator type, a model of iterator concept.
     - ReverseIterator: the reverse iterator type, a model of
          iterator concept.

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
    
     <p> Invariants <br>
    
     <p> Models <br>
    
     <p> Notes <br>

     @tparam T the type that is checked. T should be a model of CRange.

   */
  template <typename T>
  struct CRange : public CConstRange<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::Iterator Iterator;
    typedef typename T::ReverseIterator ReverseIterator;

    BOOST_CONCEPT_ASSERT(( boost_concepts::SinglePassIteratorConcept<Iterator> ));
    BOOST_CONCEPT_ASSERT(( boost_concepts::SinglePassIteratorConcept<ReverseIterator> ));
 
    BOOST_CONCEPT_USAGE(CRange)
    {
      Iterator it=i.begin();
      it=i.end();
      ReverseIterator it2=i.rbegin();
      it2=i.rend();
    };

  private:
    T i;
  }; // end of concept CRange
  
} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CRange_h

#undef CRange_RECURSES
#endif // else defined(CRange_RECURSES)
