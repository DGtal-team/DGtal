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
 * @file CIncrementalMetricHelper.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/01/23
 *
 * Header file for concept CIncrementalMetricHelper.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CIncrementalMetricHelper_RECURSES)
#error Recursive header files inclusion detected in CIncrementalMetricHelper.h
#else // defined(CIncrementalMetricHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CIncrementalMetricHelper_RECURSES

#if !defined CIncrementalMetricHelper_h
/** Prevents repeated inclusion of headers. */
#define CIncrementalMetricHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <boost/array.hpp>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CIncrementalMetricHelper
  /**
     Description of \b concept '\b CIncrementalMetricHelper' <p>
     @ingroup Concepts
     @brief Aim: Defines the concept describing a helper,  
    providing some services for an incremental metric. 
    
    @see CIncrementalMetric.  

     <p> Refinement : 
     - \t boost::CopyConstructible
   
     <p> Nested types : 
     - \t Dimension  the dimension type
     - \t Value  the type used to store the metric
  
     <p> Notation
     - \t X : a model of CIncrementalMetricHelper
     - \t x : object of type X
     - \t d : object of type Dimension
     - \t a : object of type boost::aray<Value,d>
  
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
        <td class=CName> computation method  </td> 
        <td class=CExpression> compute(a)     </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> Value     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> computes a metric value from the values containing in a </td> 
        <td class=CPostCondition>       </td> 
        <td class=CComplexity>  constant for a fixed d    </td>
      </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>
     L2FirstOrderIncrementalMetric
     LInfinityFirstOrderIncrementalMetric
    
     <p> Notes <br>

     @tparam X the type that should be a model of CIncrementalMetricHelper.
   */
  template <typename X> 
  struct CIncrementalMetricHelper : boost::CopyConstructible<X> 
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // Inner types
    typedef typename X::Dimension Dimension;
    typedef typename X::Value Value;
    // Methods
    BOOST_CONCEPT_USAGE( CIncrementalMetricHelper )
    {
      ConceptUtils::sameType( myV, myX.compute(myA) );
      ConceptUtils::sameType( myV, myX.unknownValue() );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    X myX; 
    Value myV; 
    static const Dimension myD = X::dimension; 
    boost::array<Value,myD> myA; 
  
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CIncrementalMetricHelper
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CIncrementalMetricHelper_h

#undef CIncrementalMetricHelper_RECURSES
#endif // else defined(CIncrementalMetricHelper_RECURSES)
