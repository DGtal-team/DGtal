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
 * @file CIncrementalMetric.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/01/23
 *
 * Header file for concept CIncrementalMetric.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CIncrementalMetric_RECURSES)
#error Recursive header files inclusion detected in CIncrementalMetric.h
#else // defined(CIncrementalMetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CIncrementalMetric_RECURSES

#if !defined CIncrementalMetric_h
/** Prevents repeated inclusion of headers. */
#define CIncrementalMetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CIncrementalMetric
  /**
     Description of \b concept '\b CIncrementalMetric' <p>
     @ingroup Concepts
     @brief Aim: Defines the concept describing a metric,  
    which can be incrementally computed, ie. computed at a point, 
    knowing the value of (some of) its 1-neighbors.  
     
   
     <p> Nested types : 
     - \t Point 
     - \t Value
  
     <p> Notation
     - \t X : a model of CIncrementalMetric
     - \t x : object of type X
     - \t p : object of type Point
     - \t m : object of type std::map<Point, Value>
  
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
        <td class=CExpression> x(p,m)     </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType> Value     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> computes the metric value at p, knowing the values of its 1-neigbors stored in m </td> 
        <td class=CPostCondition>       </td> 
        <td class=CComplexity>  logarithmic in the size of s    </td>
      </tr>
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>
     FirstOrderIncrementalMetric
    
     <p> Notes <br>

     @tparam X the type that should be a model of CIncrementalMetric.
   */
  template <typename X> 
  struct CIncrementalMetric : boost::CopyConstructible<X> 
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // Inner types
    typedef typename X::Point Point;
    typedef typename X::Value Value;
    // Methods
    BOOST_CONCEPT_USAGE( CIncrementalMetric )
    {
      ConceptUtils::sameType( myV, myX.operator()(myP, myM) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    X myX; 
    Point myP; 
    Value myV; 
    std::map<Point, Value> myM; 
  
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CIncrementalMetric
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CIncrementalMetric_h

#undef CIncrementalMetric_RECURSES
#endif // else defined(CIncrementalMetric_RECURSES)
