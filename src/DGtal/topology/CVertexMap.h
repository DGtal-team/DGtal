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
 * @file CVertexMap.h
 * @author Jérémy Gaillard (\c jeremy.gaillard@insa-lyon.fr )
 * Institut National des Sciences Appliquées - INSA, France
 *
 * @date 2012/07/11
 *
 * Header file for concept CVertexMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CVertexMap_RECURSES)
#error Recursive header files inclusion detected in CVertexMap.h
#else // defined(CVertexMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CVertexMap_RECURSES

#if !defined CVertexMap_h
/** Prevents repeated inclusion of headers. */
#define CVertexMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CPredicate.h"
#include "boost/concept_check.hpp"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CVertexMap
  /**
    *  Description of \b concept '\b CVertexMap' <p>
    * @ingroup Concepts
    * \brief Aim: Defines a map of vertices.
    * 
    * Associates values to vertices.
    
 ### Associated types :
    * - Vertex : specifies the type for an element of the domain (inner
    *   type).
    * - Value : specifies the type for a value (inner type).

     
###  Notation
    * - \t X : A type that is a model of CVertexMap
    * - \t x : Object of type X
    * - \t v : Object of type Vertex
    * - \t val : Object of type Value

 ### Definitions
 
 
 ###  Valid expressions and
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
        <td class=CName> set value </td>
        <td class=CExpression> x.setValue(v, val) </td>
        <td class=CRequirements>  </td>
        <td class=CReturnType>  </td>
        <td class=CPrecondition>  </td>
        <td class=CSemantics> set a given value val to a given vertex v </td>
        <td class=CPostCondition>  </td>
        <td class=CComplexity>  </td>
      </tr>
      
      <tr>
        <td class=CName> access value </td>
        <td class=CExpression> x(v) </td>
        <td class=CRequirements>  </td>
        <td class=CReturnType> val </td>
        <td class=CPrecondition>  </td>
        <td class=CSemantics> the value at vertex v </td>
        <td class=CPostCondition>  </td>
        <td class=CComplexity>  </td>
      </tr>


    </table>
    
###  Invariants
   *

###  Models
   * ImageContainerBySTLVector, ImageContainerBySTLMap, ImageContainerByITKImage, ImageContainerByHashTree
        
 ### Notes###
   */
  template <typename T>
  struct CVertexMap:
    boost::Assignable<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::Vertex Vertex;
    typedef typename T::Value Value;
    
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CVertexMap )
    {
      ConceptUtils::sameType( myValue, myX.operator()(myVertex) );
      myX.setValue(myVertex, myValue);
    }
    
    // ------------------------- Private Datas --------------------------------
  private:
    T myX;
    Vertex myVertex;
    Value myValue;
    
  }; // end of concept CVertexMap
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CVertexMap_h

#undef CVertexMap_RECURSES
#endif // else defined(CVertexMap_RECURSES)
