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
 * @file CUndirectedSimpleLocalGraph.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/01
 *
 * Header file for concept CUndirectedSimpleLocalGraph.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CUndirectedSimpleLocalGraph_RECURSES)
#error Recursive header files inclusion detected in CUndirectedSimpleLocalGraph.h
#else // defined(CUndirectedSimpleLocalGraph_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CUndirectedSimpleLocalGraph_RECURSES

#if !defined CUndirectedSimpleLocalGraph_h
/** Prevents repeated inclusion of headers. */
#define CUndirectedSimpleLocalGraph_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <boost/concept_archetype.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CUndirectedSimpleLocalGraph
  /**
     Description of \b concept '\b CUndirectedSimpleLocalGraph' <p>
     @ingroup Concepts
     @brief Aim:
     
     <p> Refinement of
    
     <p> Associated types :
    
     <p> Notation
     - \c X : A type that is a model of CUndirectedSimpleLocalGraph
     - \c x, \c y : object of type X
    
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
        <td class=CName>            </td> 
        <td class=CExpression>      </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType>      </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics>       </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>      </td>
      </tr>
    
     </table>
    
     <p> Invariants <br>
    
     <p> Models <br>

     A dummy model (for concept checking) is CCUndirectedSimpleLocalGraphArchetype.

     <p> Notes <br>

     @tparam T the type that should be a model of CUndirectedSimpleLocalGraph.
   */
  template <typename T> 
  struct CUndirectedSimpleLocalGraph 
  // Use derivation for coarser concepts, like
  // : CoarserConcept<T>
  // Think to boost::CopyConstructible<T>, boost::DefaultConstructible<T>, ...
  // http://www.boost.org/doc/libs/1_49_0/libs/concept_check/reference.htm
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::Vertex Vertex;
    typedef typename T::Size Size;
    typedef typename T::VertexSet VertexSet;
    template <typename Value> struct VertexMap {
      typedef typename T::template VertexMap<Value>::Type Type;
    };
 
    // possibly check these types so as to satisfy a concept with
    BOOST_CONCEPT_ASSERT(( CInteger< Size > ));

    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CUndirectedSimpleLocalGraph )
    {
      // check const methods.
      checkConstConstraints();
    }
    void checkConstConstraints() const
    {
      ConceptUtils::sameType( mySize, myX.bestCapacity() );
      ConceptUtils::sameType( mySize, myX.degree( myVertex ) );
      myX.writeNeighbors( myOutIt, myVertex );
      // @todo create VertexPredicate to test the other writeNeighbors method.
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // do not require T to be default constructible.
    Size mySize;
    Vertex myVertex;
    mutable boost::output_iterator_archetype<Vertex> myOutIt;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CUndirectedSimpleLocalGraph
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CUndirectedSimpleLocalGraph_h

#undef CUndirectedSimpleLocalGraph_RECURSES
#endif // else defined(CUndirectedSimpleLocalGraph_RECURSES)
