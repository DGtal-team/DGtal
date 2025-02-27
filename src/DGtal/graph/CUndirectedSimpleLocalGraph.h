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
#include "DGtal/graph/CVertexMap.h"
#include "DGtal/graph/CVertexPredicate.h"
#include "DGtal/graph/CVertexPredicateArchetype.h"
//////////////////////////////////////////////////////////////////////////////

// @since 0.8 In DGtal::concepts
namespace DGtal { 
  namespace concepts {

    /////////////////////////////////////////////////////////////////////////////
    // class CUndirectedSimpleLocalGraph
    /**
    Description of \b concept '\b CUndirectedSimpleLocalGraph' <p>
    @ingroup Concepts
    @brief Aim: Represents the concept of local graph: each vertex has neighboring vertices, but we do not necessarily know all the vertices.

    # Refinement of

    # Associated types
    - Size: an integral type to count the number of vertices.
    - Vertex: the type for the vertices of the graph (a model of boost::DefaultConstructible, boost::Assignable, boost::CopyConstructible).
    - VertexSet: the type for storing a set of vertices.
    - VertexMap: a rebinding structure to associate Value to vertices of model CVertexMap.
    
    # Notation
    - \c X : A type that is a model of CUndirectedSimpleLocalGraph
    - \c x : object of type X
    - \c v : object of type Vertex
    - \c out : an output iterator of type OutputIterator (model of boost::OutputIterator).
    - \c p : a predicate on vertex of type VertexPredicate (model of CVertexPredicate).
    
    # Definitions
    
    # Valid expressions and semantics 
     
    | Name          | Expression | Type requirements   | Return type | Precondition     | Semantics | Post condition | Complexity |
    |---------------|------------|---------------------|-------------|------------------|-----------|----------------|------------|
    | Degree        | x.degree(v)|                     | Size        |                  |Returns the degree of vertex \c v | | |
    |Capacity       | x.bestCapacity() |               | Size        |                  |Returns the approximate number of neighbors to be expected. Useful to prepare data structures. | | |      
    |Neighbourhood | x.writeNeighbors<OutputIterator>( out, v ) | | | | Writes with the output iterator \c out the neighboring vertices of \c v. | | |
    | Neighborhood | x.writeNeighbors<OutputIterator,VertexPredicate>( out, v, p ) | | | | Writes with the output iterator \c out the neighboring vertices of \c v that satisfy the predicate \c p. | | |  
            
    # Invariants#
    
    # Models#
    - DigitalSurface, LightImplicitDigitalSurface, LightExplicitDigitalSurface
    - Object, MetricAdjacency, DomainAdjacency

    # Notes#

    @tparam T the type that should be a model of CUndirectedSimpleLocalGraph.
    */
    template <typename T>
    concept CUndirectedSimpleLocalGraph = 
      std::default_initializable<typename T::Vertex> &&
      std::copy_constructible<typename T::Vertex> &&
      std::is_copy_assignable_v<typename T::Vertex> && 
      CIntegralNumber<typename T::Size> && 
      requires(
        const T myX, const typename T::Vertex myVertex, 
        const CVertexPredicateArchetype<typename T::Vertex> myVPred,
        const boost::output_iterator_archetype<typename T::Vertex> myOutIt
      )
      {
          { myX.bestCapacity() } -> std::same_as<typename T::Size>;
          { myX.degree(myVertex) } -> std::same_as<typename T::Size>;
          myX.writeNeighbors( myOutIt, myVertex );
          myX.writeNeighbors( myOutIt, myVertex, myVPred );
      };
  } // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CUndirectedSimpleLocalGraph_h

#undef CUndirectedSimpleLocalGraph_RECURSES
#endif // else defined(CUndirectedSimpleLocalGraph_RECURSES)
