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
 * @file DigitalSurfaceBoostGraphInterface.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2013/01/20
 *
 * Header file for module DigitalSurfaceBoostGraphInterface.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSurfaceBoostGraphInterface_RECURSES)
#error Recursive header files inclusion detected in DigitalSurfaceBoostGraphInterface.h
#else // defined(DigitalSurfaceBoostGraphInterface_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurfaceBoostGraphInterface_RECURSES

#if !defined DigitalSurfaceBoostGraphInterface_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurfaceBoostGraphInterface_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/topology/DigitalSurface.h"
//////////////////////////////////////////////////////////////////////////////


// The interface to the Boost Graph should be defined in namespace boost.
namespace boost
{
  /**
     This is the kind of boost graph that a digital surface can mimick.
  */
  struct DigitalSurface_graph_traversal_category 
    : public virtual adjacency_graph_tag,
      public virtual vertex_list_graph_tag,
      public virtual incidence_graph_tag { };

  /**
     Defines the graph traits for any kind of digital surface.
     
     @tparam TDigitalSurfaceContainer the container chosen for the
     digital surface. Should work with DigitalSetBoundary,
     SetOfSurfels, ExplicitDigitalSurface,
     ImplicitDigitalSurface. Light* containers may not work since
     their vertex iterator is not multipass.
  */
  template < class TDigitalSurfaceContainer >
  struct graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > > 
  {
    /// the adapted DGtal graph class.
    typedef DGtal::DigitalSurface< TDigitalSurfaceContainer > Adapted;
    /// the graph is undirected.
    typedef undirected_tag directed_category;
    /// the graph satisfies AdjacencyListGraph and VertexListGraph concepts.
    typedef DigitalSurface_graph_traversal_category traversal_category;
    /// the graph does not allow parallel edges.
    typedef disallow_parallel_edge_tag edge_parallel_category;

    /// the type for counting vertices
    typedef typename Adapted::Size vertices_size_type;
    /// the type for counting edges
    typedef typename Adapted::Size edges_size_type;
    /// the type for counting out or in edges
    typedef typename Adapted::Size degree_size_type;

    

    typedef typename Adapted::Vertex Vertex;
    /// Vertex type
    typedef Vertex vertex_descriptor;
    typedef typename Adapted::Arc Arc;
    /// Edge type
    typedef Arc edge_descriptor;
    /// Iterator for visiting vertices (should be multipass).
    typedef typename Adapted::ConstIterator vertex_iterator;
    //typedef typename std::set<Vertex>::const_iterator vertex_iterator;

    /// This is the intermediate data structure that is used for visiting adjacent vertices.
    typedef std::vector< vertex_descriptor > AdjacentVertexContainer;
    /// This is the intermediate data structure that is used for storing out edges.
    typedef typename Adapted::ArcRange OutEdgeContainer;

  public:
    /// Iterator for visiting adjacent vertices.
    class adjacency_iterator 
      : public iterator_facade< adjacency_iterator,
                                Vertex,
                                bidirectional_traversal_tag,
                                const Vertex & >
    {
    public:
      adjacency_iterator() 
        : myIterator(), myVertices( 0 ) {}
      adjacency_iterator( typename AdjacentVertexContainer::const_iterator it,
                          const DGtal::CountedPtr< AdjacentVertexContainer > & vertices )
        : myIterator( it ), myVertices( vertices ) {}
    private:
      const Vertex & dereference() const { return *myIterator; }

      bool equal(const adjacency_iterator& other) const
      { return myIterator == other.myIterator; }

      void increment() { ++myIterator; }
      void decrement() { --myIterator; }

      /// A counted pointer to the dynamically allocated container of
      /// vertices. Will be automatically deallocated when there is no
      /// more iterators pointing on it.
      DGtal::CountedPtr< AdjacentVertexContainer > myVertices;
      /// The iterator pointing in the container of adjacent vertices.
      typename AdjacentVertexContainer::const_iterator myIterator;

      friend class iterator_core_access;
    }; // end class adjacency_iterator 

    /// Iterator for visiting out edges.
    class out_edge_iterator 
      : public iterator_facade< out_edge_iterator,
                                Arc,
                                bidirectional_traversal_tag,
                                const Arc & >
    {
    public:
      out_edge_iterator() 
        : myIterator(), myOutEdges( 0 ) {}
      out_edge_iterator( typename OutEdgeContainer::const_iterator it,
                         const DGtal::CountedPtr< OutEdgeContainer > & out_edges )
        : myIterator( it ), myOutEdges( out_edges ) {}
    private:
      const Arc & dereference() const { return *myIterator; }

      bool equal(const out_edge_iterator & other) const
      { return myIterator == other.myIterator; }

      void increment() { ++myIterator; }
      void decrement() { --myIterator; }

      /// A counted pointer to the dynamically allocated container of
      /// out edges. Will be automatically deallocated when there is no
      /// more iterators pointing on it.
      DGtal::CountedPtr< OutEdgeContainer > myOutEdges;
      /// The iterator pointing in the container of out edges.
      typename OutEdgeContainer::const_iterator myIterator;

      friend class iterator_core_access;
    }; // end class out_edge_iterator

  }; // end struct graph_traits< >

  /**
     Defines the property map traits for any kind of digital surface.
     
     @tparam TDigitalSurfaceContainer the container chosen for the
     digital surface. 
  */
  // template < class TDigitalSurfaceContainer, typename TPropertyTag >
  // struct property_map< DGtal::DigitalSurface< TDigitalSurfaceContainer >, TPropertyTag >
  // {
    
  // }; 


  /////////////////////////////////////////////////////////////////////////////
  // Functions for the boost graph interface to DigitalSurface<TDigitalSurfaceContainer>.

  /**
     @param edge an arc (s,t) on \a digSurf.
     @param digSurf a valid digital surface.
     @param the vertex s.
  */
  template < class TDigitalSurfaceContainer >
  inline 
  typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_descriptor
  source( typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::edge_descriptor edge,
          const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf )
  {
    return digSurf.tail( edge );
  }
  /**
     @param edge an arc (s,t) on \a digSurf.
     @param digSurf a valid digital surface.
     @param the vertex t.
  */
  template < class TDigitalSurfaceContainer >
  inline 
  typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_descriptor
  target( typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::edge_descriptor edge,
          const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf )
  {
    return digSurf.head( edge );
  }
  
  /**
     @param digSurf a valid digital surface.
     @return a pair< vertex_iterator, vertex_iterator > that
     represents a range to visit all the vertices of \a digSurf.
  */
  template < class TDigitalSurfaceContainer >
  std::pair<
    typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_iterator,
    typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_iterator 
    >
  vertices( const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf );

  /**
     @param digSurf a valid digital surface.
     @return the invalid vertex for \a digSurf (default Vertex).
  */
  template < class TDigitalSurfaceContainer >
  inline 
  typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_descriptor
  null_vertex( const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf )
  {
    return typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::Vertex();
  }

  /**
     @param digSurf a valid digital surface.
     @return the number of vertices of \a digSurf.
  */
  template < class TDigitalSurfaceContainer >
  inline 
  typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertices_size_type
  num_vertices( const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf )
  {
    return digSurf.size();
  }
  
  /**
     @param u a vertex belonging to \a digSurf.
     @param digSurf a valid digital surface.
     @return a pair< adjacency_iterator, adjacency_iterator > that
     represents a range to visit the adjacent vertices of vertex \a
     u.
  */
  template < class TDigitalSurfaceContainer >
  inline 
  std::pair<
    typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::adjacency_iterator,
    typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::adjacency_iterator 
    >
  adjacent_vertices( typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_descriptor u,
                     const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf )
  {
    typedef typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >
      ::adjacency_iterator Iterator;
    typedef typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >
      ::AdjacentVertexContainer Container;
    DGtal::CountedPtr<Container> ptrAdjVertices( new Container );
    std::back_insert_iterator< Container > outIt = std::back_inserter( *ptrAdjVertices );
    digSurf.writeNeighbors( outIt, u );
    return std::make_pair( Iterator( ptrAdjVertices->begin(), ptrAdjVertices ),
                           Iterator( ptrAdjVertices->end(), ptrAdjVertices ) );
  }

  // namespace detail {
  //   template < class TDigitalSurfaceContainer >
  //   struct OutEdgeBackInsertIterator 
  //     : public iterator<output_iterator_tag,void,void,void,void> 
  //   {
  //     typedef DGtal::DigitalSurface< TDigitalSurfaceContainer > Graph;
  //     typedef OutEdgeBackInsertIterator< TDigitalSurfaceContainer > Self;

  //     typedef typename graph_traits< Graph >::vertex_descriptor vertex_descriptor;
  //     typedef typename graph_traits< Graph >::edge_descriptor edge_descriptor;
  //     typedef typename graph_traits< Graph >::out_edge_iterator out_edge_iterator;
  //     typedef typename graph_traits< Graph >::OutEdgeContainer OutEdgeContainer;

  //     typedef OutEdgeContainer container_type;

  //     inline
  //     OutEdgeBackInsertIterator( ConstAlias< Graph > graph, ConstAlias< OutEdgeContainer > container )
  //       : myGraph( graph ), myContainer( container )
  //     {}
  //     Self & operator=( const vertex_descriptor & t )
  //     { container->push_back(value); return *this; }

  //   };
  // }

  /**
     @param u a vertex belonging to \a digSurf.
     @param digSurf a valid digital surface.

     @return a pair< out_edge_iterator, out_edge_iterator > that
     represents a range to visit the out edges of vertex \a u. Each
     out edge is a tuple (u,t) of vertices, where t != u.
  */
  template < class TDigitalSurfaceContainer >
  inline 
  std::pair<
    typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::out_edge_iterator,
    typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::out_edge_iterator 
    >
  out_edges( typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_descriptor u,
             const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf )
  {
    typedef typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >
      ::out_edge_iterator Iterator;
    typedef typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >
      ::OutEdgeContainer Container;
    DGtal::CountedPtr<Container> ptrOutEdges( new Container( digSurf.outArcs( u ) ) );
    return std::make_pair( Iterator( ptrOutEdges->begin(), ptrOutEdges ),
                           Iterator( ptrOutEdges->end(), ptrOutEdges ) );
  }

  /**
     @param u a vertex belonging to \a digSurf.
     @param digSurf a valid digital surface.

     @return the number of out edges at vertex \a u. Each out edge is
     a tuple (u,t) of vertices, where t != u.
  */
  template < class TDigitalSurfaceContainer >
  inline 
  typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::degree_size_type
  out_degree( typename graph_traits< DGtal::DigitalSurface< TDigitalSurfaceContainer > >::vertex_descriptor u,
              const DGtal::DigitalSurface< TDigitalSurfaceContainer > & digSurf )
  {
    return digSurf.degree( u );
  }
  
} // namespace Boost


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/graph/DigitalSurfaceBoostGraphInterface.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurfaceBoostGraphInterface_h

#undef DigitalSurfaceBoostGraphInterface_RECURSES
#endif // else defined(DigitalSurfaceBoostGraphInterface_RECURSES)
