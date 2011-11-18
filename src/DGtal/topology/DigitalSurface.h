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
 * @file DigitalSurface.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2011/09/01
 *
 * Header file for module DigitalSurface.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSurface_RECURSES)
#error Recursive header files inclusion detected in DigitalSurface.h
#else // defined(DigitalSurface_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurface_RECURSES

#if !defined DigitalSurface_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurface_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalSurface
  /**
     Description of template class 'DigitalSurface' <p>

     \brief Aim: Represents a set of n-1-cells in a nD space, together
     with adjacency relation between these cells.

     Proxy class to a DigitalSurfaceContainer.

     @todo Should be a model of CSinglePassConstRange

     @tparam TDigitalSurfaceContainer any model of
     CDigitalSurfaceContainer: the concrete representation chosen for
     the digital surface.
   */
  template <typename TDigitalSurfaceContainer>
  class DigitalSurface
  {

    // ----------------------- types ------------------------------
  public:
    typedef DigitalSurface<TDigitalSurfaceContainer> Self;
    typedef TDigitalSurfaceContainer DigitalSurfaceContainer;
    typedef typename DigitalSurfaceContainer::KSpace KSpace;
    typedef typename DigitalSurfaceContainer::Cell Cell;
    typedef typename DigitalSurfaceContainer::SCell SCell;
    typedef typename DigitalSurfaceContainer::Surfel Surfel;
    typedef typename DigitalSurfaceContainer::SurfelConstIterator ConstIterator;
    typedef typename DigitalSurfaceContainer::DigitalSurfaceTracker DigitalSurfaceTracker; 
    typedef typename KSpace::SurfelSet SurfelSet;
    /// Template rebinding for defining the type that is a mapping
    /// SCell -> Value.
    template <typename Value> struct SurfelMap {
      typedef typename KSpace::template SurfelMap<Value>::Type Type;
    };

    // ----------------------- UndirectedSimpleGraph --------------------------
  public:
    /// Defines the type for a vertex.
    typedef Surfel Vertex;
    /// Defines how to represent a size (unsigned integral type).
    typedef typename KSpace::Size Size;
    /// Defines how to represent a set of vertex.
    typedef typename KSpace::SurfelSet VertexSet;
    /// Template rebinding for defining the type that is a mapping
    /// SCell -> Value.
    template <typename Value> struct VertexMap {
      typedef typename KSpace::template SurfelMap<Value>::Type Type;
    };
    /**
       An edge is a unordered pair of vertices. To make comparisons
       easier, the smallest vertex is stored before the greatest
       vertex. Note that loops are legal.
    */ 
    struct Edge {
      /// The two vertices.
      Vertex vertices[ 2 ];
      /** 
          Constructor from vertices.
          @param v1 the first vertex.
          @param v2 the second vertex.
      */
      Edge( const Vertex & v1, const Vertex & v2 )
      {
        if ( v1 <= v2 ) 
          {
            vertices[ 0 ] = v1;
            vertices[ 1 ] = v2;
          }
        else
          {
            vertices[ 0 ] = v2;
            vertices[ 1 ] = v1;
          }
      }
      bool operator==( const Edge & other ) const
      {
        return ( vertices[ 0 ] == other.vertices[ 0 ] )
          && ( vertices[ 1 ] == other.vertices[ 1 ] );
      }
      bool operator<( const Edge & other ) const
      {
        return ( vertices[ 0 ] < other.vertices[ 0 ] )
          || ( ( vertices[ 0 ] == other.vertices[ 0 ] )
               && ( vertices[ 1 ] < other.vertices[ 1 ] ) );
      }

    };
    

    // ----------------------- Standard services ------------------------------
  public:
    
    /**
     * Destructor.
     */
    ~DigitalSurface();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DigitalSurface ( const DigitalSurface & other );

    /**
       Copy constructor from container.
       @param container the container to copy.
    */
    DigitalSurface( const DigitalSurfaceContainer & container );

    /**
       Constructor from pointer on a dynamically allocated container.
       @param containerPtr the pointer to acquire.
    */
    DigitalSurface( DigitalSurfaceContainer* containerPtr );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    DigitalSurface & operator= ( const DigitalSurface & other );

    // ----------------------- Services --------------------------------------
  public:

    /**
       @return a const reference to the stored container.
    */
    const DigitalSurfaceContainer & container() const;

    /**
       @return a reference to the container (may be duplicated if
       several DigitalSurface points on the same).
    */
    DigitalSurfaceContainer & container();

    // ----------------- UndirectedSimpleGraph realization --------------------
  public:
    
    /**
       @return a ConstIterator on the first surfel in the container.
    */
    ConstIterator begin() const;

    /**
       @return a ConstIterator after the last surfel in the container.
    */
    ConstIterator end() const;

    /// @return the number of vertices of the graph.
    Size size() const;

    /**
       @param v any vertex of this graph
       @return the number of neighbors of this Vertex/Surfel.a
       @pre container().isInside( v )
    */
    Size degree( const Vertex & v ) const;

    /**
       Writes the neighbors of [v] in the output iterator
       [it]. Neighbors are given in no specific order.

       @tparam OutputIterator the type for the output iterator
       (e.g. back_insert_iterator<std::vector<Vertex> >).

       @param[in,out] it any output iterator on Vertex (*it++ should
       be allowed), which specifies where neighbors are written.

       @param[in] v any vertex of this graph

       @pre container().isInside( v )
    */
    template <typename OutputIterator>
    void writeNeighbors( OutputIterator & it,
                         const Vertex & v ) const;

    /**
       Writes the neighbors of [v], verifying the predicate [pred] in
       the output iterator [it]. Neighbors are given in no specific
       order.

       @tparam OutputIterator the type for the output iterator
       (e.g. back_insert_iterator<std::vector<Vertex> >).

       @tparam VertexPredicate any type of predicate taking a Vertex as input.
  
       @param[in,out] it any output iterator on Vertex (*it++ should
       be allowed), which specifies where neighbors are written.

       @param[in] v any vertex of this graph
       
       @param[in] pred the predicate for selecting neighbors.

       @pre container().isInside( v )
    */
    template <typename OutputIterator, typename VertexPredicate>
    void writeNeighbors( OutputIterator & it,
                         const Vertex & v,
                         const VertexPredicate & pred ) const;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:

    /// a smart pointer on the container.
    CountedPtr<DigitalSurfaceContainer> myContainer;
    /// a pointer on a tracker.
    mutable DigitalSurfaceTracker* myTracker;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DigitalSurface();

  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DigitalSurface


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurface'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurface' to write.
   * @return the output stream after the writing.
   */
  template <typename TDigitalSurfaceContainer>
  std::ostream&
  operator<< ( std::ostream & out, 
	       const DigitalSurface<TDigitalSurfaceContainer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/DigitalSurface.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurface_h

#undef DigitalSurface_RECURSES
#endif // else defined(DigitalSurface_RECURSES)
