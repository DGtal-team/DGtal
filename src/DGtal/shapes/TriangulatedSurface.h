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
 * @file TriangulatedSurface.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/05
 *
 * Header file for module TriangulatedSurface.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(TriangulatedSurface_RECURSES)
#error Recursive header files inclusion detected in TriangulatedSurface.h
#else // defined(TriangulatedSurface_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TriangulatedSurface_RECURSES

#if !defined TriangulatedSurface_h
/** Prevents repeated inclusion of headers. */
#define TriangulatedSurface_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <set>
#include <map>
#include "DGtal/base/Common.h"
#include "DGtal/topology/HalfEdgeDataStructure.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class TriangulatedSurface
  /**
   * Description of template class 'TriangulatedSurface' <p> \brief
   * Aim: Represents a triangulated surface. The topology is stored
   * with a half-edge data structure. Geometry and other data can be
   * stored within this object, and may be specialised with template
   * parameters TVertexData.
   *
   * For now, the user must add vertices and triangles, and when
   * finished, call 'build()'.
   *
   * Model of CUndirectedSimpleLocalGraph: the vertices and edges of the
   * triangulated surface form indeed a graph structure.
   *
   * @tparam TVertexData a type that must have an attribute \a
   * position (a RealPoint) and an attribute \a flags.
   *
   * @see HalfEdgeDataStructure
   *
   * @todo Model of CUndirectedSimpleGraph, because we could provide a
   * trivial iterator that enumerates integer from 0 to the number of
   * vertices.
   */
  template <typename TVertexData>
  class TriangulatedSurface
  {
  public:
    typedef TVertexData                        VertexData;
    typedef TriangulatedSurface<TVertexData>   Self;
    typedef HalfEdgeDataStructure::Size        Size;
    typedef HalfEdgeDataStructure::Index       Index;
    typedef HalfEdgeDataStructure::VertexIndex VertexIndex;
    typedef HalfEdgeDataStructure::EdgeIndex   EdgeIndex;
    typedef HalfEdgeDataStructure::FaceIndex   FaceIndex;
    typedef HalfEdgeDataStructure::Triangle    Triangle;
    typedef std::vector<VertexData>            VertexDataStorage;
    typedef std::vector<Triangle>              TriangleStorage;

    // Required by CUndirectedSimpleLocalGraph
    typedef VertexIndex                        Vertex;
    typedef std::set<Vertex>                   VertexSet; 
    template <typename Value> struct           VertexMap {
      typedef typename std::map<Vertex, Value> Type;
    };
    
    // Required for CCombinatorialSurface
    typedef HalfEdgeDataStructure::HalfEdgeIndex Arc;
    typedef HalfEdgeDataStructure::FaceIndex     Face;
    typedef std::vector<Arc>                     ArcRange;
    typedef std::vector<Face>                    FaceRange;
    typedef std::vector<Vertex>                  VertexRange;
    typedef std::set<Face>                       FaceSet;

    BOOST_STATIC_CONSTANT( Face, INVALID_FACE = -1 );
    
  protected:
    typedef HalfEdgeDataStructure::HalfEdge      HalfEdge;
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~TriangulatedSurface() {}

    /**
     * Constructor.
     */
    TriangulatedSurface() : isHEDSValid( false ) {}

    /// Builds the half-edge data structure from the given triangles
    /// and vertices. After that, the surface is valid.
    ///
    /// @return true if everything went allright, false if it was not
    /// possible to build a consistent data structure (e.g., butterfly
    /// neighborhoods).
    bool build();
    
    /// Adds a new vertex to the surface with data \a vdata.
    /// @param vdata the data associated to this new vertex.
    /// @return the new index given to this vertex.
    VertexIndex addVertex( const VertexData& vdata );
    
    /// Adds a new triangle of vertices \a v0, \a v1, \a v2 to the surface.
    /// @return the corresponding index of the triangle.
    FaceIndex addTriangle( VertexIndex v0, VertexIndex v1, VertexIndex v2 );

    /// Mutable accessor to vertex data.
    /// @param v any vertex.
    /// @return the mutable data associated to \a v.
    VertexData& vData();
    /// Const accessor to vertex data.
    /// @param v any vertex.
    /// @return the non-mutable data associated to \a v.
    const VertexData& vData() const;
    
    // ----------------------- Undirected simple graph services -------------------------
  public:
    /**
     * @Return the number of vertices of the surface.
     */
    Size size() const;
    
    /**
     * @return an estimate of the maximum number of neighbors for this adjacency
     *
     * @note chosen here to be 8. Number of neighbors is 6 on average
     * for planar triangulations.
     */
    Size bestCapacity() const;
    
    /**
     * @param v any vertex
     * 
     * @return the number of neighbors of this vertex
     */
    Size degree( const Vertex & v ) const;
    
    /**
     * Writes the neighbors of a vertex using an output iterator
     * 
     * 
     * @tparam OutputIterator the type of an output iterator writing
     * in a container of vertices.
     * 
     * @param it the output iterator
     * 
     * @param v the vertex whose neighbors will be writen
     */
    template <typename OutputIterator>
    void  
    writeNeighbors( OutputIterator &it ,
		    const Vertex & v ) const;
    
    /**
     * Writes the neighbors of a vertex which satisfy a predicate using an 
     * output iterator
     * 
     * 
     * @tparam OutputIterator the type of an output iterator writing
     * in a container of vertices.
     * 
     * @tparam VertexPredicate the type of the predicate
     * 
     * @param it the output iterator
     * 
     * @param v the vertex whose neighbors will be written
     * 
     * @param pred the predicate that must be satisfied
     */
    template <typename OutputIterator, typename VertexPredicate>
    void
    writeNeighbors( OutputIterator &it ,
		    const Vertex & v,
		    const VertexPredicate & pred) const;

    // ----------------------- CombinatorialSurface --------------------------
  public:

    /**
       @param v any vertex of the surface.
       @return the outgoing arcs from [v]
    */
    ArcRange outArcs( const Vertex & v ) const;

    /**
       @param v any vertex of the surface.
       @return the ingoing arcs to [v]
    */
    ArcRange inArcs( const Vertex & v ) const;

    /**
       @param v any vertex of the surface.
       @return the faces containing this vertex [v].
    */
    FaceRange facesAroundVertex( const Vertex & v ) const;

    /**
      @param a any arc (s,t)
      @return the vertex t
    */
    Vertex head( const Arc & a ) const;

    /**
      @param a any arc (s,t)
      @return the vertex s
    */
    Vertex tail( const Arc & a ) const;

    /**
       @param a any arc (s,t)
       @return the arc (t,s)
    */
    Arc opposite( const Arc & a ) const;

    /**
       [tail] and [head] should be adjacent vertices.
       
       @param tail the vertex at the tail of the arc.
       @param head the vertex at the head of the arc.
       @return the arc (tail, head)
    */
    Arc arc( const Vertex & tail, const Vertex & head ) const;

    /**
       Convenience method for computing the face incident to a given
       arc. There is at most one face that borders an arc for
       triangulated surface since it is a combinatorial 2-manifold.

       @param a any arc on the surface.

       @return the face around the given arc or the InvalidFace if the
       arc was a boundary arc.
    */
    Face faceAroundArc( const Arc & a ) const;
    
    /**
       Computes the faces incident to a given arc. There is only one
       for triangulated surface since it is a combinatorial
       2-manifold.

       @param a any arc on the surface.
       @return a vector containing the faces incident to this arc.
    */
    FaceRange facesAroundArc( const Arc & a ) const;

    /**
       If f is incident to the arcs (s,t) and (t,u) (say), then
       (s,t,u) is a subsequence of the returned sequence.

       @param f any valid face on the digital surface (open or closed ).

       @return the sequence of vertices that touches this face. The
       order follows the order of incident arcs (the range size should be 3).
    */
    VertexRange verticesAroundFace( const Face & f ) const;

    /**
       @param v any vertex.
       @return 'true' if and only if vertex \a v lies on a boundary.
       @note O(1) operation
    */
    bool isVertexBoundary( const Vertex& v ) const;

    /**
       @param a any arc.
       @return 'true' if and only if arc \a a lies on a boundary (note
       that the opposite arc does not lie on the boundary).
       @note O(1) operation
    */
    bool isArcBoundary( const Arc& v ) const;
    
    /**
       @return the set of all faces of the triangulated surface.
    */
    FaceSet allFaces() const;

    /**
       This set of arcs is sufficient for displaying the boundary of
       the surface.

       @return the set of all arcs (oriented edges) lying on the
       boundary of the surface (in no particular order).
    */
    ArcRange allBoundaryArcs() const;

    /**
       This set of arcs is sufficient for displaying the boundary of
       the surface.

       @return the set of vertices lying on the boundary of the
       surface (in no particular order).
    */
    VertexRange allBoundaryVertices() const;

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
  protected:
    /// Indicates if the half-edge structure has been created/updated.
    bool isHEDSValid;
    /// The half-edge data structure that stores the topology of the mesh
    HalfEdgeDataStructure myHEDS;
    /// Stores the information for each Vertex.
    VertexDataStorage myVertexDatas;
    /// Stores the triangles.
    TriangleStorage myTriangles;
    
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

  private:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class TriangulatedSurface


  /**
   * Overloads 'operator<<' for displaying objects of class 'TriangulatedSurface'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'TriangulatedSurface' to write.
   * @return the output stream after the writing.
   */
  template <typename TVertexData>
  std::ostream&
  operator<< ( std::ostream & out, const TriangulatedSurface<TVertexData> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/TriangulatedSurface.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TriangulatedSurface_h

#undef TriangulatedSurface_RECURSES
#endif // else defined(TriangulatedSurface_RECURSES)
