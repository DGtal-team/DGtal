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
 * @file HalfEdgeDataStructure.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/03
 *
 * Header file for module HalfEdgeDataStructure.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HalfEdgeDataStructure_RECURSES)
#error Recursive header files inclusion detected in HalfEdgeDataStructure.h
#else // defined(HalfEdgeDataStructure_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HalfEdgeDataStructure_RECURSES

#if !defined HalfEdgeDataStructure_h
/** Prevents repeated inclusion of headers. */
#define HalfEdgeDataStructure_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <array>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /// Defines the invalid half-edge (i.e. exterior).
  ///
  /// @note It is defined external to the HalfEdgeDataStructure
  /// because there was link error with gcc 4.8.4 for all variants
  /// within HalfEdgeDataStructure (e.g. BOOST_STATIC_CONSTANT, c++
  /// constexpr, c++ constexpr with const reference).
  static std::size_t const HALF_EDGE_INVALID_INDEX = std::numeric_limits<std::size_t>::max();
  
  /////////////////////////////////////////////////////////////////////////////
  // class HalfEdgeDataStructure
  /**
   * Description of template class 'HalfEdgeDataStructure' <p> \brief
   * Aim: This class represents an half-edge data structure, which is
   * a structure for representing the topology of a combinatorial
   * 2-dimensional surface or an embedding of a planar graph in the
   * plane. It does not store any geometry. As a minimal example,
   * these lines of code build two triangles connected by the edge
   * {1,2}.
   *
   * \code
   * std::vector< HalfEdgeDataStructure::Triangle > triangles( 2 );
   * triangles[0].v = { 0, 1, 2 };
   * triangles[1].v = { 2, 1, 3 };
   * HalfEdgeDataStructure mesh;
   * mesh.build( triangles );
   * std::cout << mesh << std::endl;
   * \endcode
   *
   * @note Large parts of this class are taken from
   * https://github.com/yig/halfedge, written by Yotam Gingold.
   */
  class HalfEdgeDataStructure
  {
  public:

    /// The type for counting elements.
    typedef std::size_t    Size;
    /// The type used for numbering half-edges (an offset an the half-edges structure).
    typedef std::size_t    Index;
    /// The type used for numbering half-edges (alias)
    typedef Index   HalfEdgeIndex;
    /// The type for numbering vertices
    typedef Index   VertexIndex;
    /// The type for numbering edges
    typedef Index   EdgeIndex;
    /// The type for numbering faces
    typedef Index   FaceIndex;

    // Defines the invalid index.
    // JOL: works with Apple LLVM version 8.1.0 (clang-802.0.42), but does not work with gcc 4.8.4
    //
    // static constexpr Index const& INVALID_INDEX = boost::integer_traits<Index>::const_max;
    //
    // JOL: does NOT work with Apple LLVM version 8.1.0 (clang-802.0.42)
    // (undefined reference at link time).
    // static constexpr Index const INVALID_INDEX = boost::integer_traits<Index>::const_max;
    // static Index const INVALID_INDEX = boost::integer_traits<Index>::const_max;
    // BOOST_STATIC_CONSTANT( Index, INVALID_INDEX = boost::integer_traits<Index>::const_max );
    
    typedef std::vector<HalfEdgeIndex> HalfEdgeIndexRange;
    typedef std::vector<VertexIndex>   VertexIndexRange;
    typedef std::vector<EdgeIndex>     EdgeIndexRange;
    typedef std::vector<FaceIndex>     FaceIndexRange;

    /// An arc is a directed edge from a first vertex to a second vertex.
    typedef std::pair<VertexIndex, VertexIndex> Arc;
    // A map from an arc (a std::pair of VertexIndex's) to its
    // half edge index (i.e. and offset into the 'halfedge' sequence).
    typedef std::map< Arc, Index > Arc2Index;
    // A map from an arc (a std::pair of VertexIndex's) to its face
    // index.
    typedef std::map< Arc, FaceIndex > Arc2FaceIndex;
    
    /// Represents an unoriented edge as two vertex indices, the first
    /// lower than the second.
    struct Edge
    {
      /// The two vertex indices.
      VertexIndex v[2];
      
      VertexIndex& start() { return v[0]; }
      const VertexIndex& start() const { return v[0]; }
      
      VertexIndex& end() { return v[1]; }
      const VertexIndex& end() const { return v[1]; }
      
      Edge()
      {
        v[0] = v[1] = -1;
      }
      Edge( VertexIndex vi, VertexIndex vj )
      {
        if ( vi <= vj ) { v[0] = vi; v[1] = vj; }
        else            { v[0] = vj; v[1] = vi; }
      }
      bool operator<( const Edge& other ) const
      {
        return ( start() < other.start() )
          || ( ( start() == other.start() ) && ( end() < other.end() ) );
      }
    };

    /// Represents an unoriented triangle as three vertices.
    struct Triangle
    {
      /// The three vertex indices.
      std::array<VertexIndex,3> v;
      
      VertexIndex& i() { return v[0]; }
      const VertexIndex& i() const { return v[0]; }
      
      VertexIndex& j() { return v[1]; }
      const VertexIndex& j() const { return v[1]; }
      
      VertexIndex& k() { return v[2]; }
      const VertexIndex& k() const { return v[2]; }
      
      Triangle()
      {
        v[0] = v[1] = v[2] = -1;
      }
      Triangle( VertexIndex v0, VertexIndex v1, VertexIndex v2 )
      {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
      }
    };

    /// Defines an arbitrary polygonal face as a vector of vertex
    /// indices. To be valid, its size must be at least 3.
    typedef std::vector<VertexIndex> PolygonalFace;
    
    /// The half edge structure. Each half-edge points to some Vertex,
    /// is incident to one face, lies on one undirected edge, has an
    /// opposite half-edge lying on the same edge, and has a next
    /// half-edge along the face.
    struct HalfEdge
    {
      /// The end vertex of this half-edge as an index into the vertex array.
      VertexIndex toVertex;
      /// Index into the face array.
      FaceIndex face;
      /// Index into the edges array.
      EdgeIndex edge;
      /// Index into the halfedges array.
      Index opposite;
      /// Index into the halfedges array.
      Index next;

      /// Default constructor. The half-edge is invalid.
      HalfEdge() :
        toVertex( HALF_EDGE_INVALID_INDEX ),
        face( HALF_EDGE_INVALID_INDEX ),
        edge( HALF_EDGE_INVALID_INDEX ),
        opposite( HALF_EDGE_INVALID_INDEX ),
        next( HALF_EDGE_INVALID_INDEX )
      {}
    };

  public:
    /// Default constructor. The data structure is empty. @see build
    HalfEdgeDataStructure() {}
    
    /** 
     * Computes all the unoriented edges of the given triangles. 
     *
     * @note Method build() needs the unordered edges of the mesh.  If
     * you don't have them, call this first.
     *
     * @param[in] triangles the vector of input oriented triangles.
     *
     * @param[out] edges_out the vector of all the unoriented edges of
     * the given triangles.
     *
     * @return the total number of different vertices (note that the
     * vertex numbering should be between 0 and this number minus
     * one).
     */
    static Size getUnorderedEdgesFromTriangles
    ( const std::vector<Triangle>& triangles, std::vector< Edge >& edges_out )
    {
      typedef std::set< Edge > EdgeSet;
      typedef std::set< VertexIndex > VertexIndexSet;
      VertexIndexSet vertexSet;
      EdgeSet edgeSet;
      for( const Triangle& T : triangles )
        {
          edgeSet.insert( Edge( T.i(), T.j() ) );
          edgeSet.insert( Edge( T.j(), T.k() ) );
          edgeSet.insert( Edge( T.k(), T.i() ) );
          vertexSet.insert( T.i() );
          vertexSet.insert( T.j() );
          vertexSet.insert( T.k() );
        }
      edges_out.resize( edgeSet.size() );
      Size e = 0;
      for ( const Edge& edge : edgeSet )
        {
          edges_out.at(e) = edge;
          ++e;
        }
      return vertexSet.size();
    }

    /** 
     * Computes all the unoriented edges of the given polygonal faces.
     *
     * @note Method build() needs the unordered edges of the mesh.  If
     * you don't have them, call this first.
     *
     * @param[in] polygonal_faces the vector of input oriented polygonal faces.
     *
     * @param[out] edges_out the vector of all the unoriented edges of
     * the given triangles.
     *
     * @return the total number of different vertices (note that the
     * vertex numbering should be between 0 and this number minus
     * one).
     */
    static Size getUnorderedEdgesFromPolygonalFaces
    ( const std::vector<PolygonalFace>& polygonal_faces, std::vector< Edge >& edges_out );
    
    /**
     * Builds the half-edge data structures from the given triangles
     * and edges.  It keeps the numbering of vertices given in the
     * input \a triangles as well as the numbering of triangles in the
     * vector \a triangles.
     *
     * @note Parameter \a edges can be computed from \a triangles by calling
     * getUnorderedEdgesFromTriangles() before.
     *
     * @note Both \a triangles and \a edges are not needed after the call to build()
     * completes and may be destroyed. 
     *
     * @param[in] num_vertices the number of vertices (one more than the
     * maximal vertex index).
     *
     * @param[in] triangles the vector of input triangles.
     * @param[in] edges the vector of input unoriented edges.
     *
     * @return 'true' if everything went well, 'false' if their was
     * error in the given topology (for instance, three triangles
     * sharing an edge).
     */
    bool build( const Size num_vertices, 
                const std::vector<Triangle>& triangles,
                const std::vector<Edge>&     edges );

    /**
     * Builds the half-edge data structures from the given polygonal faces
     * and edges.  It keeps the numbering of vertices given in the
     * input \a polygonal_faces as well as the numbering of faces in the
     * vector \a polygonal_faces.
     *
     * @note Parameter \a edges can be computed from \a polygonal_faces by calling
     * getUnorderedEdgesFromPolygonalFaces() before.
     *
     * @note Both \a polygonal_faces and \a edges are not needed after the call to build()
     * completes and may be destroyed. 
     *
     * @param[in] num_vertices the number of vertices (one more than the
     * maximal vertex index).
     *
     * @param[in] polygonal_faces the vector of input polygonal_faces.
     * @param[in] edges the vector of input unoriented edges.
     *
     * @return 'true' if everything went well, 'false' if their was
     * error in the given topology (for instance, three triangles
     * sharing an edge).
     */
    bool build( const Size                        num_vertices, 
                const std::vector<PolygonalFace>& polygonal_faces,
                const std::vector<Edge>&          edges );

    /**
     * Builds the half-edge data structure from the given triangles.
     * It keeps the numbering of vertices given in the input \a
     * triangles as well as the numbering of triangles in the vector
     * \a triangles.
     *
     * @param[in] triangles the vector of input triangles.
     */
    bool build( const std::vector<Triangle>& triangles )
    {
      std::vector<Edge> edges;
      const Size nbVtx = getUnorderedEdgesFromTriangles( triangles, edges );
      return build( nbVtx, triangles, edges );
    }

    /**
     * Builds the half-edge data structure from the given polygonal faces.
     * It keeps the numbering of vertices given in the input \a
     * polygonal_faces as well as the numbering of faces in the vector
     * \a polygonal_faces.
     *
     * @param[in] polygonal_faces the vector of input polygonal faces.
     */
    bool build( const std::vector<PolygonalFace>& polygonal_faces )
    {
      std::vector<Edge> edges;
      const Size nbVtx = getUnorderedEdgesFromPolygonalFaces( polygonal_faces, edges );
      return build( nbVtx, polygonal_faces, edges );
    }

    /// Clears the data structure.
    void clear()
    {
      myHalfEdges.clear();
      myVertexHalfEdges.clear();
      myFaceHalfEdges.clear();
      myEdgeHalfEdges.clear();
      myArc2Index.clear();
    }

    /// @return the number of half edges in the structure.
    Size nbHalfEdges() const { return myHalfEdges.size(); }

    /// @return the number of vertices in the structure.
    Size nbVertices() const { return myVertexHalfEdges.size(); }

    /// @return the number of unoriented edges in the structure.
    Size nbEdges() const { return myEdgeHalfEdges.size(); }
    
    /// @return the number of faces in the structure.
    Size nbFaces() const { return myFaceHalfEdges.size(); }

    /// @return the euler characteristic of the corresponding combinatorial mesh.
    long Euler() const
    { return (long) nbVertices() - (long) nbEdges() + (long) nbFaces(); }
    
    /// @param i any valid half-edge index.
    /// @return the half-edge of index \a i.
    const HalfEdge& halfEdge( const Index i ) const { return myHalfEdges.at( i ); }

    /// @param i any valid half-edge index.
    /// @return the corresponding directed edge as an arc (v(opp(i)), v(i)).
    Arc arcFromHalfEdgeIndex( const Index i ) const
    {
      const HalfEdge& he = myHalfEdges[ i ];
      return std::make_pair( myHalfEdges[ he.opposite ].toVertex, he.toVertex );
    }

    /// @param i the vertex index of some vertex.
    /// @param j the vertex index of some other vertex.
    /// @return the index of the half-edge from \a i to \a j or HALF_EDGE_INVALID_INDEX if not found.
    Index halfEdgeIndexFromArc( const VertexIndex i, const VertexIndex j ) const
    { return halfEdgeIndexFromArc( std::make_pair( i, j ) ); }
    
    /// @param arc any directed edge (i,j)
    /// @return the index of the half-edge from \a i to \a j or HALF_EDGE_INVALID_INDEX if not found.
    Index halfEdgeIndexFromArc( const Arc& arc ) const
    {
      auto result = myArc2Index.find( arc );
      return ( result == myArc2Index.end() ) ? HALF_EDGE_INVALID_INDEX : result->second;
    }

    /// @note In opposition with halfEdgeIndexFromArc, this method
    /// does not use the map myArc2Index to find the half-edge
    /// associated with arc (v1,v2), but rather gets an half-edge
    /// originating from v1, and turn around to find the one pointing
    /// to v2. This method is useful if you do not update myArc2Index
    /// when flipping.
    /// 
    /// @param i the vertex index of some vertex.
    /// @param j the vertex index of some other vertex.
    /// @return the index of the half-edge from \a i to \a j or HALF_EDGE_INVALID_INDEX if not found.
    Index findHalfEdgeIndexFromArc( const VertexIndex i, const VertexIndex j ) const
    { return findHalfEdgeIndexFromArc( std::make_pair( i, j ) ); }

    /// @note In opposition with halfEdgeIndexFromArc, this method
    /// does not use the map myArc2Index to find the half-edge
    /// associated with arc (v1,v2), but rather gets an half-edge
    /// originating from v1, and turn around to find the one pointing
    /// to v2. This method is useful if you do not update myArc2Index
    /// when flipping.
    /// 
    /// @param arc any directed edge (i,j)
    /// @return the index of the half-edge from \a i to \a j or HALF_EDGE_INVALID_INDEX if not found.
    Index findHalfEdgeIndexFromArc( const Arc& arc ) const
    {
      const Index s = halfEdgeIndexFromVertexIndex( arc.first );
      Index       i = s;
      do {
        const HalfEdge& he = myHalfEdges[ i ];
        if ( he.toVertex == arc.second ) return i;
        i = halfEdge( he.opposite ).next;
      } while ( i != s );
      return HALF_EDGE_INVALID_INDEX;
    }

    /// @param[in] vi any vertex index.
    /// @return the index of an half-edge originating from \a vi.
    Index halfEdgeIndexFromVertexIndex( const VertexIndex vi ) const
    { return myVertexHalfEdges[ vi ]; }

    /// @param[in] fi any face index.
    /// @return the index of an half-edge that borders the face \a fi.
    Index halfEdgeIndexFromFaceIndex( const FaceIndex fi ) const
    { return myFaceHalfEdges[ fi ]; }

    /// @param[in] ei any edge index.
    /// @return the index of an half-edge that borders the edge \a ei.
    Index halfEdgeIndexFromEdgeIndex( const EdgeIndex ei ) const
    { return myEdgeHalfEdges[ ei ]; }
    
    /// @param[in] vi any vertex index.
    /// @param[out] result the sequence of vertex neighbors of the given vertex \a vi (clockwise if triangles were given ccw).
    void getNeighboringVertices( const VertexIndex vi, VertexIndexRange& result ) const
    {
      result.clear();
      const Index start_hei = halfEdgeIndexFromVertexIndex( vi );
      Index hei = start_hei;
      do
        {
          const HalfEdge& he = halfEdge( hei );
          result.push_back( he.toVertex );
          hei = halfEdge( he.opposite ).next;
        }
      while ( hei != start_hei );
    }

    /// @param[in] vi any vertex index.
    /// @return the sequence of vertex neighbors of the given vertex \a vi (clockwise if triangles were given ccw).
    VertexIndexRange neighboringVertices( const VertexIndex vi ) const
    {
      VertexIndexRange result;
      getNeighboringVertices( vi, result );
      return result;
    }

    /// @param[in] vi any vertex index.
    /// @return the number of vertex neighbors to \a vi.
    Size nbNeighboringVertices( const Index vi ) const
    {
      Size nb = 0;
      const Index start_hei = halfEdgeIndexFromVertexIndex( vi );
      Index hei = start_hei;
      do
        {
          const HalfEdge& he = halfEdge( hei );
          nb++;
          hei = halfEdge( he.opposite ).next;
        }
      while ( hei != start_hei );
      return nb;
    }

    /// @param[in] vi any vertex index.
    /// @param[out] result the sequence of neighboring faces of the given vertex \a vi .
    void getNeighboringFaces( const VertexIndex vi, FaceIndexRange& result ) const
    {
      result.clear();
      const Index start_hei = halfEdgeIndexFromVertexIndex( vi );
      Index hei = start_hei;
      do
        {
          const HalfEdge& he = halfEdge( hei );
          if( HALF_EDGE_INVALID_INDEX != he.face ) result.push_back( he.face );
          hei = halfEdge( he.opposite ).next;
        }
      while ( hei != start_hei );
    }

    /// @param[in] vi any vertex index.
    /// @return the sequence of neighboring faces of the given vertex \a vi .
    FaceIndexRange neighboringFaces( const VertexIndex vi ) const
    {
      FaceIndexRange result;
      getNeighboringFaces( vi, result );
      return result;
    }

    /// @param[in] vi any vertex index.
    /// @return true if and only if the vertex \a vi lies on the boundary.
    bool isVertexBoundary( const VertexIndex vi ) const
    {
      return HALF_EDGE_INVALID_INDEX == halfEdge( myVertexHalfEdges[ vi ] ).face;
    }

    /// @return a sequence containing the indices of the vertices
    /// lying on the boundary.
    ///
    /// @note O(nb half-edges) operation.
    /// @note no particular order.
    VertexIndexRange boundaryVertices() const
    {
      VertexIndexRange result;
      // std::set< VertexIndex > result;
      for( Index hei = 0; hei < myHalfEdges.size(); ++hei )
        {
          const HalfEdge& he = halfEdge( hei );
          if( HALF_EDGE_INVALID_INDEX == he.face )
            result.push_back( he.toVertex );
        }
      return result;
    }

    /// @return a sequence containing the indices of half-edges  lying on the boundary.
    /// @note O(nb half-edges) operation.
    /// @note no particular order.
    std::vector< Index > boundaryHalfEdgeIndices() const
    {
      std::vector< Index > result;
      for( Index hei = 0; hei < myHalfEdges.size(); ++hei )
        {
          const HalfEdge& he = halfEdge( hei );
          if( HALF_EDGE_INVALID_INDEX == he.face )
            result.push_back( hei );
        }
      return result;
    }
    /// @return a sequence containing the arcs lying on the boundary.
    /// @note O(nb half-edges) operation.
    /// @note no particular order.
    std::vector< Arc > boundaryArcs() const
    {
        std::vector< Arc > result;
        for( Index hei = 0; hei < myHalfEdges.size(); ++hei )
          {
            const HalfEdge& he = halfEdge( hei );
            if( HALF_EDGE_INVALID_INDEX == he.face )
              result.push_back( arcFromHalfEdgeIndex( hei ) );
          }
        return result;
    }

    /// @param hei any valid half-edge index.
    /// @return the number of sides of the face that contains the half-edge \a hei.
    Size nbSides( Index hei ) const
    {
       ASSERT( hei != HALF_EDGE_INVALID_INDEX );
       Size nb = 0;
       const Index start = hei;
       do {
         hei = halfEdge( hei ).next;
         nb++;
       }
       while ( hei != start );
       return nb;
    }

    /// @param f any valid face index.
    /// @return the number of sides of the face \a f.
    Size nbSidesOfFace( const FaceIndex f ) const
    {
      return nbSides( halfEdgeIndexFromFaceIndex( f ) );
    }
    
    /// @param f any valid face index.
    /// @return the sequence of vertices of the face \a f.
    VertexIndexRange verticesOfFace( const FaceIndex f ) const
    {
      VertexIndexRange result;
      result.reserve( 3 );
      Index hei = halfEdgeIndexFromFaceIndex( f );
      const Index start = hei;
       do {
         const HalfEdge& he = halfEdge( hei );
         result.push_back( he.toVertex );
         hei = he.next;
       }
       while ( hei != start );
       return result;
    }

    // -------------------- Modification services -------------------------
  public:
    
    /// An edge is (topologically) flippable iff: (1) it does not lie
    /// on the boundary, (2) it is bordered by two triangles, (3) the
    /// two other vertices of the quad are not already neighbors.
    ///
    /// @param hei any valid half-edge index.
    /// @return 'true' if the edge containing \a hei is topologically flippable.
    ///
    /// @note Time complexity is O(1).
    bool isFlippable( const Index hei ) const
    {
      ASSERT( hei != HALF_EDGE_INVALID_INDEX );
      const HalfEdge&   he = halfEdge( hei );
      const Index     hei2 = he.opposite;
      const HalfEdge&  he2 = halfEdge( hei2 );
      // check if hei borders an infinite face.
      if ( he.face  == HALF_EDGE_INVALID_INDEX ) return false; 
      // check if hei2 borders an infinite face.
      if ( he2.face == HALF_EDGE_INVALID_INDEX ) return false; 
      // Checks that he1 and he2 border a triangle.
      if ( ( nbSides( hei ) != 3 ) || ( nbSides( hei2 ) != 3 ) ) return false;
      // Checks that the two other vertices of the two surrounding
      // triangles are not already neighbors
      const VertexIndex v1 = halfEdge( he.next  ).toVertex;
      const VertexIndex v2 = halfEdge( he2.next ).toVertex;
      const auto neighb_v1 = neighboringVertices( v1 );
      const auto     it_v2 = std::find( neighb_v1.cbegin(), neighb_v1.cend(), v2 );
      return it_v2 == neighb_v1.cend();
    }

    /// Tries to flip the edge containing \a hei.
    ///
    /// @param hei any valid half-edge index.
    ///
    /// @param update_arc2index (optimisation parameter), when 'true'
    /// updates everything consistently; when 'false' do not update
    /// myArc2Index map, which means you cannot get an half edge from
    /// two vertices afterwards. Use 'false' only if you know that you
    /// never use this mapping (e.g. no call to halfEdgeIndexFromArc)
    ///
    /// @pre the edge must be flippable, `isFlippable( hei ) == true`
    /// @see isFlippable
    ///
    /// @note Time complexity is O(1) if \a update_arc2index is false,
    /// otherwise it is O(log n) is n is the number of arcs.
    void flip( const Index hei, bool update_arc2index = true )
    {
      const Index       i1 = hei;
      HalfEdge&        he1 = myHalfEdges[ i1 ];
      const Index       i2 = he1.opposite;
      HalfEdge&        he2 = myHalfEdges[ i2 ];
      const VertexIndex v2 = he1.toVertex;
      const VertexIndex v1 = he2.toVertex;
      const Index  i1_next = he1.next;
      const Index  i2_next = he2.next;
      HalfEdge&   he1_next = myHalfEdges[ i1_next ];
      HalfEdge&   he2_next = myHalfEdges[ i2_next ];
      const Index i1_next2 = he1_next.next;
      const Index i2_next2 = he2_next.next;
      myHalfEdges[ i1_next2 ].next = i2_next;
      myHalfEdges[ i2_next2 ].next = i1_next;
      he2_next.next  = i1;
      he1_next.next  = i2;
      he2_next.face  = he1.face;
      he1_next.face  = he2.face;
      he1.next       = i1_next2;
      he2.next       = i2_next2;
      he1.toVertex   = he1_next.toVertex;
      he2.toVertex   = he2_next.toVertex;
      // Reassign the mapping vertex v -> index of half edge starting from v
      // (JOL): must check before reassign for boundary vertices.
      if ( myVertexHalfEdges[ v1 ] == i1 ) myVertexHalfEdges[ v1 ] = i2_next;
      if ( myVertexHalfEdges[ v2 ] == i2 ) myVertexHalfEdges[ v2 ] = i1_next;
      // Reassign the mapping face f -> index of half edge contained in f
      myFaceHalfEdges[ he1.face ] = i1;
      myFaceHalfEdges[ he2.face ] = i2;
      // No need to reassign edge... it has just changed of vertices
      // but is still based on half-edges i1 and i2
      // Now taking care of mapping (vertex,vertex) -> half-edge.
      if ( update_arc2index ) 
        {
          myArc2Index.erase( Arc( v1, v2 ) );
          myArc2Index.erase( Arc( v2, v1 ) );
          const VertexIndex v2p = he1.toVertex;
          const VertexIndex v1p = he2.toVertex;
          myArc2Index[ Arc( v1p, v2p ) ] = i1;
          myArc2Index[ Arc( v2p, v1p ) ] = i2;
        }
    }

    /// Splits the edge specified by the half-edge \a i.
    ///
    /// @param i any valid half-edge index.
    ///
    /// @param update_arc2index (optimisation parameter), when 'true'
    /// updates everything consistently; when 'false' do not update
    /// myArc2Index map, which means you cannot get an half edge from
    /// two vertices afterwards. Use 'false' only if you know that you
    /// never use this mapping (e.g. no call to halfEdgeIndexFromArc)
    ///
    /// @return the index of the created vertex.
    ///
    /// @pre the edge must be flippable, `isFlippable( i ) == true`
    /// @see isFlippable
    /// @todo We could also split boundary triangles or more general faces.
    ///
    /// @note Time complexity is O(1) if \a update_arc2index is false,
    /// otherwise it is O(log n) is n is the number of arcs.
    VertexIndex split( const Index i, bool update_arc2index = true )
    {
      Index        new_hei = myHalfEdges.size();
      VertexIndex new_vtxi = myVertexHalfEdges.size();
      EdgeIndex  new_edgei = myEdgeHalfEdges.size();
      FaceIndex  new_facei = myFaceHalfEdges.size();
      myHalfEdges.resize( new_hei + 6 );
      myVertexHalfEdges.push_back( new_hei );
      HalfEdge&       hei = myHalfEdges[ i ];
      HalfEdge&  hei_next = myHalfEdges[ hei.next ];
      Index             j = hei.opposite;
      HalfEdge&       hej = myHalfEdges[ j ];
      HalfEdge&  hej_next = myHalfEdges[ hej.next ];
      HalfEdge&       he0 = myHalfEdges[ new_hei ];
      HalfEdge&       he1 = myHalfEdges[ new_hei + 1 ];
      HalfEdge&       he2 = myHalfEdges[ new_hei + 2 ];
      HalfEdge&       he3 = myHalfEdges[ new_hei + 3 ];
      HalfEdge&       he4 = myHalfEdges[ new_hei + 4 ];
      HalfEdge&       he5 = myHalfEdges[ new_hei + 5 ];
      // HalfEdge = { toVertex, face, edge, opposite, next }
      // Taking care of new half-edges
      he0.toVertex = hei_next.toVertex;
      he0.face     = hei.face;
      he0.edge     = new_edgei;
      he0.opposite = new_hei + 1;
      he0.next     = hei_next.next;
      he1.toVertex = new_vtxi;
      he1.face     = new_facei;
      he1.edge     = new_edgei;
      he1.opposite = new_hei;
      he1.next     = new_hei + 2;
      he2.toVertex = hei.toVertex;
      he2.face     = new_facei;
      he2.edge     = new_edgei + 1;
      he2.opposite = j;
      he2.next     = hei.next;
      he3.toVertex = hej_next.toVertex;
      he3.face     = hej.face;
      he3.edge     = new_edgei + 2;
      he3.opposite = new_hei + 4;
      he3.next     = hej_next.next;
      he4.toVertex = new_vtxi;
      he4.face     = new_facei + 1;
      he4.edge     = new_edgei + 2;
      he4.opposite = new_hei + 3;
      he4.next     = new_hei + 5;
      he5.toVertex = hej.toVertex;
      he5.face     = new_facei + 1;
      he5.edge     = hei.edge;
      he5.opposite = i;
      he5.next     = hej.next;
      // Updating existing half-edges
      hei.toVertex = new_vtxi;
      hei.opposite = new_hei + 5;
      hei.next     = new_hei;
      hej.toVertex = new_vtxi;
      hej.edge     = new_edgei + 1;
      hej.opposite = new_hei + 2;
      hej.next     = new_hei + 3;
      hei_next.face     = new_facei;
      hei_next.next     = new_hei + 1;
      hej_next.face     = new_facei + 1;
      hej_next.next     = new_hei + 4;
      // Updating other arrays
      myEdgeHalfEdges.push_back( new_hei );
      myEdgeHalfEdges.push_back( j );
      myEdgeHalfEdges.push_back( new_hei + 3 );
      myFaceHalfEdges.push_back( new_hei + 1 );
      myFaceHalfEdges.push_back( new_hei + 4 );
      myFaceHalfEdges[ hei.face ] = i;
      myFaceHalfEdges[ hej.face ] = j;
      myEdgeHalfEdges[ hei.edge ] = i;
      if ( update_arc2index )
	{
          const VertexIndex vi = he5.toVertex;
          const VertexIndex vk = hei_next.toVertex;
          const VertexIndex vj = he2.toVertex;
          const VertexIndex vl = hej_next.toVertex;
          myArc2Index.erase( Arc( vi, vj ) );
          myArc2Index.erase( Arc( vj, vi ) );
          myArc2Index[ Arc( vi, new_vtxi ) ] = i;
          myArc2Index[ Arc( new_vtxi, vi ) ] = new_hei + 5;
          myArc2Index[ Arc( vj, new_vtxi ) ] = j;
          myArc2Index[ Arc( new_vtxi, vj ) ] = new_hei + 2;
          myArc2Index[ Arc( vk, new_vtxi ) ] = new_hei + 1;
          myArc2Index[ Arc( new_vtxi, vk ) ] = new_hei;
          myArc2Index[ Arc( vl, new_vtxi ) ] = new_hei + 4;
          myArc2Index[ Arc( new_vtxi, vl ) ] = new_hei + 3;
	}
      return new_vtxi;
    }

    /// An edge is (topologically) mergeable iff: (1) it is bordered
    /// by two triangles, (2) there is no boundary vertex in
    /// the two triangles bordering the edge.
    ///
    /// @param hei any valid half-edge index.
    /// @return 'true' if the edge containing \a hei is topologically mergeable.
    ///
    /// @note Time complexity is O(1).
    bool isMergeable( const Index hei ) const { 
      ASSERT( hei != HALF_EDGE_INVALID_INDEX );
      const HalfEdge&  he = halfEdge( hei );
      const Index    hei2 = he.opposite;
      const HalfEdge& he2 = halfEdge( hei2 );
      // check if hei borders an infinite face.
      if ( ( he.face == HALF_EDGE_INVALID_INDEX )
	   || ( he2.face == HALF_EDGE_INVALID_INDEX ) )
	return false; 
      // Checks that he1 and he2 border a triangle.
      if ( ( nbSides( hei ) != 3 ) || ( nbSides( hei2 ) != 3) )
	return false;
      // Checks that no vertices around lie on the boundary.
      return ( ! isVertexBoundary( he.toVertex ) )
	&&   ( ! isVertexBoundary( he2.toVertex ) )
	&&   ( ! isVertexBoundary( halfEdge( he.next ).toVertex ) )
	&&   ( ! isVertexBoundary( halfEdge( he2.next ).toVertex ) );
    }

    
    /// Merges the edge specified by the half-edge \a hei.
    ///
    /// @param hei any valid half-edge index.
    ///
    /// @param update_arc2index (optimisation parameter), when 'true'
    /// updates everything consistently; when 'false' do not update
    /// myArc2Index map, which means you cannot get an half edge from
    /// two vertices afterwards. Use 'false' only if you know that you
    /// never use this mapping (e.g. no call to halfEdgeIndexFromArc)
    ///
    /// @return the index of the merged vertex.
    ///
    /// @pre the edge must be mergeable, `isMergeable( hei ) == true`
    /// @see isMergeable
    ///
    /// @note Time complexity is O(1) if \a update_arc2index is false,
    /// otherwise it is O(log n) is n is the number of arcs.
    /// @todo We could also merge boundary triangles or more general faces.
    VertexIndex merge( const Index hei, bool update_arc2index = true ) {
      const Index       i1 = hei;             // arc (v1,v2)
      const HalfEdge&  he1 = halfEdge( i1 );
      const Index      i1n = he1.next;
      const HalfEdge& he1n = halfEdge( i1n );
      const Index     i1nn = he1n.next;
      const Index       i2 = he1.opposite;    // arc (v2,v1)
      const HalfEdge&  he2 = halfEdge( i2 );
      const Index      i2n = he2.next;
      const HalfEdge& he2n = halfEdge( i2n );
      const Index     i2nn = he2n.next;
      const Index  iext1nn = halfEdge( i1nn ).opposite;
      const Index   iext1n = halfEdge( i1n  ).opposite;
      const Index  iext2nn = halfEdge( i2nn ).opposite;
      const Index   iext2n = halfEdge( i2n  ).opposite;
      const VertexIndex v1 = he2.toVertex;
      // Storing v2 the deleted vertex.
      const VertexIndex v2 = he1.toVertex;
      const VertexIndex v3 = he1n.toVertex;
      const VertexIndex v4 = he2n.toVertex;
      // Storing deleted face indices f1 and f2
      const FaceIndex   f1 = he1.face;
      const FaceIndex   f2 = he2.face;
      // Storing deleted edge indices (v1,v2), (v2,v3) and (v2,v3').
      const EdgeIndex   e1 = he1.edge;
      const EdgeIndex   e2 = he1n.edge;
      const EdgeIndex   e3 = halfEdge( i2nn ).edge;
      const EdgeIndex ev13 = halfEdge( iext1nn ).edge;
      const EdgeIndex ev14 = halfEdge( iext2n ).edge;
      // For debug
      auto nbV1 = nbNeighboringVertices( v1 );
      auto nbV2 = nbNeighboringVertices( v2 );
      auto nbV3 = nbNeighboringVertices( v3 );
      auto nbV4 = nbNeighboringVertices( v4 );
      // Changes toVertex field of half-edges pointing to v2	 
      std::vector<VertexIndex> outer_v;  // stores vertices around v2
      std::vector<Index>       inner_he; // stores half-edges from v2
      std::vector<Index>       outer_he; // stores half-edges toward v2
      Index i = i1n;
      do {
	const HalfEdge& he = halfEdge( i );
	const Index   iopp = he.opposite;
	outer_v. push_back( he.toVertex ); 
	inner_he.push_back( i );
	outer_he.push_back( iopp );
	HalfEdge&    heopp = myHalfEdges[ iopp ];
	ASSERT( heopp.toVertex == v2 );
	heopp.toVertex = v1;
	i = heopp.next;
      } while ( i != i1n ); // i2 precedes i1n around the vertex v2.
      // std::cout << "#outer_v=" << outer_v.size() << std::endl;
      // Gluing arcs around the two triangles
      // std::cout << "Gluing arcs around the two triangles" << std::endl;
      myHalfEdges[ iext1nn ].opposite = iext1n;
      myHalfEdges[ iext1n  ].opposite = iext1nn;
      myHalfEdges[ iext2nn ].opposite = iext2n;
      myHalfEdges[ iext2n  ].opposite = iext2nn;
      // Changing edges of merged edges
      // std::cout << "Changing edges of merged edges" << std::endl;
      myHalfEdges[ iext1n  ].edge     = ev13;
      myHalfEdges[ iext2nn ].edge     = ev14;
      // Taking care of look-up tables.
      // (1) myVertexHalfEdges
      // std::cout << "(1) myVertexHalfEdges" << std::endl;
      myVertexHalfEdges[ v1 ] = iext1nn;
      if ( myVertexHalfEdges[ v3 ] == i1nn )
	myVertexHalfEdges[ v3 ] = iext1n;
      if ( myVertexHalfEdges[ v4 ] == i2nn )
	myVertexHalfEdges[ v4 ] = iext2n;
      // (2) myFaceHalfEdges -> nothing to do.
      // (3) myEdgeHalfEdges
      // std::cout << "(3) myEdgeHalfEdges" << std::endl;
      myEdgeHalfEdges[ ev13 ] = iext1nn;
      myEdgeHalfEdges[ ev14 ] = iext2n;
      // (4) myArc2Index only if asked
      if ( update_arc2index ) {
	for ( std::size_t j = 0; j < outer_v.size(); ++j ) {
	  myArc2Index.erase( Arc( v2, outer_v[ j ] ) );
	  myArc2Index.erase( Arc( outer_v[ j ], v2 ) );
	}
	for ( std::size_t j = 1; j < ( outer_v.size() - 2 ); ++j ) {
	  myArc2Index[ Arc( v1, outer_v[ j ] ) ] = inner_he[ j ];
	  myArc2Index[ Arc( outer_v[ j ], v1 ) ] = outer_he[ j ];
	} 
	myArc2Index[ Arc( v3, v1 ) ] = iext1n;
	myArc2Index[ Arc( v1, v4 ) ] = iext2nn;
      }
      // Debug
      auto new_nbV1 = nbNeighboringVertices( v1 );
      auto new_nbV3 = nbNeighboringVertices( v3 );
      auto new_nbV4 = nbNeighboringVertices( v4 );
      if ( ( new_nbV1 != nbV1+nbV2-4 )
	   || ( new_nbV3 != nbV3 -1 )
	   || ( new_nbV4 != nbV4 -1 ) )
	trace.warning() << "Invalid nb of neighbors: "
			<< " nbV1=" << nbV1
			<< " nbV2=" << nbV2
			<< " nbV3=" << nbV3
			<< " nbV4=" << nbV4 << std::endl
			<< " new_nbV1=" << new_nbV1
			<< " new_nbV3=" << new_nbV3
			<< " new_nbV4=" << new_nbV4 << std::endl;

      // Renumbering of 1 vertex, 3 edges, 2 faces, 6 half-edges
      renumberVertex( v2, update_arc2index );
      std::array< EdgeIndex, 3 > E = { e1, e2, e3 };
      std::sort( E.begin(), E.end(), std::greater<EdgeIndex>() );
      for ( Index e : E ) {
	renumberEdge( e );
      }
      std::array< FaceIndex, 2 > F = { f1, f2 };
      std::sort( F.begin(), F.end(), std::greater<FaceIndex>() );
      for ( Index f : F ) {
	renumberFace( f );
      }
      std::array< Index, 6 > T = { i1, i1n, i1nn, i2, i2n, i2nn };
      std::sort( T.begin(), T.end(), std::greater<Index>() );
      for ( Index t : T ) {
	renumberHalfEdge( t );
      }
      return v1;
    }

    // ------------------------ protected services -------------------------
  protected:
    
    /// Renumber the last vertex of the triangulated surface as vertex
    /// i (this replaces it). The number of vertices of the data
    /// structure is decreased by 1.
    void renumberVertex( const VertexIndex vi, bool update_arc2index = true ) {
      // Vertex j becomes vertex i
      const VertexIndex vj = nbVertices() - 1;
      if ( vi != vj ) {
	const Index        j = myVertexHalfEdges[ vj ];
	Index              k = j;
	// Turns around vertex vj to modify toVertex fields.
	do {
	  const HalfEdge&    hek = halfEdge( k );
	  const Index       kopp = hek.opposite;
	  HalfEdge&       hekopp = myHalfEdges[ kopp ];
	  ASSERT( hekopp.toVertex == vj );
	  hekopp.toVertex = vi;
	  if ( update_arc2index ) {
	    myArc2Index.erase( Arc( vj, hek.toVertex ) );
	    myArc2Index.erase( Arc( hek.toVertex, vj ) );
	    myArc2Index[ Arc( vi, hek.toVertex ) ] = k;
	    myArc2Index[ Arc( hek.toVertex, vi ) ] = kopp;
	  }
	  k = hekopp.next;
	} while ( k != j );
	myVertexHalfEdges[ vi ] = j;
      }
      myVertexHalfEdges.pop_back();
    }

    /// Renumber the last edge of the triangulated surface as edge ei
    /// (this replaces it). The number of edges of the data
    /// structure is decreased by 1.
    void renumberEdge( const EdgeIndex ei ) {
      const EdgeIndex     ej = nbEdges() - 1;
      if ( ei != ej ) {
	const Index    j = myEdgeHalfEdges[ ej ];
	HalfEdge&    hej = myHalfEdges[ j ];
	const Index jopp = hej.opposite;
	HalfEdge& hejopp = myHalfEdges[ jopp ];
	ASSERT( hej.edge == ej );
	ASSERT( hejopp.edge == ej );
	hej.edge    = ei;
	hejopp.edge = ei;
	myEdgeHalfEdges[ ei ]  = j;
      }
      myEdgeHalfEdges.pop_back();
    }

    /// Renumber the last face of the triangulated surface as face fi
    /// (this replaces it). The number of faces of the data
    /// structure is decreased by 1.
    void renumberFace( const FaceIndex fi ) {
      const FaceIndex     fj = nbFaces() - 1;
      if ( fi != fj ) {
	const Index          j = myFaceHalfEdges[ fj ];
	Index k = j;
	do {
	  HalfEdge&    hek = myHalfEdges[ k ];
	  ASSERT( hek.face == fj );
	  hek.face         = fi;
	  k = hek.next;
	} while ( k != j );
	myFaceHalfEdges[ fi ] = j;
      }
      myFaceHalfEdges.pop_back();
    }

    /// Renumber the last half-edge of the triangulated surface as half-edge i
    /// (this replaces it). The number of half-edges of the data
    /// structure is decreased by 1.
    void renumberHalfEdge( const Index i, bool update_arc2index = true ) {
      const Index          j = nbHalfEdges() - 1;
      if ( i != j ) {
	// std::cout << "renumberHalfEdge j=" << j  << std::endl;
	const HalfEdge&    hej = halfEdge( j );
	const Index       jopp = hej.opposite;
	// std::cout << "renumberHalfEdge jopp=" << jopp << std::endl;
	HalfEdge&       hejopp = myHalfEdges[ jopp ];
	const VertexIndex   vj = hejopp.toVertex; // hej corresponds to (vj,vk)
	const VertexIndex   vk = hej.toVertex;    // hejopp corresponds to (vk,vj)
	// Update opposite and previous
	Index k = hej.next;
	while ( halfEdge( k ).next != j ) k = halfEdge( k ).next;
	myHalfEdges[ k ].next = i;
	hejopp.opposite       = i;
	// Take care of look-up tables
	// std::cout << "myVertexHalfEdges[" << vj << "]" << std::endl;
	if ( myVertexHalfEdges[ vj ] == j )
	  myVertexHalfEdges[ vj ] = i;
	// std::cout << "myEdgeHalfEdges[" << hej.edge << "]" << std::endl;
	if ( myEdgeHalfEdges[ hej.edge ] == j )
	  myEdgeHalfEdges[ hej.edge ] = i;
	// std::cout << "myFaceHalfEdges[" << hej.face << "]" << std::endl;
	if ( myFaceHalfEdges[ hej.face ] == j )
	  myFaceHalfEdges[ hej.face ] = i;
	// std::cout << "myArc2Index[" << vj << "," << vk << "]" << std::endl;
	if ( update_arc2index ) {
	  myArc2Index[ Arc( vj, vk ) ] = i;
	}
	// Copy last half-edge into i-th half-edge.
	myHalfEdges[ i ] = hej;
      }
      myHalfEdges.pop_back();
    }

    // ------------------------- consistency services --------------------
  public:
    /// Checks the whole half-edge structure for consistency.
    /// Complexity is at O(n log n) if n in the number of half-edges.
    /// 
    /// @param check_arc2index (optimisation parameter), when 'true'
    /// checks everything, otherwise does not check that the mapping
    /// myArc2Index is correct. This is used in conjunction with flip
    /// method when using the optimisation.
    ///
    /// @return 'true' iff all checks have passed.
    ///
    /// @see flip
    bool isValid( bool check_arc2index = true ) const
    {
      bool ok = true;
      // Checks that indices are within range
      for ( Index i = 0; i < nbHalfEdges(); i++ )
	{
          const HalfEdge& he = myHalfEdges[ i ];
	  if ( he.next >= nbHalfEdges() ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "half-edge " << i
			    << " has invalid next half-edge " << he.next
			    << std::endl;
            ok = false;
          }
	  if ( he.opposite >= nbHalfEdges() ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "half-edge " << i
			    << " has invalid opposite half-edge " << he.opposite
			    << std::endl;
            ok = false;
          }
	  if ( he.toVertex >= nbVertices() ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "half-edge " << i
			    << " has invalid toVertex " << he.toVertex
			    << std::endl;
            ok = false;
          }
	  if ( he.face >= nbFaces() && he.face != HALF_EDGE_INVALID_INDEX ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "half-edge " << i
			    << " has invalid face " << he.face
			    << std::endl;
            ok = false;
          }
	}
      // Checks that opposite are correct
      for ( Index i = 0; i < nbHalfEdges(); i++ )
        {
          const Index j = myHalfEdges[ i ].opposite;
          if ( j == HALF_EDGE_INVALID_INDEX ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "half-edge " << i << " has invalid opposite half-edge." << std::endl;
            ok = false;
          }
          if ( myHalfEdges[ j ].opposite != i ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "half-edge " << i << " has opposite half-edge " << j
                            << " but the latter has opposite half-edge " << myHalfEdges[ j ].opposite << std::endl;
            ok = false;
          }
	  const VertexIndex vi = myHalfEdges[ i ].toVertex;
	  const VertexIndex vj = myHalfEdges[ j ].toVertex;
	  if ( vi == vj ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
			    << "half-edge " << i << " and its opposite half-edge " << j
                            << " have the same toVertex " << vi << std::endl;
            ok = false;
          }
	  if ( vi >= nbVertices()  ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
			    << "half-edge " << i
			    << " points to an invalid vertex " << vi << std::endl;
            ok = false;
          }
	  if ( vj >= nbVertices()  ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
			    << "opposite half-edge " << j
			    << " points to an invalid vertex " << vj << std::endl;
            ok = false;
          }
        }
      // Checks that vertices have a correct starting half-edge.
      for ( VertexIndex i = 0; i < nbVertices(); i++ )
        {
          const Index        j = myVertexHalfEdges[ i ];
          const Index     jopp = myHalfEdges[ j ].opposite;
          const VertexIndex ip = myHalfEdges[ jopp ].toVertex;
          if ( ip != i ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "vertex " << i << " is associated to half-edge " << j
                            << " but its opposite half-edge " << jopp << " points to vertex " << ip << std::endl;
            ok = false;
          }
        }
      // Checks that faces have a correct bordering half-edge.
      for ( FaceIndex f = 0; f < nbFaces(); f++ )
        {
          const Index        i = myFaceHalfEdges[ f ];
          if ( myHalfEdges[ i ].face != f ) {
            trace.warning() << "[HalfEdgeDataStructure::isValid] "
                            << "face " << f << " is associated to half-edge " << i
                            << " but its associated face is " << myHalfEdges[ i ].face << std::endl;
            ok = false;
          }
        }
      // Checks that following next half-edges turns around the same face.
      for ( FaceIndex f = 0; f < nbFaces(); f++ )
        {
          Index           i = myFaceHalfEdges[ f ];
          const Index start = i;
          do {
            const HalfEdge& he = halfEdge( i );
            if ( he.face != f ) {
              trace.warning() << "[HalfEdgeDataStructure::isValid] "
                              << "when turning around face " << f << ", half-edge " << i
                              << " is associated to face " << he.face << std::endl;
              ok = false;
            }
            i = he.next;
          }
          while ( i != start );          
        }
      // Checks that turning around a vertex gives half-edges associated to this vertex.
      for ( VertexIndex v = 0; v < nbVertices(); v++ )
        {
          const Index s = halfEdgeIndexFromVertexIndex( v );
          Index       i = s;
          do
            {
              const HalfEdge&  he = halfEdge( i );
              const VertexIndex w = halfEdge( he.opposite ).toVertex;
              if ( v != w ) {
                trace.warning() << "[HalfEdgeDataStructure::isValid] "
                                << "when turning around vertex " << v << ", some opposite half-edge "
                                << he.opposite << " points to " << w << std::endl;
                ok = false;
              }
              i = halfEdge( he.opposite ).next;
            }
          while ( i != s );          
        }

      // Checks that boundary vertices have specific associated half-edges.
      VertexIndexRange bdryV = boundaryVertices();
      for ( VertexIndex i : bdryV ) {
        const Index j = myVertexHalfEdges[ i ];
        if ( halfEdge( j ).face != HALF_EDGE_INVALID_INDEX ) {
          trace.warning() << "[HalfEdgeDataStructure::isValid] "
                          << "boundary vertex " << i << " is associated to the half-edge " << j 
                          << " that does not lie on the boundary but on face " << halfEdge( j ).face 
                          << std::endl;
          ok = false;
        }
      }

      // Checks that that we can find arcs.
      for ( Index i = 0; i < nbHalfEdges(); i++ ) {
        const HalfEdge&   he = halfEdge( i );
        const VertexIndex v2 = he.toVertex;
        const VertexIndex v1 = halfEdge( he.opposite ).toVertex;
        const Index        j = findHalfEdgeIndexFromArc( v1, v2 );
        const HalfEdge&  he2 = halfEdge( j );
        if ( he2.toVertex != v2 ) {
          trace.warning() << "[HalfEdgeDataStructure::isValid] "
                          << "arc (" << v1 << "," << v2 << ") was found to be half-edge " << j
                          << " but it should be half-edge " << i << std::endl;
          ok = false;
        }
      }

      // Checks that edges have two correct associated half-edges.
      for ( EdgeIndex ei = 0; ei < nbEdges(); ei++ ) {
	const Index        i = myEdgeHalfEdges[ ei ];
        const HalfEdge&  hei = halfEdge( i );
	const Index        j = hei.opposite;
        const HalfEdge&  hej = halfEdge( j );
	if ( hei.edge != ei ) {
          trace.warning() << "[HalfEdgeDataStructure::isValid] "
                          << "edge " << ei << " is associated to half-edge " << i
                          << " but its edge is " << hei.edge << std::endl;
          ok = false;
	}
	if ( hej.edge != ei ) {
          trace.warning() << "[HalfEdgeDataStructure::isValid] "
                          << "edge " << ei << " is associated to half-edge " << i
			  << " of opposite half-edge " << j 
                          << " but its edge is " << hej.edge << std::endl;
          ok = false;
	}
      }
      
      // Checks that arcs have a correct associated half-edge.
      if ( check_arc2index )
        {
          for ( auto arc2idx : myArc2Index )
            {
              const VertexIndex v1 = arc2idx.first.first;
              const VertexIndex v2 = arc2idx.first.second;
              const Index        i = arc2idx.second;
              if ( myHalfEdges[ i ].toVertex != v2 ) {
                trace.warning() << "[HalfEdgeDataStructure::isValid] "
                                << "arc (" << v1 << "," << v2 << ") is associated to half-edge " << i 
                                << " but it points to vertex " << myHalfEdges[ i ].toVertex << std::endl;
                ok = false;
              }
              const Index       i2 = myHalfEdges[ i ].opposite;
              if ( myHalfEdges[ i2 ].toVertex != v1 ) {
                trace.warning() << "[HalfEdgeDataStructure::isValid] "
                                << "arc (" << v1 << "," << v2 << ") is associated to half-edge " << i 
                                << " but its opposite half-edge " << i2 << " points to vertex " << myHalfEdges[ i2 ].toVertex 
                                << std::endl;
                ok = false;
              }
            }
        }
      return ok;
    }

    /// Specific checks that are meaningful only for a triangulation
    /// (i.e. faces have three vertices).
    ///
    /// @return 'true' if all checks have passed.
    bool isValidTriangulation() const
    {
      bool ok = true;
      // Checks that indices are within range
      // Checks that following next half-edges turns around the same face.
      for ( FaceIndex f = 0; f < nbFaces(); f++ )
        {
          Index           i = myFaceHalfEdges[ f ];
          const Index start = i;
	  int           nbv = 0;
	  std::set<VertexIndex> V;
	  std::set<EdgeIndex>   E;
          do {
            const HalfEdge& he = halfEdge( i );
            if ( he.face != f ) {
              trace.warning() << "[HalfEdgeDataStructure::isValidTriangulation] "
                              << "when turning around face " << f
			      << ", half-edge " << i
                              << " is associated to face " << he.face
			      << std::endl;
              ok = false;
            }
	    V.insert( he.toVertex );
	    E.insert( he.edge );
	    nbv++;
            i = he.next;
          }
          while ( i != start );          
	  if ( nbv != 3 ) {
	    trace.warning() << "[HalfEdgeDataStructure::isValidTriangulation] "
			    << "when turning around face " << f
			    << ", we had to visit " << nbv
			    << " half-edges to loop (instead of 3)" << std::endl;
	    ok = false;
	  }
	  if ( V.size() != 3 ) {
	    trace.warning() << "[HalfEdgeDataStructure::isValidTriangulation] "
			    << "when turning around face " << f
			    << ", the set of vertices has not a size 3:";
	    for ( auto v : V ) trace.warning() << " " << v;
	    trace.warning() << std::endl;
	    ok = false;
	  }
	  if ( E.size() != 3 ) {
	    trace.warning() << "[HalfEdgeDataStructure::isValidTriangulation] "
			    << "when turning around face " << f
			    << ", the set of edges has not a size 3:";
	    for ( auto e : E ) trace.warning() << " " << e;
	    trace.warning() << std::endl;
	    ok = false;
	  }
        }
      return ok;
    }
  protected:

    /// Stores all the half-edges.
    std::vector< HalfEdge > myHalfEdges;
    /// Offsets into the 'halfedges' sequence, one per
    /// vertex. Associates to each vertex index the index of an half-edge
    /// originating from this vertex.
    std::vector< Index > myVertexHalfEdges;
    /// Offset into the 'halfedges' sequence, one per face. Associates
    /// to each face index the index of an half-edge lying on the
    /// border of this face.
    std::vector< Index > myFaceHalfEdges;
    /// Offset into the 'halfedges' sequence, one per edge (unordered
    /// pair of vertex indices). Associates to each edge index the
    /// index of an half-edge on this edge.
    std::vector< Index > myEdgeHalfEdges;
    /// The mapping between arcs to their half-edge index.
    Arc2Index myArc2Index;
    

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    // ------------------------- Hidden services ------------------------------
  protected:

    static
    FaceIndex arc2FaceIndex( const Arc2FaceIndex& de2fi,
                             VertexIndex vi, VertexIndex vj )
    {
      ASSERT( !de2fi.empty() );

      Arc2FaceIndex::const_iterator it = de2fi.find( Arc( vi, vj ) );
      // If no such directed edge exists, then there's no such face in the mesh.
      // The edge must be a boundary edge.
      // In this case, the reverse orientation edge must have a face.
      if( it == de2fi.end() )
        {
          ASSERT( de2fi.find( Arc( vj, vi ) ) != de2fi.end() );
          return HALF_EDGE_INVALID_INDEX;
        }
      return it->second;
    }
        
  }; // end of class HalfEdgeDataStructure


  /**
   * Overloads 'operator<<' for displaying objects of class 'HalfEdgeDataStructure'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'HalfEdgeDataStructure' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<< ( std::ostream & out, const HalfEdgeDataStructure & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/HalfEdgeDataStructure.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HalfEdgeDataStructure_h

#undef HalfEdgeDataStructure_RECURSES
#endif // else defined(HalfEdgeDataStructure_RECURSES)
