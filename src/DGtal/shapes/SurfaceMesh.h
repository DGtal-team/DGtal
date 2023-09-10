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
 * @file SurfaceMesh.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/11
 *
 * Header file for module SurfaceMesh.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SurfaceMesh_RECURSES)
#error Recursive header files inclusion detected in SurfaceMesh.h
#else // defined(SurfaceMesh_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SurfaceMesh_RECURSES

#if !defined SurfaceMesh_h
/** Prevents repeated inclusion of headers. */
#define SurfaceMesh_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/base/IntegerSequenceIterator.h"
#include "DGtal/helpers/StdDefs.h"

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class SurfaceMesh
  /**
     Description of template class 'SurfaceMesh' <p> \brief Aim:
     Represents an embedded mesh as faces and a list of vertices. Vertices may
     be shared among faces but no specific topology is
     required. However, you also have methods to navigate between
     neighbor vertices, faces, etc. The mesh can be equipped with
     normals at faces and/or vertices.

     We sum up below the possible classes for representing meshes:
     
     - @ref Mesh represents soaps of triangles with traversal
     operations but without any link between vertices;
     - @ref TriangulatedSurface represents 2-manifold triangulated
     meshes, possibly with boundaries, and provides traversal and
     neighborhood operations (underlying fast index representation);
     - @ref PolygonalSurface represents 2-manifold polygonal
     meshes, possibly with boundaries, and provides traversal and
     neighborhood operations (underlying fast index representation);
     - @ref SurfaceMesh represents arbitrary sets of faces with
     possible connexions between faces, edges and vertices, provides
     traversal and neighborhood operations (underlying fast index
     representation), and may represent non manifold meshes.
     - @ref DigitalSurface is a common interface to represent
     arbitrary digital surfaces, i.e. boundaries of sets of voxels
     or interfaces between such sets;
     - @ref IndexedDigitalSurface is a common interface to represent
     arbitrary digital surfaces, i.e. boundaries of sets of voxels
     or interfaces between such sets, but with underlying fast index
     representation.

     See also SurfaceMeshReader and SurfaceMeshWriter for input/output
     operations for SurfaceMesh.

     @tparam TRealPoint an arbitrary model of 3D RealPoint.
     @tparam TRealVector an arbitrary model of 3D RealVector.
  */
  template < typename TRealPoint, typename TRealVector >
  struct SurfaceMesh
  {
    typedef TRealPoint                              RealPoint;
    typedef TRealVector                             RealVector;
    typedef SurfaceMesh< RealPoint, RealVector >    Self;
    
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );
    
    typedef typename RealVector::Component          Scalar;
    typedef std::vector<Scalar>                     Scalars;
    /// The type for counting elements.
    typedef std::size_t                             Size;
    /// The type used for numbering vertices and faces
    typedef std::size_t                             Index;
    typedef Index                                   Face;
    typedef Index                                   Edge;
    typedef Index                                   Vertex;
    typedef std::pair< Edge, Scalar >               WeightedEdge;
    typedef std::pair< Face, Scalar >               WeightedFace;
    /// The type that defines a list/range of vertices (e.g. to define faces)
    typedef std::vector< Vertex >                   Vertices;
    /// The type that defines a list/range of edges
    typedef std::vector< Edge >                     Edges;
    typedef std::vector< WeightedEdge >             WeightedEdges;
    typedef std::vector< Face >                     Faces;
    typedef std::vector< WeightedFace >             WeightedFaces;
    typedef std::pair< Vertex, Vertex >             VertexPair;

    // Required by CUndirectedSimpleLocalGraph
    typedef std::set<Vertex>                   VertexSet;
    template <typename Value> struct           VertexMap {
      typedef typename std::map<Vertex, Value> Type;
    };

    // Required by CUndirectedSimpleGraph
    
    /// Non mutable iterator for visiting vertices.
    typedef IntegerSequenceIterator< Vertex >       ConstIterator;
    
    //---------------------------------------------------------------------------
  public:
    /// @name Standard services
    /// @{
    
    /// Default destructor.
    ~SurfaceMesh() = default;
    /// Default constructor.
    ///
    /// A typical construction usage is
    /// @code
    /// std::vector< RealPoint > positions = { { 0, 0, 5 }, { 1, 1, 3 }, { -1, 1, 3 }, { -1, -1, 3 }, { 1, -1, 3 } };
    /// std::vector< Vertices  > faces = { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 1 }, { 4, 3, 2, 1 } };
    /// SurfMesh pyramid_mesh;
    /// pyramid_mesh.init( positions.cbegin(), positions.cend(), faces.cbegin(), faces.cend() );
    /// @endcode
    SurfaceMesh() = default;
    /// Default copy constructor.
    /// @param other the object to clone
    SurfaceMesh( const Self& other ) = default;
    /// Default move constructor.
    /// @param other the object to move
    SurfaceMesh( Self&& other ) = default;
    /// Default assignment constructor.
    /// @param other the object to clone
    /// @return a reference to 'this'.
    Self& operator=( const Self& other ) = default;

    /// Builds a mesh from vertex positions and polygonal faces.
    ///
    /// @tparam RealPointIterator any forward iterator on RealPoint.
    /// @tparam VerticesIterator any forward iterator on the range of vertices defining a face.
    ///
    /// @param itPos start of range of iterators pointing on the positions of vertices of the mesh
    /// @param itPosEnd end of range of iterators pointing on the positions of vertices of the mesh.
    ///
    /// @param itVertices start of range of iterators pointing on the (oriented)
    ///   faces of the mesh, each face being a range of vertex indices.
    /// @param itVerticesEnd end of range of iterators pointing on the (oriented)
    ///   faces of the mesh, each face being a range of vertex indices.
    ///
    /// A typical construction usage is
    /// @code
    /// std::vector< RealPoint > positions = { { 0, 0, 5 }, { 1, 1, 3 }, { -1, 1, 3 }, { -1, -1, 3 }, { 1, -1, 3 } };
    /// std::vector< Vertices  > faces = { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 1 }, { 4, 3, 2, 1 } };
    /// auto pyramid_mesh = SurfMesh( positions.cbegin(), positions.cend(), faces.cbegin(), faces.cend() );
    /// @endcode
    template <typename RealPointIterator, typename VerticesIterator>
    SurfaceMesh( RealPointIterator itPos, RealPointIterator itPosEnd,
                 VerticesIterator itVertices, VerticesIterator itVerticesEnd );

    /// Initializes a mesh from vertex positions and polygonal faces
    /// (clears everything before).
    ///
    /// @tparam RealPointIterator any forward iterator on RealPoint.
    /// @tparam VerticesIterator any forward iterator on a range of vertices.
    ///
    /// @param itPos start of range of iterators pointing on the positions of vertices of the mesh
    /// @param itPosEnd end of range of iterators pointing on the positions of vertices of the mesh.
    ///
    /// @param itVertices start of range of iterators pointing on the (oriented)
    ///   faces of the mesh, each face being a range of vertex indices.
    /// @param itVerticesEnd end of range of iterators pointing on the (oriented)
    ///   faces of the mesh, each face being a range of vertex indices.
    ///
    /// A typical construction usage is
    /// @code
    /// std::vector< RealPoint > positions = { { 0, 0, 5 }, { 1, 1, 3 }, { -1, 1, 3 }, { -1, -1, 3 }, { 1, -1, 3 } };
    /// std::vector< Vertices  > faces = { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 1 }, { 4, 3, 2, 1 } };
    /// SurfMesh pyramid_mesh;
    /// pyramid_mesh.init( positions.cbegin(), positions.cend(), faces.cbegin(), faces.cend() );
    /// @endcode
    template <typename RealPointIterator, typename VerticesIterator>
    bool init( RealPointIterator itPos, RealPointIterator itPosEnd,
               VerticesIterator itVertices, VerticesIterator itVerticesEnd );

    /// Clears everything. The object is empty.
    void clear();

    /// @}
    
    //---------------------------------------------------------------------------
  public:
    /// @name Vertex and face vectors initialization and conversion services
    /// @{
    
    /// Given a range of real vectors, sets the normals of every
    /// vertex to the given vectors.
    template <typename RealVectorIterator>
    bool setVertexNormals( RealVectorIterator itN, RealVectorIterator itNEnd );

    /// Given a range of real vectors, sets the normals of every
    /// face to the given vectors.
    template <typename RealVectorIterator>
    bool setFaceNormals( RealVectorIterator itN, RealVectorIterator itNEnd );

    /// Uses the positions of vertices to compute a normal vector to
    /// each face of the mesh. It computes the barycenter,
    /// triangulates implicitly the face to build the normal vector
    /// from the average of implicit triangle normals.
    void computeFaceNormalsFromPositions();

    /// Uses the positions of vertices to compute a normal vector to
    /// the face \a f of the mesh. It computes the barycenter,
    /// triangulates implicitly the face to build the normal vector
    /// from the average of implicit triangle normals.
    /// @param f any valid index of face.
    void computeFaceNormalFromPositions( const Face f );

    /// Uses the normals associated with vertices to compute a normal
    /// vector to each face of the mesh. It simply averages the
    /// normals at every incident vertex.
    void computeFaceNormalsFromVertexNormals();

    /// Uses the normals associated with faces to compute a normal
    /// vector to each vertex of the mesh. It simply averages the
    /// normals of every incident face.
    void computeVertexNormalsFromFaceNormals();
    
    /// Uses the normals associated with faces to compute a normal
    /// vector to each vertex of the mesh. It uses the weights
    /// proposed by \cite max1999weights for combining face information into
    /// vertex information.
    void computeVertexNormalsFromFaceNormalsWithMaxWeights();

    /// @param vvalues any vector of vertex values.
    /// @return a vector of face values approximating \a vvalues.
    /// @tparam AnyRing any summable and averagable type.
    template <typename AnyRing>
    std::vector<AnyRing> computeFaceValuesFromVertexValues
    ( const std::vector<AnyRing>& vvalues ) const;
    
    /// @param fvalues any vector of vertex values.
    /// @return a vector of vertex values approximating \a fvalues.
    /// @tparam AnyRing any summable and averagable type.
    template <typename AnyRing>
    std::vector<AnyRing> computeVertexValuesFromFaceValues
    ( const std::vector<AnyRing>& fvalues ) const;

    /// @param vuvectors any vector of unit vectors on vertices.
    /// @return a vector of unit vectors on faces approximating \a vuvectors.
    std::vector<RealVector> computeFaceUnitVectorsFromVertexUnitVectors
    ( const std::vector<RealVector>& vuvectors ) const;

    /// @param fuvectors any vector of unit vectors on faces.
    /// @return a vector of unit vectors on vertices approximating \a fuvectors.
    std::vector<RealVector> computeVertexUnitVectorsFromFaceUnitVectors
    ( const std::vector<RealVector>& fuvectors ) const;
    
    /// @}

    //---------------------------------------------------------------------------
  public:
    /// @name Accessors
    /// @{

    /// @return the number of vertices of the mesh.
    Size nbVertices() const
    { return myIncidentFaces.size(); }

    /// @return the number of (unordered) edges of the mesh.
    Size nbEdges() const
    { return myEdgeVertices.size(); }

    /// @return the number of faces of the mesh.
    Size nbFaces() const
    { return myIncidentVertices.size(); }

    /// @return the euler characteristic of the triangulated surface
    /// (a famous topological invariant that is the number of vertices
    /// minus the number of edges plus the number of faces).
    long Euler() const
    { return nbVertices() - nbEdges() + nbFaces(); }
    
    /// @param i any vertex of the mesh
    /// @param j any vertex of the mesh
    /// @return the edge index of edge (i,j) or `nbEdges()` if this
    /// edge does not exist.
    /// @note O(log E) time complexity.
    Edge makeEdge( Vertex i, Vertex j ) const;

    /// @param f any face
    /// @return a const reference to the range giving for face \a f 
    /// its incident vertices.
    const Vertices&  incidentVertices( Face f ) const
    { return myIncidentVertices[ f ]; }

    /// @param v any vertex
    /// @return a const reference to the range giving for vertex \a v
    /// its incident faces.
    const Faces& incidentFaces( Vertex v ) const
    { return myIncidentFaces[ v ]; }
    
    /// @param f any face
    /// @return a const reference to the range of neighbor faces for face \a f.
    const Faces& neighborFaces( Face f ) const
    { return myNeighborFaces[ f ]; }

    /// @param v any vertex
    /// @return a const reference to the range of neighbor vertices for vertex \a v.
    const Vertices& neighborVertices( Vertex v ) const
    { return myNeighborVertices[ v ]; }

    /// @param e any edge
    /// @return a pair giving for edge \a e its two vertices (as a
    /// pair (i,j), i<j).
    /// @note if the edge is not valid, return {0,0}.
    VertexPair edgeVertices( Edge e ) const
    { return myEdgeVertices[ e ]; }
    
    /// @param e any edge
    /// @return a const reference to the range giving for edge \a e
    /// its incident faces (one, two, or more if non manifold)
    const Faces& edgeFaces( Edge e ) const
    { return myEdgeFaces[ e ]; }

    /// @param e any edge
    /// @return a const reference to the range giving for edge \a e
    /// its incident faces to its right (zero if open, one, or more if
    /// non manifold).
    ///
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its right, being defined ccw, means that the face is
    /// some `(..., j, i, ... )`.
    const Faces& edgeRightFaces( Edge e ) const 
    { return myEdgeRightFaces[ e ]; }

    /// @param e any edge
    /// @return a const reference to the range giving for edge \a e
    /// its incident faces to its left (zero if open, one, or more if
    /// non manifold).
    ///
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its left, being defined ccw, means that the face is
    /// some `(..., i, j, ... )`.
    const Faces& edgeLeftFaces( Edge e ) const 
    { return myEdgeLeftFaces[ e ]; }

    /// @return a const reference to the vector giving for each face
    /// its incident vertices.
    const std::vector< Vertices >& allIncidentVertices() const
    { return myIncidentVertices; }

    /// @return a const reference to the vector giving for each vertex
    /// its incident faces.
    const std::vector< Faces >& allIncidentFaces() const
    { return myIncidentFaces; }
    
    /// @return a const reference to the vector of neighbor faces for each face.
    const std::vector< Faces >& allNeighborFaces() const
    { return myNeighborFaces; }

    /// @return a const reference to the vector of neighbor vertices for each vertex.
    const std::vector< Vertices >& allNeighborVertices() const
    { return myNeighborVertices; }

    /// @return a vector giving for each edge its two vertices (as a
    /// pair (i,j), i<j).
    /// @note Since 1.4, order is not significant (this is to allow
    /// flip and modification in the surface mesh).
    const std::vector< VertexPair >& allEdgeVertices() const
    { return myEdgeVertices; }
    
    /// @return a const reference to the vector giving for each edge
    /// its incident faces (one, two, or more if non manifold)
    const std::vector< Faces >& allEdgeFaces() const
    { return myEdgeFaces; }

    /// @return a const reference to the vector giving for each edge
    /// its incident faces to its right (zero if open, one, or more if
    /// non manifold).
    ///
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its right, being defined ccw, means that the face is
    /// some `(..., j, i, ... )`.
    const std::vector< Faces >& allEdgeRightFaces() const 
    { return myEdgeRightFaces; }

    /// @return a const reference to the vector giving for each edge
    /// its incident faces to its left (zero if open, one, or more if
    /// non manifold).
    ///
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its left, being defined ccw, means that the face is
    /// some `(..., i, j, ... )`.
    const std::vector< Faces >& allEdgeLeftFaces() const 
    { return myEdgeLeftFaces; }
    
    /// @}

    //---------------------------------------------------------------------------
  public:
    /// @name Other topological services
    /// @{

    /// @return the edges that lie on the boundary of the mesh,
    /// i.e. they have only one incident face.
    Edges computeManifoldBoundaryEdges() const;
    /// @return the edges that lie on the inside of the mesh, with
    /// two incident faces, consistently oriented or not.
    ///
    /// @note union of \ref computeManifoldInnerConsistentEdges and
    /// \ref computeManifoldInnerUnconsistentEdges
    Edges computeManifoldInnerEdges() const;
    /// @return the edges that lie on the inside of the mesh, with
    /// consistent local orientation, i.e. they have one left incident
    /// face, and one right incident face.
    Edges computeManifoldInnerConsistentEdges() const;
    /// @return the edges that have two incident faces, but not
    /// correctly oriented, i.e. they may have two left incident faces
    /// and no right incident face, or two right incident faces and no
    /// left incident face.
    Edges computeManifoldInnerUnconsistentEdges() const;
    /// @return the edges that are non manifold, i.e. neither boundary
    /// or inner edges: they may have more than two incident faces, or
    /// two left incident faces for instance.
    Edges computeNonManifoldEdges() const;
    
    /// @return true if the boundary edges define a collection of
    /// manifold 1d polygonal curves (at most 2 adjecent edges per vertex).
    /// If checkClosed is set to true, we also check that all polygonal curves are closed.
    ///
    /// The method returns false if the surface mesh has no boundary.
    ///
    /// @param checkClosed if true, we check that each vertex has exactly two adejcent edges.
    bool isBoundariesManifold( bool checkClosed = true ) const
    {
      // computes unordered list of boundary vertices
      std::map<Vertex,Vertices> adjacent;
      auto MBE = this->computeManifoldBoundaryEdges();
      if ( MBE.size() == 0 ) return false;
      
      for (auto e : MBE)
      {
        auto ij = this->edgeVertices(e);
        adjacent[ ij.first ].push_back( ij.second );
        if ( adjacent[ ij.first ] .size() > 2 ) return false;
        adjacent[ ij.second ].push_back( ij.first );
        if ( adjacent[ ij.second ].size() > 2 )  return false;
      }
      //we may check if all curves are closed.
      if ( checkClosed )
        for ( const auto &adj : adjacent )
          if ( adj.second.size() != 2 ) return false;
      return true;
    }
    
    /// Extract the boundary of a surface mesh as a collection of sequences
    /// of vertices. The boundaries must be 1d manifold polygonal curves.
    /// @pre the boundaries must be manifold.
    ///
    /// @return a vector of polygonal simple curves (vector of vertices).
    std::vector<Vertices> computeManifoldBoundaryChains() const
    {
      std::vector<Vertices> boundaries;
      Vertices boundary;
      auto MBE = this->computeManifoldBoundaryEdges();
      std::map<Vertex,bool> visited;
      std::map<Vertex,Vertices> adjacent;
      
      ASSERT_MSG(MBE.size()>0,"The surface mesh must have boundary edges");
      ASSERT_MSG(this->isBoundariesManifold(), "The surface mesh mush have manifold boundaries");
      
      //Coompute adjecency relationships
      for (auto e : MBE)
      {
        auto ij = this->edgeVertices(e);
        visited[ij.first] = false;
        visited[ij.second] = false;
        adjacent[ij.first].push_back(ij.second);
        adjacent[ij.second].push_back(ij.first);
      }
      
      auto boundary_it = visited.begin();
      
      while(boundary_it != visited.end() )
      {
        Vertex first = (*boundary_it).first;
        visited[first] = true;
        boundary.clear();
        boundary.push_back(first);
        
        Vertex current = first;
        size_t nb_iter = 0;
        bool pushed=false;
        
        while ((!pushed) && (nb_iter < MBE.size()*2))
        {
          bool ok = false;
          for (auto other : adjacent[current])
            if (!visited[other])
            {
              boundary.push_back(other);
              current = other;
              visited[other] = true;
              ok = true;
              break;
            }
          if (!ok)
          {
            //all neighboors are visited
            for (auto other : adjacent[current])
              if (other == first)
              {
                boundaries.push_back(boundary);
                pushed = true;
                break;
              }
            //if first vertex isn't found then this chain is not
            //homeomorphic to a circle, hence isn't added to boundaries
          }
          nb_iter++;
        }
        boundary_it = std::find_if(visited.begin(), visited.end(),
                                   []
                                   (std::pair<Vertex,bool> x){return !x.second;});
      }
      return boundaries;
    }
    /// @}
    
    // ----------------------- Undirected simple graph services ----------------------
  public:
    /// @name Undirected simple graph services
    /// @{
    
    /**
     * @return the number of vertices of the surface.
     */
    Size size() const
    { return nbVertices(); }

    /**
     * @return an estimate of the maximum number of neighbors for this adjacency
     *
     * @note chosen here to be 8. Number of neighbors is 6 on average
     * for planar triangulations.
     */
    Size bestCapacity() const
    { return 8; }
      
    /**
     * @param v any vertex
     *
     * @return the number of neighbors of this vertex
     */
    Size degree( const Vertex & v ) const
    { return myNeighborVertices[ v ].size(); }

    /**
     * Writes the neighbors of a vertex using an output iterator
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
                    const Vertex & v ) const
    {
      for ( auto&& nv : myNeighborVertices[ v ] )
        *it++ = nv;
    }

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
                    const VertexPredicate & pred) const
    {
      for ( auto&& nv : myNeighborVertices[ v ] )
        if ( pred( nv ) ) *it++ = nv;
    }

    /// @return a (non mutable) iterator pointing on the first vertex.
    ConstIterator begin() const
    { return ConstIterator( 0 ); }

    /// @return a (non mutable) iterator pointing after the last vertex.
    ConstIterator end() const
    { return ConstIterator( nbVertices() ); }

    /// @}
    
    //---------------------------------------------------------------------------
  public:
    /// @name Geometric services
    /// @{

    /// @return a const reference to the vector of positions (of vertices).
    const std::vector< RealPoint >& positions() const
    { return myPositions; }

    /// Mutable accessor to vertex position.
    /// @param v any vertex.
    /// @return the mutable position associated to \a v.
    RealPoint& position( Vertex v )
    { return myPositions[ v ]; }
    
    /// Const accessor to vertex position.
    /// @param v any vertex.
    /// @return the non-mutable position associated to \a v.
    const RealPoint& position( Vertex v ) const
    { return myPositions[ v ]; }
      
    /// @return a const reference to the vector of normals to vertices.
    const std::vector< RealVector >& vertexNormals() const
    { return myVertexNormals; }

    /// @return a reference to the vector of normals to vertices.
    std::vector< RealVector >& vertexNormals() 
    { return myVertexNormals; }

    /// Mutable accessor to vertex normal.
    /// @param v any vertex.
    /// @return the mutable normal associated to \a v.
    RealVector& vertexNormal( Vertex v )
    { return myVertexNormals[ v ]; }
    
    /// Const accessor to vertex normal.
    /// @param v any vertex.
    /// @return the non-mutable normal associated to \a v.
    const RealVector& vertexNormal( Vertex v ) const
    { return myVertexNormals[ v ]; }
    
    /// @return a const reference to the vector of normals to faces.
    const std::vector< RealVector >& faceNormals() const
    { return myFaceNormals; }

    /// @return a reference to the vector of normals to faces.
    std::vector< RealVector >& faceNormals() 
    { return myFaceNormals; }

    /// Mutable accessor to face normal.
    /// @param f any face.
    /// @return the mutable normal associated to \a f.
    RealVector& faceNormal( Face f )
    { return myFaceNormals[ f ]; }
    
    /// Const accessor to face normal.
    /// @param f any face.
    /// @return the non-mutable normal associated to \a f.
    const RealVector& faceNormal( Face f ) const
    { return myFaceNormals[ f ]; }
    
    /// @return the average of the length of edges.
    Scalar averageEdgeLength() const;

    ///@return the Euclidean distance between any two vertices i and j.
    ///@param i first vertex
    ///@param j second vertex
    Scalar distance(const Vertex i, const Vertex j) const
    {
      //unrolling for compiler optimization
      const auto p=this->myPositions[ i ];
      const auto q=this->myPositions[ j ];
      return std::sqrt( (p[0]-q[0])*(p[0]-q[0]) + (p[1]-q[1])*(p[1]-q[1])+ (p[2]-q[2])*(p[2]-q[2]));
    }
    
    /// @param f any valid face index
    /// @return the average distance between the centroid of face \a f
    /// and its vertices.
    Scalar localWindow( Face f ) const;

    /// Perturbate the positions with a uniform random noise of 'p *
    /// averageEdgeLength' along arbitrary directions.
    /// @param p any positive real value.
    void perturbateWithUniformRandomNoise( Scalar p );

    /// Perturbate the positions with a uniform random noise of 'p *
    /// averageLocalEdgeLength' along arbitrary directions.
    /// @param p any positive real value.
    void perturbateWithAdaptiveUniformRandomNoise( Scalar p );

    /// @param e any valid edge index.
    /// @return the centroid (or barycenter) of edge \a f.
    RealPoint edgeCentroid( Index e ) const;

    /// @param f any valid face index.
    /// @return the centroid (or barycenter) of face \a f.
    RealPoint faceCentroid( Index f ) const;

    /// @param f any valid face index.
    /// @return the area of face \a f.
    Scalar faceArea( Index f ) const;

    /// @param v any valid vertex index.
    /// @return the Max's weights for each incident face to \a v, in the same order as `myIncidentFaces[ v ]`.
    /// @note Used in computeVertexNormalsFromFaceNormalsWithMaxWeights, see \cite max1999weights
    Scalars getMaxWeights( Index v ) const;
    
    /// Given a ball of radius \a r centered on the centroid of face
    /// \a f, return the faces having a non empty intersection with
    /// this ball, each one weighted by its ratio of inclusion (in the
    /// range [0,1] where 0 is empty intersection and 1 is completely
    /// included).
    ///
    /// @param r the radius of the ball.
    /// @param f the face where the ball is centered.
    ///
    /// @return the range of faces having an non empty intersection
    /// with this ball, each one weighted by its ratio of inclusion
    /// (in the range [0,1] where 0 is empty intersection and 1 is
    /// completely included).
    ///
    /// @note For faces, the ratio is the @e approximated @e area of
    /// the face part inside the ball and the face area. The
    /// approximation is estimated by computing the distance of face
    /// vertices and face barycenter to the ball center and by linear
    /// interpolation of the results.
    WeightedFaces
    computeFacesInclusionsInBall( Scalar r, Index f ) const;
    
    /// Given a ball of radius \a r centered on a point \a p belonging
    /// to face \a f, return the faces having a non empty intersection
    /// with this ball, each one weighted by its ratio of inclusion
    /// (in the range [0,1] where 0 is empty intersection and 1 is
    /// completely included).
    ///
    /// @param r the radius of the ball.
    /// @param f the face where the ball is centered.
    /// @param p the position on the face where the ball is centered.
    ///
    /// @return the range of faces having an non empty intersection
    /// with this ball, each one weighted by its ratio of inclusion
    /// (in the range [0,1] where 0 is empty intersection and 1 is
    /// completely included).
    ///
    /// @note For faces, the ratio is the @e approximated @e area of
    /// the face part inside the ball and the face area. The
    /// approximation is estimated by computing the distance of face
    /// vertices and face barycenter to the ball center and by linear
    /// interpolation of the results.
    WeightedFaces
    computeFacesInclusionsInBall( Scalar r, Index f, RealPoint p ) const;
    
    /// Given a ball of radius \a r centered on the centroid of face
    /// \a f, return the vertices/edges/faces having an non empty
    /// intersection with this ball, each edge/face weighted by its
    /// ratio of inclusion (in the range [0,1] where 0 is empty
    /// intersection and 1 is completely included).
    ///
    /// @param r the radius of the ball.
    /// @param f the face where the ball is centered.
    ///
    /// @return the range of vertices/edges/faces having an non empty
    /// intersection with this ball, each edge/face weighted by its
    /// ratio of inclusion (in the range [0,1] where 0 is empty
    /// intersection and 1 is completely included).
    ///
    /// @note a vertex is either included or not, so no weight is necessary.
    std::tuple< Vertices, WeightedEdges, WeightedFaces >
    computeCellsInclusionsInBall( Scalar r, Index f ) const;

    /// Given a ball of radius \a r centered on a point \a p belonging
    /// to face \a f, return the vertices/edges/faces having an non
    /// empty intersection with this ball, each edge/face weighted by
    /// its ratio of inclusion (in the range [0,1] where 0 is empty
    /// intersection and 1 is completely included).
    ///
    /// @param r the radius of the ball.
    /// @param f the face where the ball is centered.
    /// @param p the position on the face where the ball is centered.
    ///
    /// @return the range of vertices/edges/faces having an non empty
    /// intersection with this ball, each edge/face weighted by its
    /// ratio of inclusion (in the range [0,1] where 0 is empty
    /// intersection and 1 is completely included).
    ///
    /// @note a vertex is either included or not, so no weight is necessary.
    std::tuple< Vertices, WeightedEdges, WeightedFaces >
    computeCellsInclusionsInBall( Scalar r, Index f, RealPoint p ) const;
    
    /// Computes an approximation of the inclusion ratio of a given
    /// face \a f with a ball of radius \a r and center \a p.
    ///
    /// @param p the center of the ball.
    /// @param r the radius of the ball.
    /// @param f any index of face.
    ///
    /// @return the inclusion ratio as a scalar between 0 (no
    /// intersection) and 1 (inclusion).
    ///
    /// @note The approximation is estimated by computing the distance
    /// of face vertices and face barycenter to the ball center and by
    /// linear interpolation of the results.
    Scalar faceInclusionRatio( RealPoint p, Scalar r, Index f ) const;

    /// Computes an approximation of the inclusion ratio of a given
    /// edge \a e with a ball of radius \a r and center \a p.
    ///
    /// @param p the center of the ball.
    /// @param r the radius of the ball.
    /// @param e any index of edge.
    ///
    /// @return the inclusion ratio as a scalar between 0 (no
    /// intersection) and 1 (inclusion).
    ///
    /// @note The approximation is estimated by computing the distance
    /// of edge vertices and edge midpoint to the ball center and by
    /// linear interpolation of the results.
    Scalar edgeInclusionRatio( RealPoint p, Scalar r, Index e ) const;

    /// Computes the inclusion ratio of a given
    /// vertex \a v with a ball of radius \a r and center \a p.
    ///
    /// @param p the center of the ball.
    /// @param r the radius of the ball.
    /// @param v any index of vertex.
    ///
    /// @return the inclusion ratio as a scalar, either 0 (no
    /// intersection) or 1 (inclusion).
    Scalar vertexInclusionRatio( RealPoint p, Scalar r, Index v ) const;

    /// @}

    //---------------------------------------------------------------------------
  public:
    /// @name Mesh editing services
    /// @{

    /// An edge is (topologically) flippable iff: (1) it does not lie
    /// on the boundary, (2) it is bordered by two triangles, one that
    /// to its right, one to its left, (3) the two other vertices of
    /// the quad are not already neighbors, (4) the edge is not
    /// bordered by the same two triangles, in opposite orientation.
    ///
    /// @param e any edge.
    /// @return 'true' if the edge \a e is topologically flippable.
    ///
    /// @note Time complexity is O(1).
    bool isFlippable( const Edge e ) const;

    /// @pre `isFlippable( e )` must be true.
    /// @param e any edge.
    /// @return the two other vertices of the quadrilateral around the edge \a e. 
    VertexPair otherDiagonal( const Edge e ) const;
    
    /**
       Flip the edge \a e. Be careful that after the flip, this edge
       index determines another edge, which is the other diagonal of
       the quadrilateral having \a e as its diagonal.
      
       \verbatim
             l                   l
            / \                 /|\
           /   \               / | \
          /     \             /  |  \
         /   lf  \           /   |   \
        /         \         /    |    \
       i --- e --- j  ==>  i  lf e  rf j    if k < l otherwise rf and lf are swapped
        \         /         \    |    /
         \   rf  /           \   |   /
          \     /             \  |  /
           \   /               \ | /
            \ /                 \|/
             k                   k
       \endverbatim
      
       @param e any valid edge.
      
       @param recompute_face_normals when 'true', recompute normals
       of flipped faces with the positions of the vertices.
      
       @pre the edge must be flippable, `isFlippable( e ) == true`
      
       @post After the flip, the edge index \a e corresponds to the
       index of the flipped edge (if you reflip it you get your
       former configuration).
      
       @note Time complexity is O(log n), due to the updating of
       surrounding edges information.
      
       @warning For performance reasons, The neighbor faces of each
       face are not recomputed. One should call \ref computeNeighbors
       to recompute them. However the neighbor vertices to each
       vertex are recomputed.
      
       @warning Vertex normals are not recomputed, but face normals
       may be recomputed if asked for. The face normals are then the
       geometric normals of triangles.
    */
    void flip( const Edge e, bool recompute_face_normals = false );
    
    /// @}    

    //---------------------------------------------------------------------------
  public:
    /// @name Look-up table computation services
    /// @{
    
    /// Computes neighboring information.
    void computeNeighbors();
    /// Computes edge information.
    void computeEdges();

    /// @}
    
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
    /// For each face, its range of incident vertices
    std::vector< Vertices >     myIncidentVertices;
    /// For each vertex, its range of incident faces
    std::vector< Faces >        myIncidentFaces;
    /// For each vertex, its position
    std::vector< RealPoint >    myPositions;
    /// For each vertex, its normal vector
    std::vector< RealVector >   myVertexNormals;
    /// For each face, its normal vector
    std::vector< RealVector >   myFaceNormals;
    /// For each face, its range of neighbor faces (no particular order)
    std::vector< Faces >        myNeighborFaces;
    /// For each vertex, its range of neighbor vertices (no particular order)
    std::vector< Vertices >     myNeighborVertices;
    /// For each edge, its two vertices
    std::vector< VertexPair >   myEdgeVertices;
    /// For each vertex pair, its edge index.
    std::map< VertexPair,Edge > myVertexPairEdge;
    /// For each edge, its faces (one, two, or more if non manifold)
    std::vector< Faces >        myEdgeFaces;
    /// For each edge, its faces to its right  (zero if open, one, or more if
    /// non manifold).
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its right, being defined ccw, means that the face is
    /// some `(..., j, i, ... )`.
    std::vector< Faces >        myEdgeRightFaces;
    /// For each edge, its faces to its left  (zero if open, one, or more if
    /// non manifold).
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its left, being defined ccw, means that the face is
    /// some `(..., i, j, ... )`.
    std::vector< Faces >        myEdgeLeftFaces;

    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  protected:

    /// Removes the index \a i from the vector \a v.
    /// @param[inout] v a vector of indices
    /// @param[in] i an index
    void removeIndex( std::vector< Index >& v, Index i )
    {
      const std::size_t n = v.size();
      for ( std::size_t j = 0; j < n; j++ )
	if ( v[ j ] == i )
	  {
	    std::swap( v[ j ], v.back() );
	    v.resize( n - 1 );
	    return;
	  }
      trace.error() << "[SurfaceMesh::removeIndex] Index " << i
		    << " is not in vector:";
      for ( auto e : v ) std::cerr << " " << e;
      std::cerr << std::endl;
    }

    /// Replaces the index \a i with the index \a ri in the vector \a v.
    /// @param[inout] v a vector of indices
    /// @param[in] i an index
    /// @param[in] ri an index    
    void replaceIndex( std::vector< Index >& v, Index i, Index ri )
    {
      const std::size_t n = v.size();
      for ( std::size_t j = 0; j < n; j++ )
	if ( v[ j ] == i )
	  {
	    v[ j ] = ri;
	    return;
	  }
      trace.error() << "[SurfaceMesh::replaceIndex] Index " << i
		    << " (subs=" << ri << ") is not in vector:";
      for ( auto e : v ) std::cerr << " " << e;
      std::cerr << std::endl;
    }

    /// Adds the index \a i to the vector \a v.
    /// @param[inout] v a vector of indices
    /// @param[in] i an index
    void addIndex( std::vector< Index >& v, Index i )
    {
      v.push_back( i );
    }
    

    /// @return a random number between 0.0 and 1.0
    static Scalar rand01()
    { return (Scalar) rand() / (Scalar) RAND_MAX; }
    
  }; // end of class SurfaceMesh

  /**
   * Overloads 'operator<<' for displaying objects of class 'SurfaceMesh'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'SurfaceMesh' to write.
   * @return the output stream after the writing.
   */
  template < typename TRealPoint, typename TRealVector >
  std::ostream&
  operator<< ( std::ostream & out,
               const SurfaceMesh<TRealPoint, TRealVector> & object );  
  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "SurfaceMesh.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SurfaceMesh_h

#undef SurfaceMesh_RECURSES
#endif // else defined(SurfaceMesh_RECURSES)
