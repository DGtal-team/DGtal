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
#include "DGtal/helpers/StdDefs.h"

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class SurfaceMesh
  /**
     Description of template class 'SurfaceMesh' <p> \brief Aim:
     Represents a mesh as faces and a list of vertices. Vertices may
     be shared among faces but no correct topology is
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
    typedef SurfaceMesh< RealPoint, RealVector > Self;
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
    /// The type that defines a range of vertices
    typedef std::vector< Vertex >                   Vertices;
    /// The type that defines a range of faces
    typedef std::vector< Edge >                     Edges;
    typedef std::vector< WeightedEdge >             WeightedEdges;
    typedef std::vector< Face >                     Faces;
    typedef std::vector< WeightedFace >             WeightedFaces;
    typedef std::pair< Vertex, Vertex >             VertexPair;
    //---------------------------------------------------------------------------
  public:
    /// @name Standard services
    /// @{
    
    /// Default destructor.
    ~SurfaceMesh() = default;
    /// Default constructor.
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
    template <typename RealPointIterator, typename FaceIterator>
    SurfaceMesh( RealPointIterator itPos, RealPointIterator itPosEnd,
                    FaceIterator itFace, FaceIterator itFaceEnd );

    /// Initializes a mesh from vertex positions and polygonal faces
    /// (clears everything before).
    template <typename RealPointIterator, typename FaceIterator>
    bool init( RealPointIterator itPos, RealPointIterator itPosEnd,
	       FaceIterator itFace, FaceIterator itFaceEnd );

    /// Clears everything. The object is empty.
    void clear();

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
    /// triangulates implicitly the face to build the normal vector.
    void computeFaceNormalsFromPositions();

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
    /// proposed by [Max, 1999] for combining face information into
    /// vertex information.
    void computeVertexNormalsFromFaceNormalsWithMaxWeights();

    template <typename AnyRing>
    std::vector<AnyRing> computeFaceValuesFromVertexValues
    ( const std::vector<AnyRing>& vvalues ) const;
    
    template <typename AnyRing>
    std::vector<AnyRing> computeVertexValuesFromFaceValues
    ( const std::vector<AnyRing>& fvalues ) const;

    std::vector<RealVector> computeFaceUnitVectorsFromVertexUnitVectors
    ( const std::vector<RealVector>& vuvectors ) const;

    std::vector<RealVector> computeVertexUnitVectorsFromFaceUnitVectors
    ( const std::vector<RealVector>& fuvectors ) const;
    
    /// @}

    //---------------------------------------------------------------------------
  public:
    /// @name Accessors
    /// @{

    /// @return the number of faces of the mesh.
    Size nbFaces() const
    { return myIncidentVertices.size(); }

    /// @return the number of (unordered) edges of the mesh.
    Size nbEdges() const
    { return myEdgeVertices.size(); }

    /// @return the number of vertices of the mesh.
    Size nbVertices() const
    { return myIncidentFaces.size(); }

    /// @param i any vertex of the mesh
    /// @param j any vertex of the mesh
    /// @return the edge index of edge (i,j) or `nbEdges()` if this
    /// edge does not exist.
    /// @note O(log E) time complexity.
    Edge makeEdge( Vertex i, Vertex j ) const;
    
    /// @return a const reference to the vector giving for each face
    /// its incident vertices.
    const std::vector< Vertices >& incidentVertices() const
    { return myIncidentVertices; }

    /// @return a const reference to the vector giving for each vertex
    /// its incident faces.
    const std::vector< Faces >& incidentFaces() const
    { return myIncidentFaces; }

    /// @return a const reference to the vector of positions (of vertices).
    const std::vector< RealVector >& positions() const
    { return myPositions; }

    /// @return a const reference to the vector of normals to vertices.
    const std::vector< RealVector >& vertexNormals() const
    { return myVertexNormals; }

    /// @return a const reference to the vector of normals to faces.
    const std::vector< RealVector >& faceNormals() const
    { return myFaceNormals; }

    /// @return a reference to the vector of normals to vertices.
    std::vector< RealVector >& vertexNormals() 
    { return myVertexNormals; }

    /// @return a reference to the vector of normals to faces.
    std::vector< RealVector >& faceNormals() 
    { return myFaceNormals; }

    /// @return a const reference to the vector of neighbor faces for each face.
    const std::vector< Faces >& neighborFaces() const
    { return myNeighborFaces; }

    /// @return a const reference to the vector of neighbor vertices for each vertex.
    const std::vector< Vertices >& neighborVertices() const
    { return myNeighborVertices; }

    /// @return a const reference to the vector giving for each edge
    /// its two vertices (as a pair (i,j), i<j).
    /// @note edges are sorted in increasing order.
    const std::vector< VertexPair >& edgeVertices() const
    { return myEdgeVertices; }
    
    /// @return a const reference to the vector giving for each edge
    /// its incident faces (one, two, or more if non manifold)
    const std::vector< Faces >& edgeFaces() const
    { return myEdgeFaces; }

    /// @return a const reference to the vector giving for each edge
    /// its incident faces to its right (zero if open, one, or more if
    /// non manifold).
    ///
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its right, being defined ccw, means that the face is
    /// some `(..., j, i, ... )`.
    const std::vector< Faces >& edgeRightFaces() const 
    { return myEdgeRightFaces; }

    /// @return a const reference to the vector giving for each edge
    /// its incident faces to its left (zero if open, one, or more if
    /// non manifold).
    ///
    /// @note an edge is stored as a vertex pair (i,j), i < j. So a
    /// face to its left, being defined ccw, means that the face is
    /// some `(..., i, j, ... )`.
    const std::vector< Faces >& edgeLeftFaces() const 
    { return myEdgeLeftFaces; }
    
    
    /// @}

    //---------------------------------------------------------------------------
  public:
    /// @name Geometric services
    /// @{

    /// @return the average of the length of edges.
    Scalar averageEdgeLength() const;
    /// @param f any valid face index
    /// @return the average distance between the centroid of face \a f and its vertices.
    Scalar localWindow( Face f ) const;

    /// Perturbate the positions with a uniform random noise of 'p *
    /// averageEdgeLength' along arbitrary directions.
    /// @param p any positive real value.
    void perturbateWithUniformRandomNoise( Scalar p );
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
    /// @note Used in computeVertexNormalsFromFaceNormalsWithMaxWeights
    Scalars getMaxWeights( Index v ) const;
    
    WeightedFaces
    computeFacesInclusionsInBall( Scalar r, Index f ) const;
    std::tuple< Vertices, WeightedEdges, WeightedFaces >
    computeCellsInclusionsInBall( Scalar r, Index f ) const;
    
    /// Computes an approximation of the inclusion ratio of a given
    /// face \a f with a ball of radius \a r and center \a p.
    ///
    /// @param p the center of the ball.
    /// @param r the radius of the ball.
    /// @param f any index of face.
    ///
    /// @return the inclusion ratio as a scalar between 0 (no
    /// intersection) and 1 (inclusion).
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

    /// Computes neighboring information.
    void computeNeighbors();
    /// Computes edge information.
    void computeEdges();

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

