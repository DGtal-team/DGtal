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
 * @file ConvexCellComplex.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/12/20
 *
 * Header file for module ConvexCellComplex.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConvexCellComplex_RECURSES)
#error Recursive header files inclusion detected in ConvexCellComplex.h
#else // defined(ConvexCellComplex_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConvexCellComplex_RECURSES

#if !defined ConvexCellComplex_h
/** Prevents repeated inclusion of headers. */
#define ConvexCellComplex_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/math/linalg/SimpleMatrix.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexCellComplex
  /**
     Description of template class 'ConvexCellComplex' <p> \brief Aim:
     represents a d-dimensional complex in a d-dimensional
     space with the following properties and restrictions:
     - all cells (maximal and below) are convex
     - k-cells span a k-dimensional affine space
     - cells are convex, their geometry is described by their vertices
     - the boundary of each d-cell is composed of d-1-cells, called faces
     - each face is shared by two d-cells (one can be infinite), but
       are viewed with opposite orientation from each d-cell
     - each vertex hold its position
     - cells may be spherical (their vertices are cospherical) if the
       complex comes from a Delaunay triangulation.

     All cells are indexed per dimension, starting from 0, hence:
     - vertices are numbered from 0 to `nbVertices()-1`
     - faces are numbered from 0 to `nbFaces()-1`
     - cells are numbered from 0 to `nbCells()-1` (the infinite cell
       is not in this range).

     The ConvexCellComplex is used to represent the Delaunay
     decomposition into convex cells of a range of points. It is built
     with ConvexityHelper functions like
     ConvexityHelper::computeDelaunayCellComplex.

     @tparam TPoint an arbitrary model of Point.
 
     @see moduleQuickHull
  */
  template < typename TPoint >
  struct ConvexCellComplex {
    static const Dimension dimension = TPoint::dimension;

    typedef TPoint      Point;
    typedef std::size_t Index;
    typedef std::size_t Size;

    static const Index INFINITE_CELL = (Index) -1;
    typedef Index                    Cell;
    typedef std::pair< Index, bool > Face;
    typedef Index                    Vertex;
    typedef std::vector< Index >     IndexRange;
    typedef std::vector< Vertex >    VertexRange;
    typedef std::vector< Face >      FaceRange;

    typedef typename Point::Coordinate       Scalar;
    typedef PointVector< dimension, Scalar > Vector;
    typedef PointVector< dimension, double > RealPoint;
    typedef PointVector< dimension, double > RealVector;
    
    // ----------------------- standard services ---------------------------
  public:
    /// @name Standard services
    /// @{

    /// Defaut constructor.
    ConvexCellComplex()
    { clear(); }

    /// Clears the complex (as if it was just default constructed).
    void clear()
    {
      cell_faces.clear();
      cell_vertices.clear();
      true_face_cell.clear();
      false_face_cell.clear();
      true_face_vertices.clear();
      vertex_position.clear();
      has_face_geometry = false;
      true_face_normal.clear();
      true_face_intercept.clear();
    }
    
    /// @return the number of finite cells with maximal dimension
    Size nbCells() const
    { return cell_faces.size(); }
    /// @return the number of unoriented cells of codimension 1 
    Size nbFaces() const
    { return true_face_cell.size(); }
    /// @return the number of vertices
    Size nbVertices() const
    { return vertex_position.size(); }

    /// @}
    // -------------------- primal structure topology services ---------------------
  public:
    /// @name Primal structure topology services
    /// @{
    
    /// @return the index of the infinite cell.
    Cell infiniteCell() const
    { return INFINITE_CELL; }

    /// @param[in] c any cell
    /// @return 'true' is the cell is infinite
    bool isInfinite( const Cell c ) const
    { return c == INFINITE_CELL; }
      
    
    /// @param[in] f any face
    /// @return its opposite face
    Face opposite( const Face f ) const
    { return std::make_pair( f.first, ! f.second ); }

    /// @param[in] c any valid cell
    /// @return its range of faces.
    const FaceRange& cellFaces( const Cell c ) const
    { return cell_faces[ c ]; }
    
    /// Lazy computation of cell vertices from face vertices if they
    /// were not given.
    ///
    /// @param[in] c any (finite) cell
    /// @return its vertices
    const VertexRange& cellVertices( const Cell c ) const
    {
      ASSERT( c != INFINITE_CELL && c < nbCells() );
      ASSERT( ! cell_vertices.empty() );
      if ( cell_vertices.empty() )
        cell_vertices.resize( nbCells() );
      if ( cell_vertices[ c ].empty() )
        computeCellVertices( c );
      return cell_vertices[ c ];
    }

    /// @return the range of vertex ranges giving for each cell its vertices.
    const std::vector< VertexRange > & allCellVertices() const
    {
      if ( cell_vertices.empty() )
        cell_vertices.resize( nbCells() );
      for ( Index c = 0; c < nbCells(); ++c )
        if ( cell_vertices[ c ].empty() )
          computeCellVertices( c );
      return cell_vertices;
    }
    
    /// @param[in] f any face
    /// @return its vertices (in order depending on the side of the face)
    VertexRange faceVertices( const Face f ) const
    {
      if ( f.second ) return true_face_vertices[ f.first ];
      return VertexRange( true_face_vertices[ f.first ].crbegin(),
                          true_face_vertices[ f.first ].crend() );
    }

    /// @param[in] f any face
    /// @return its associated cell (or INFINITE_CELL)
    Cell faceCell( const Face f ) const
    {
      return f.second ? true_face_cell[ f.first ] : false_face_cell[ f.first ];
    }

    /// @param[in] f any face
    ///
    /// @return the vertices within the cell containing \a f that are
    /// not in \a f.
    VertexRange faceComplementVertices( const Face f ) const
    {
      VertexRange result;
      const Cell c = faceCell( f );
      if ( isInfinite( c ) ) return result;
      const auto& c_vtcs = cellVertices( c );
      auto        f_vtcs = true_face_vertices[ f.first ];
      std::sort( f_vtcs.begin(), f_vtcs.end() );
      for ( auto v : c_vtcs )
        if ( ! std::binary_search( f_vtcs.cbegin(), f_vtcs.cend(), v ) )
          result.push_back( v );
      return result;
    }
    
    /// @}
    // -------------------- primal structure geometry services ---------------------
  public:
    /// @name Primal structure geometry services
    /// @{

    /// @param[in] c any valid cell (non infinite)
    /// @return the range of positions of all cell vertices
    std::vector< Point > cellVertexPositions( const Cell c ) const
    {
      ASSERT( ! isInfinite( c ) );
      const auto vtcs = cellVertices( c );
      std::vector< Point > pts( vtcs.size() );
      std::transform( vtcs.cbegin(), vtcs.cend(), pts.begin(),
                      [&] ( Vertex v ) { return this->position( v ); } );
      return pts;
    }

    /// @param[in] f any face
    /// @return the range of positions of all face vertices
    std::vector< Point > faceVertexPositions( const Face f ) const
    {
      const auto vtcs = faceVertices( f );
      std::vector< Point > pts( vtcs.size() );
      std::transform( vtcs.cbegin(), vtcs.cend(), pts.begin(),
                      [&] ( Vertex v ) { return this->position( v ); } );
      return pts;
    }
    
    /// @param[in] v any vertex
    /// @return the vertex position
    Point position( const Vertex v ) const
    {
      return vertex_position[ v ];
    }

    /// @param[in] p any point
    /// @return its embedding in the real Euclidean space.
    RealPoint toReal( const Point p ) const
    {
      RealPoint x;
      for ( Dimension k = 0; k < dimension; ++k )
        x[ k ] = (double) p[ k ];
      return x;
    }

    /// @param[in] p any real point
    /// @param[in] factor the dilation applied to point \a p before integer conversion
    /// @return its embedding in the real Euclidean space.
    template < typename LatticePoint >
    LatticePoint toLattice( const RealPoint p, double factor = 1.0 ) const
    {
      LatticePoint x;
      for ( Dimension k = 0; k < dimension; ++k )
        x[ k ] = (typename LatticePoint::Coordinate) round( p[ k ] * factor );
      return x;
    }

    
    /// @param[in] c any valid cell
    /// @return the cell barycenter.
    RealPoint cellBarycenter( const Cell c ) const
    {
      ASSERT( ! isInfinite( c ) );
      RealPoint b;
      const auto& vtcs = cellVertices( c );
      for ( auto v : vtcs )
        b += toReal( position( v ) );
      return b / vtcs.size();
    }

    /// @pre `hasFaceGeometry() == true`
    /// @param[in] c any valid cell
    /// @param[in] factor the dilation applied to all points before integer conversion
    /// @return the corresponding polytope
    template < typename LatticePolytope >
    LatticePolytope
    cellLatticePolytope( const Cell c, const double factor = 1.0 ) const
    {
      ASSERT( hasFaceGeometry() );
      ASSERT( ! isInfinite( c ) );
      typedef typename LatticePolytope::Point     LatticePoint;
      typedef typename LatticePolytope::HalfSpace HalfSpace;
      typedef typename LatticePolytope::Domain    Domain;
      const auto vtcs = cellVertices( c );
      std::vector< LatticePoint > pts;
      for ( auto v : vtcs )
        pts.push_back( toLattice< LatticePoint >( position ( v ), factor ) );
      LatticePoint l = pts[ 0 ];
      LatticePoint u = pts[ 0 ];
      for ( const auto& p : pts ) {
        l = l.inf( p );
        u = u.sup( p );
      }
      Domain domain( l, u );
      std::vector< HalfSpace > HS;
      for ( auto f : cellFaces( c ) ) {
        HS.emplace_back( faceNormal( f ), faceIntercept( f ) );
      }
      return LatticePolytope( domain, HS.cbegin(), HS.cend(), false );
    }

    /// @pre `hasFaceGeometry() == true`
    /// @param[in] f any face
    /// @return its normal oriented outward its enclosing cell.
    const Vector& faceNormal( const Face f ) const
    {
      ASSERT( hasFaceGeometry() );
      ASSERT( f.first < nbFaces() );
      if ( f.second ) return  true_face_normal[ f.first ];
      else            return -true_face_normal[ f.first ];
    }

    /// @pre `hasFaceGeometry() == true`
    /// @param[in] f any face
    /// @return its intercept.
    Scalar faceIntercept( const Face f ) const
    {
      ASSERT( hasFaceGeometry() );
      ASSERT( f.first < nbFaces() );
      if ( f.second ) return  true_face_intercept[ f.first ];
      else            return -true_face_intercept[ f.first ];
    }
      
    /// @return 'true' iff the face geometry is computed.
    bool hasFaceGeometry() const
    { return has_face_geometry; } 

    /// Forces the computation of face geometry.
    void requireFaceGeometry() 
    {
      if ( ! hasFaceGeometry() )
        computeFaceGeometry();
    }

    /// Release ressources allocated to face geometry.
    void unrequireFaceGeometry()
    {
      has_face_geometry = false;
      true_face_normal.clear();
      true_face_intercept.clear();
    }

    
    /// @}
    // ----------------------- datas for primal structure ----------------------
  public:
    /// @name Data for primal structure
    /// @{
    
    // for each cell, gives its faces
    std::vector< FaceRange >   cell_faces;
    // for each cell, gives its vertices
    mutable std::vector< VertexRange > cell_vertices;
    // for each 'true' face, gives its cell (or INFINITE_CELL)
    std::vector< Cell >        true_face_cell;
    // for each 'false' face, gives its cell (or INFINITE_CELL)
    std::vector< Cell >        false_face_cell;
    // for each true face, gives its vertices in order.
    std::vector< VertexRange > true_face_vertices;
    // for each vertex, gives its position
    std::vector< Point >       vertex_position;

    /// Tells if the face geometry has been computed
    bool has_face_geometry;
    /// Contains the outward oriented normal of each 'true' face.
    std::vector< Vector > true_face_normal;
    /// Contains the intercept of each 'true' face.
    std::vector< Scalar > true_face_intercept;
    
    /// @}
    // ----------------------- Interface ----------------------
  public:
    /// @name Interface
    /// @{

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[ConvexCellComplex<" << dimension << ">"
          << " #C=" << nbCells()
          << " #F=" << nbFaces()
          << " #V=" << nbVertices();
      if ( hasFaceGeometry() ) out << " hasFaceGeometry";
      out << " ]";
    }

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid (at least one cell), 'false' otherwise.
     */
    bool isValid() const
    {
      return nbCells() != 0;
    }

    /// @}
    // ----------------------- protected services ----------------------
  protected:
    /// @name Protected services
    /// @{

    /// computes the vertices of a given cell \a c upon request.
    /// @param c any valid cell
    void computeCellVertices( const Cell c ) const
    {
      std::set< Vertex > vertices;
      for ( auto f : cell_faces[ c ] )
        vertices.insert( true_face_vertices[ f.first ].cbegin(),
                         true_face_vertices[ f.first ].cend() );
      cell_vertices[ c ] = VertexRange( vertices.cbegin(), vertices.cend() );
    }

    /// Computes for each face its outward oriented normal vector.
    void computeFaceGeometry()
    {
      true_face_normal   .resize( nbFaces() );
      true_face_intercept.resize( nbFaces() );
      // Compute normals and intercepts
      for ( Index f = 0; f < nbFaces(); ++f ) {
        std::tie( true_face_normal[ f ], true_face_intercept[ f ] )
          = computeHalfSpace( true_face_vertices[ f ] );
        if ( true_face_normal[ f ] == Vector::zero )
          trace.error() << "[ConvexCellComplex::computeFaceGeometry]"
                        << " null normal vector at face " << f << std::endl;
      }
      // Orient them consistently
      for ( Index c = 0; c < nbCells(); c++ ) {
        //const auto& c_vtcs = cellVertices( c ); //not used
        for ( auto f : cellFaces( c ) ) {
          if ( ! f.second ) continue; // process only 'true' faces
          const   auto ov = faceComplementVertices( f );
          ASSERT( ! ov.empty() );
          const Vector  N = true_face_normal   [ f.first ];
          const Scalar nu = true_face_intercept[ f.first ];
          const Scalar iv = N.dot( position( ov.front() ) );
          if ( ( iv - nu ) > 0 ) {
            // Should be inside
            true_face_normal   [ f.first ] = -N;
            true_face_intercept[ f.first ] = -nu;
            std::reverse( true_face_vertices[ f.first ].begin(),
                          true_face_vertices[ f.first ].end() );
          }
          reorderFaceVertices( f.first );
        }
      }
      has_face_geometry = true;
    }

    /// @param v a range of vertices of size() >= dimension
    /// @return the pair normal/intercept of the face.
    std::pair< Vector, Scalar >
    computeHalfSpace( const VertexRange& v ) const
    {
      typedef DGtal::SimpleMatrix< Scalar, dimension, dimension > Matrix;
      Matrix A;
      Vector N;
      Scalar c;
      for ( size_t i = 1; i < dimension; i++ )
        for ( size_t j = 0; j < dimension; j++ )
          A.setComponent( i-1, j, position( v[ i ] )[ j ] - position( v[ 0 ] )[ j ] );
      for ( size_t j = 0; j < dimension; j++ )
        N[ j ] = A.cofactor( dimension-1, j );
      c = N.dot( position( v[ 0 ] ) );
      return std::make_pair( N, c );
    }

    /// Given a face index \a f, reorder all its vertices so that they are oriented
    /// consistently with respect to the normal.
    ///
    /// @param f any valid face index
    ///
    /// @note the order of points is consistent if, picking any
    /// d-simplex in order in this range, their associated half-spaces
    /// have all the same orientation.
    void reorderFaceVertices( Index f )
    {
      VertexRange& result = true_face_vertices[ f ];
      Vector       N      = true_face_normal  [ f ];
      // Sort a facet such that its points, taken in order, have
      // always the same orientation of the facet.  More precisely,
      // facets span a `dimension-1` vector space. There are thus
      // dimension-2 fixed points, and the last ones (at least two)
      // may be reordered.
      VertexRange splx( dimension );
      for ( Dimension k = 0; k < dimension-2; k++ )
        splx[ k ] = result[ k ];
      std::sort( result.begin()+dimension-2, result.end(),
                 [&] ( Index i, Index j )
                 {
                   splx[ dimension-2 ] = i;
                   splx[ dimension-1 ] = j;
                   const auto H = computeHalfSpace( splx );
                   const auto orient = N.dot( H.first );
                   return orient > 0;
                 } );
    }
    
    /// @}

  };  // class ConvexCellComplex

  /**
   * Overloads 'operator<<' for displaying objects of class 'ConvexCellComplex'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ConvexCellComplex' to write.
   * @return the output stream after the writing.
   */
  template <typename TPoint>
  std::ostream&
  operator<< ( std::ostream & out, const ConvexCellComplex<TPoint> & object )
  {
    object.selfDisplay( out );
    return out;
  }
  
} // namespace DGtal

#endif // !defined ConvexCellComplex_h

#undef ConvexCellComplex_RECURSES
#endif // else defined(ConvexCellComplex_RECURSES)

