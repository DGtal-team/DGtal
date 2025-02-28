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
 * @file SurfaceMeshMeasure.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/23
 *
 * Header file for module SurfaceMeshMeasure.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SurfaceMeshMeasure_RECURSES)
#error Recursive header files inclusion detected in SurfaceMeshMeasure.h
#else // defined(SurfaceMeshMeasure_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SurfaceMeshMeasure_RECURSES

#if !defined SurfaceMeshMeasure_h
/** Prevents repeated inclusion of headers. */
#define SurfaceMeshMeasure_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <vector>
#include "DGtal/kernel/CCommutativeRing.h"
#include "DGtal/shapes/SurfaceMesh.h"

namespace DGtal
{
  /**
     Description of template class 'SurfaceMeshMeasure' <p> \brief
     Aim: stores an arbitrary measure on a SurfaceMesh object.  The
     measure can be spread onto its vertices, edges, or faces.  This
     class is notably used by CorrectedNormalCurrentComputer and
     NormalCycleComputer to store the curvature measures, which may be
     located on different cells. The measure can be scalar or any
     other summable type (see template parameter TValue).

     The common workflow to use this object is to access its public
     array data with a call to SurfaceMeshMeasure::kMeasures, then
     resize it and fill in its values. Of course, the indices used to
     number cells in the SurfaceMesh are the same as the indices used
     to store the measure associated to cells.

     \code
     ScalarMeasure mu0( &myMesh, 0.0 );
     auto& face_mu0 = mu0.kMeasures( 2 );
     face_mu0.resize( myMesh.nbFaces() );
     ...
     \endcode

     Afterwards, you can access the global measure of a set of cells
     specified by their indices through convenient methods like
     SurfaceMeshMeasure::vertexMeasure,
     SurfaceMeshMeasure::edgeMeasure, SurfaceMeshMeasure::faceMeasure,
     with potential weights.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
     @tparam TValue an arbitrary model of CCommutativeRing
   */
  template < typename TRealPoint, typename TRealVector, typename TValue >
  struct SurfaceMeshMeasure 
  {
    // ------------------------- Public Types ------------------------------
  public:
    
    typedef TRealPoint                     RealPoint;
    typedef TRealVector                    RealVector;
    typedef TValue                         Value;
    typedef SurfaceMeshMeasure< RealPoint, RealVector, Value > Self;
    typedef DGtal::SurfaceMesh< RealPoint, RealVector > SurfaceMesh;
    typedef typename SurfaceMesh::Index    Index;
    typedef typename SurfaceMesh::Size     Size;
    typedef typename SurfaceMesh::Vertex   Vertex;
    typedef typename SurfaceMesh::Edge     Edge;
    typedef typename SurfaceMesh::Face     Face;
    typedef std::vector< Value >           Values;
    typedef typename RealVector::Component Scalar;
    typedef std::pair< Face, Scalar >      WeightedFace;
    typedef std::pair< Edge, Scalar >      WeightedEdge;
    typedef std::pair< Vertex, Scalar >    WeightedVertex;
    /// The type that defines a range of vertices
    typedef std::vector< Vertex >          Vertices;
    /// The type that defines a range of edges
    typedef std::vector< Edge >            Edges;
    /// The type that defines a range of faces
    typedef std::vector< Face >            Faces;
    typedef std::vector< WeightedVertex >  WeightedVertices;
    typedef std::vector< WeightedEdge >    WeightedEdges;
    typedef std::vector< WeightedFace >    WeightedFaces;
    static const Dimension dimension = RealPoint::dimension;

    // ------------------------- Standard services ------------------------------
  public:
    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /// Constructor from mesh.
    /// @param aMesh any simplified mesh that is referenced in this object.
    /// @param zero_value the value 0 for the specified type TValue.
    SurfaceMeshMeasure( ConstAlias< SurfaceMesh > aMesh = nullptr,
                        Value zero_value = Value() )
      : myMeshPtr( &aMesh ), myZero( zero_value ) {}

    /// @return a pointer to the associated mesh or nullptr if the
    /// measure is not valid.
    const SurfaceMesh* meshPtr() const
    {
      return myMeshPtr;
    }

    /// @param dim the dimension of the cells where the measures are
    /// defined, among 0: vertex, 1: edges, 2: faces.
    /// @return a const reference to the `dim`-measures.
    const Values& kMeasures( Dimension dim ) const
    {
      return ( dim == 0 ) ? vertex_measures
        : ( (dim == 1 ) ? edge_measures : face_measures );
    }
    /// @param dim the dimension of the cells where the measures are
    /// defined, among 0: vertex, 1: edges, 2: faces.
    /// @return a reference to the `dim`-measures.
    Values& kMeasures( Dimension dim )
    {
      return ( dim == 0 ) ? vertex_measures
        : ( (dim == 1 ) ? edge_measures : face_measures );
    }
    
    /// @}
    
    // ------------------------- Measure services ------------------------------
  public:
    /// @name Measure services
    /// @{

    /// @return the total measure (i.e. onto the whole space).
    Value measure() const
    {
      Value m = myZero;
      for ( auto&& lm : vertex_measures ) m += lm;
      for ( auto&& lm : edge_measures   ) m += lm;
      for ( auto&& lm : face_measures   ) m += lm;
      return m;
    }
    /// Computes the total measure on the ball of center \a x and
    /// radius \a r. The center \a x must lie on or close to the face \a
    /// f (as a hint to compute cells in the given ball).
    ///
    /// @param x the position where the ball is centered.
    /// @param r the radius of the ball.
    /// @param f the face where center point \a x lies.
    Value measure( const RealPoint& x, Scalar r, Face f  ) const
    {
      if ( vertex_measures.empty() && edge_measures.empty() )
        {
          WeightedFaces
            faces = myMeshPtr->computeFacesInclusionsInBall( r, f, x );
          return faceMeasure( faces );
        }
      else
        {
          std::tuple< Vertices, WeightedEdges, WeightedFaces >
            wcells = myMeshPtr->computeCellsInclusionsInBall( r, f, x );
          Value m = vertexMeasure( std::get< 0 >( wcells ) );
          m      += edgeMeasure  ( std::get< 1 >( wcells ) );
          m      += faceMeasure  ( std::get< 2 >( wcells ) );
          return m;
        }
    }
      
    /// @param v any vertex index.
    /// @return its measure.
    Value vertexMeasure( Vertex v ) const
    {
      return v < vertex_measures.size() ? vertex_measures[ v ] : Value();
    }
    
    /// @param vertices any range of (valid) vertex indices.
    /// @return its measure.
    Value vertexMeasure( const Vertices& vertices ) const
    {
      Value m = myZero;
      if ( vertex_measures.empty() ) return m;
      for ( auto&& v : vertices )  m += vertex_measures[ v ];
      return m;
    }

    /// @param wvertices any range of weighted (valid) vertex indices.
    /// @return its measure.
    Value vertexMeasure( const WeightedVertices& wvertices ) const
    {
      Value m = myZero;
      if ( vertex_measures.empty() ) return m;
      for ( auto&& v : wvertices )  m += vertex_measures[ v.first ] * v.second;
      return m;
    }
    
    /// @param e any edge index.
    /// @return its measure.
    Value edgeMeasure( Edge e ) const
    {
      return e < edge_measures.size() ? edge_measures[ e ] : Value();
    }

    /// @param edges any range of (valid) edge indices.
    /// @return its measure.
    Value edgeMeasure( const Edges& edges ) const
    {
      Value m = myZero;
      if ( edge_measures.empty() ) return m;
      for ( auto&& e : edges )  m += edge_measures[ e ];
      return m;
    }

    /// @param wedges any range of weighted (valid) edge indices.
    /// @return its measure.
    Value edgeMeasure( const WeightedEdges& wedges ) const
    {
      Value m = myZero;
      if ( edge_measures.empty() ) return m;
      for ( auto&& e : wedges )  m += edge_measures[ e.first ] * e.second;
      return m;
    }
    
    /// @param f any face index.
    /// @return its measure.
    Value faceMeasure( Face f ) const
    {
      return f < face_measures.size() ? face_measures[ f ] : Value();
    }

    /// @param faces any range of (valid) face indices.
    /// @return its measure.
    Value faceMeasure( const Faces& faces ) const
    {
      Value m = myZero;
      if ( face_measures.empty() ) return m;
      for ( auto&& v : faces )  m += face_measures[ v ];
      return m;
    }

    /// @param wfaces any range of weighted (valid) face indices.
    /// @return its measure.
    Value faceMeasure( const WeightedFaces& wfaces ) const
    {
      Value m = myZero;
      if ( face_measures.empty() ) return m;
      for ( auto&& v : wfaces )  m += face_measures[ v.first ] * v.second;
      return m;
    }
    
    /// @}
    
    // ------------------------- Public Datas ------------------------------
  public:
    /// Stores the scalar curvature measure per indexed vertex element.
    Values vertex_measures;
    /// Stores the scalar curvature measure per indexed edge element.
    Values edge_measures;
    /// Stores the scalar curvature measure per indexed face element.
    Values face_measures;

    // ------------------------- Protected Datas ------------------------------
  protected:
    /// A pointer to the mesh over which computations are done.
    const SurfaceMesh* myMeshPtr;
    /// Zero value for the given type.
    Value myZero;
  };

  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SurfaceMeshMeasure_h

#undef SurfaceMeshMeasure_RECURSES
#endif // else defined(SurfaceMeshMeasure_RECURSES)
