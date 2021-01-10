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
 * @file ConvexityHelper.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/12/20
 *
 * Header file for module ConvexityHelper.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConvexityHelper_RECURSES)
#error Recursive header files inclusion detected in ConvexityHelper.h
#else // defined(ConvexityHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConvexityHelper_RECURSES

#if !defined ConvexityHelper_h
/** Prevents repeated inclusion of headers. */
#define ConvexityHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
#include "DGtal/shapes/PolygonalSurface.h"
#include "QuickHull.h"
#include "ConvexCellComplex.h"

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexityHelper
  /**
     Description of template class 'ConvexityHelper' <p> \brief Aim:
     Provides a set of functions to facilitate the computation of
     convex hulls and polytopes, as well as shortcuts to build cell
     complex representing a Delaunay complex.

     @tparam dim the dimension of the space where points and further objects live.
  */
  template < int dim, typename TInteger = DGtal::int32_t >
  struct ConvexityHelper {
    BOOST_STATIC_ASSERT( dim > 1 );
    static const Dimension dimension = dim;

    typedef TInteger                        Integer;
    typedef SpaceND< dim, Integer >         Space;
    typedef typename Space::Point           Point;
    typedef typename Space::Vector          Vector;
    typedef typename Space::RealPoint       RealPoint;
    typedef std::size_t                     Size;
    typedef std::size_t                     Index;
    typedef std::vector< Index >            IndexRange;
    typedef ConvexHullIntegralKernel< dim > LatticeConvexHullKernel;
    typedef ConvexHullRationalKernel< dim > RealConvexHullKernel;
    typedef DelaunayIntegralKernel< dim >   LatticeDelaunayKernel;
    typedef DelaunayRationalKernel< dim >   RealDelaunayKernel;
    typedef BoundedLatticePolytope< Space > LatticePolytope;

    // ----------------- lattice convex hull services -------------------------
  public:
    /// @name Lattice convex hull services
    /// @{
    
    /// Computes and returns a halfspace representation of the tightiest lattice
    /// polytope enclosing all the given input lattice points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return the tightiest bounded lattice polytope
    /// (i.e. H-representation) including the given range of points,
    /// or an empty polytope if the given range of points was not full
    /// dimensional.
    static
    LatticePolytope
    computeLatticePolytope( const std::vector< Point >& input_points,
                            bool remove_duplicates = true )
    {
      typedef typename LatticePolytope::Domain     Domain;
      typedef typename LatticePolytope::HalfSpace  PolytopeHalfSpace;
      typedef QuickHull< LatticeConvexHullKernel > ConvexHull;
      typedef typename ConvexHull::HalfSpace       ConvexHullHalfSpace;
      if ( input_points.empty() ) return LatticePolytope();
      // Compute domain
      Point l = input_points[ 0 ];
      Point u = input_points[ 0 ];
      for ( const auto& p : input_points ) {
        l = l.inf( p );
        u = u.sup( p );
      }
      Domain domain( l, u );
      // Compute convex hull
      ConvexHull hull;
      hull.setInput( input_points, remove_duplicates );
      bool ok = hull.computeConvexHull( ConvexHull::Status::FacetsCompleted );
      if ( ! ok ) return LatticePolytope();
      // Initialize polytope
      std::vector< ConvexHullHalfSpace > HS;
      std::vector< PolytopeHalfSpace >   PHS;
      hull.getFacetHalfSpaces( HS );
      PHS.reserve( HS.size() );
      for ( auto& H : HS ) {
        Vector  N;
        Integer nu;
        for ( Dimension i = 0; i < dim; ++i )
          N[ i ] = (Integer) H.internalNormal()[ i ];
        nu = (Integer) H.internalIntercept();
        PHS.emplace_back( N, nu );
      }
      return LatticePolytope( domain, PHS.cbegin(), PHS.cend(),
                              dimension == 2, true );
    }
    
    /// Computes and returns a halfspace representation of the
    /// tightiest lattice polytope enclosing all the given input
    /// lattice points. Other constraints are added so that we can
    /// perform axis aligned Minkowski sums on this polytope.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return the tightiest bounded lattice polytope
    /// (i.e. H-representation) including the given range of points,
    /// or an empty polytope if the given range of points was not full
    /// dimensional.
    static
    LatticePolytope
    computeMinkowskiSummableLatticePolytope
    ( const std::vector< Point >& input_points,
      bool remove_duplicates = true )
    {
      if ( dimension == 2 )
        return computeLatticePolytope( input_points, true );
      else if ( dimension >= 4 )
        trace.warning() << "[ConvexityHelper::computeMinkowskiSummableLatticePolytope]"
                        << " Not implemented starting from dimension 4."
                        << std::endl;
      typedef typename LatticePolytope::Domain     Domain;
      typedef typename LatticePolytope::HalfSpace  PolytopeHalfSpace;
      typedef QuickHull< LatticeConvexHullKernel > ConvexHull;
      typedef typename ConvexHull::HalfSpace       ConvexHullHalfSpace;
      typedef typename ConvexHull::Ridge           Ridge;
      if ( input_points.empty() ) return LatticePolytope();
      // Compute domain
      Point l = input_points[ 0 ];
      Point u = input_points[ 0 ];
      for ( const auto& p : input_points ) {
        l = l.inf( p );
        u = u.sup( p );
      }
      Domain domain( l, u );
      // Compute convex hull
      ConvexHull hull;
      hull.setInput( input_points, remove_duplicates );
      bool ok = hull.computeConvexHull( ConvexHull::Status::VerticesCompleted );
      if ( ! ok ) return LatticePolytope();
      // Initialize polytope
      std::vector< ConvexHullHalfSpace > HS;
      std::vector< PolytopeHalfSpace >   PHS;
      hull.getFacetHalfSpaces( HS );
      PHS.reserve( HS.size() );
      for ( auto& H : HS ) {
        Vector  N;
        Integer nu;
        for ( Dimension i = 0; i < dim; ++i )
          N[ i ] = (Integer) H.internalNormal()[ i ];
        nu = (Integer) H.internalIntercept();
        PHS.emplace_back( N, nu );
      }

      if ( dimension == 3 )
        { // Compute ridge vertices to add edge constraints.
          std::vector< Point > positions;
          std::vector< IndexRange > facet_vertices; 
          std::vector< IndexRange > ridge_vertices; 
          std::map< Ridge, Index > ridge2index;
          hull.getVertexPositions( positions );
          computeFacetAndRidgeVertices( hull, facet_vertices,
                                        ridge2index, ridge_vertices );
          for ( auto p : ridge2index ) {
            const auto   r = p.first;
            const auto&  U = PHS[ r.first  ].N; // normal of facet 1
            const auto&  V = PHS[ r.second ].N; // normal of facet 2
            const auto&  S = ridge_vertices[ p.second ]; // vertices along facets 1, 2
            ASSERT( S.size() == 2 && "Invalid ridge" );
            const auto& P0 = positions[ S[ 0 ] ];
            const auto& P1 = positions[ S[ 1 ] ];
            auto         E = P1 - P0; // edge 1, 2
            const auto UxV =
              detail::BoundedLatticePolytopeSpecializer< dimension, Integer>
              ::crossProduct( U, V ); // parallel to E
            ASSERT( E.dot( UxV ) != 0 && "Invalid E / UxV" );
            if ( E.dot( UxV ) <= 0 ) E = -E; // force correct orientation
            const auto  E1 = 
              detail::BoundedLatticePolytopeSpecializer< dimension, Integer>
              ::crossProduct( U, E ); // edge on facet 1
            const auto  E2 = 
              detail::BoundedLatticePolytopeSpecializer< dimension, Integer>
              ::crossProduct( E, V ); // edge on facet 2
            ASSERT( E1.dot( U ) == 0 && "Invalid E1 / U" );
            ASSERT( E1.dot( V ) <  0 && "Invalid E1 / V" );
            ASSERT( E2.dot( V ) == 0 && "Invalid E1 / V" );
            ASSERT( E2.dot( U ) <  0 && "Invalid E1 / U" );
            for ( Dimension k = 0; k < dimension; ++k ) {
              const auto W = U[ k ] * V - V[ k ] * U;
              const auto nn1 = W.dot( E1 ); 
              const auto nn2 = W.dot( E2 ); 
              if ( nn1 > 0 && nn2 > 0 ) {
                PHS.emplace_back( -W, -W.dot( P0 ) );
                ASSERT( E1.dot(-W ) < 0 && "Invalid E1 /-W" );
                ASSERT( E2.dot(-W ) < 0 && "Invalid E2 /-W" );
              }
              else if ( nn1 < 0 && nn2 < 0 ) {
                PHS.emplace_back( W, W.dot( P0 ) );
                ASSERT( E1.dot( W ) < 0 && "Invalid E1 / W" );
                ASSERT( E2.dot( W ) < 0 && "Invalid E2 / W" );
              }
            }
          }
        }
      return LatticePolytope( domain, PHS.cbegin(), PHS.cend(),
                              dimension <= 3, true );
    }

    
    /// Computes a surface mesh representation of the boundary of the
    /// convex hull of the given lattice points.
    ///
    /// @tparam TSurfaceMesh any model of surface that can be
    /// initialized with a range of input positions (cast as real
    /// coordinates) and a range of index ranges giving for each face
    /// its range of incident vertices. For instance, you may use
    /// class SurfaceMesh.
    ///
    /// @param[out] mesh the output surface mesh that represents the
    /// boundary of the convex hull of the given range of points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output mesh is correct, otherwise return 'false'.
    template < typename TSurfaceMesh >
    static
    bool
    computeConvexHullBoundary( TSurfaceMesh&               mesh,
                               const std::vector< Point >& input_points,
                               bool remove_duplicates = true )
    {
      typedef TSurfaceMesh                         SurfaceMesh;
      typedef QuickHull< LatticeConvexHullKernel > ConvexHull;
      typedef typename ConvexHull::IndexRange      IndexRange;
      ConvexHull hull;
      hull.setInput( input_points, remove_duplicates );
      bool ok = hull.computeConvexHull( ConvexHull::Status::VerticesCompleted );
      if ( !ok ) return false;
      std::vector< RealPoint > positions;
      hull.getVertexPositions( positions );
      std::vector< IndexRange > faces;
      hull.getFacetVertices( faces );
      mesh = SurfaceMesh( positions.cbegin(), positions.cend(),
                          faces.cbegin(), faces.cend() );
      return true;
    }

    /// Computes a polygonal surface representation of the boundary of the
    /// convex hull of the given lattice points.
    ///
    /// @param[out] polysurf the output polygonal surface that
    /// represents the boundary of the convex hull of the given range
    /// of points. Its euler characteristic should be 0 in even
    /// dimension, 2 in odd dimension.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output surface is correct, otherwise return 'false'.
    static
    bool
    computeConvexHullBoundary( PolygonalSurface< Point >&  polysurf,
                               const std::vector< Point >& input_points,
                               bool remove_duplicates = true )
    {
      typedef QuickHull< LatticeConvexHullKernel > ConvexHull;
      typedef typename ConvexHull::IndexRange      IndexRange;
      ConvexHull hull;
      hull.setInput( input_points, remove_duplicates );
      bool ok = hull.computeConvexHull( ConvexHull::Status::VerticesCompleted );
      if ( !ok ) return false;
      std::vector< Point > positions;
      hull.getVertexPositions( positions );
      std::vector< IndexRange > faces;
      hull.getFacetVertices( faces );
      // build polygonal surface
      polysurf.clear();
      for ( auto p : positions ) polysurf.addVertex( p );
      for ( auto f : faces )     polysurf.addPolygonalFace( f );
      return polysurf.build();
    }

    /// Computes a cell complex representing the convex hull of the
    /// given lattice points, formed of one maximal dimension cell and
    /// as many cells of codimension 1 as the number of facets of the
    /// convex hull.
    ///
    /// @param[out] cell_complex the output cell complex that
    /// represents the convex hull of the given lattice points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output complex is correct, otherwise return 'false'.
    static
    bool
    computeConvexHullCellComplex( ConvexCellComplex< Point >& cell_complex,
                                  const std::vector< Point >& input_points,
                                  bool remove_duplicates = true )
    {
      typedef QuickHull< LatticeConvexHullKernel > ConvexHull;
      typedef typename ConvexHull::IndexRange      IndexRange;
      typedef typename ConvexCellComplex< Point >::FaceRange FaceRange;
      ConvexHull hull;
      hull.setInput( input_points, remove_duplicates );
      bool ok = hull.computeConvexHull( ConvexHull::Status::VerticesCompleted );
      cell_complex.clear();
      if ( ! ok ) return false;
      // Build complex, only 1 finite cell and as many faces as convex hull facets.
      // Taking care of faces for each cell (here one cell borders all faces).
      std::vector< IndexRange > faces;
      hull.getFacetVertices( faces );
      FaceRange all_faces;
      for ( Index i = 0; i < faces.size(); i++ )
        all_faces.push_back( { i, true } );
      cell_complex.cell_faces.push_back( all_faces );
      // Vertices of this unique cell will be computed lazily on request.
      // Taking care of each face.
      for ( const auto& vtcs : faces )
        {
          // every inner face borders cell 0
          cell_complex.true_face_cell.push_back( 0 );
          // every outer face borders the infinite cell
          cell_complex.false_face_cell.push_back( cell_complex.infiniteCell() );
        }
      // Taking care of vertices (in consistent order) of every face
      cell_complex.true_face_vertices.swap( faces );
      // Taking care of vertex positions
      hull.getVertexPositions( cell_complex.vertex_position );
      return true;
    }

    /// @}
    
    // ----------------- lattice Delaunay services -------------------------
  public:
    /// @name Lattice Delaunay services
    /// @{

    /// Computes the Delaunay cell complex associated to the given
    /// range of input points.
    ///
    /// @param[out] cell_complex the output cell complex that
    /// represents the Delaunay complex of the given lattice points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output complex is correct, otherwise return 'false'.
    ///
    /// @note The Delaunay cell complex may not be simplicial if some
    /// points are cospherical.
    static
    bool
    computeDelaunayCellComplex( ConvexCellComplex< Point >& cell_complex,
                                const std::vector< Point >& input_points,
                                bool remove_duplicates = true )
    {
      typedef QuickHull< LatticeDelaunayKernel >   Delaunay;
      typedef typename Delaunay::Ridge             Ridge;
      typedef typename Delaunay::IndexRange        IndexRange;
      typedef typename ConvexCellComplex< Point >::FaceRange FaceRange;

      std::cout << "Compute convex hull in higher dimension" << std::endl;
      Delaunay del;
      del.setInput( input_points, remove_duplicates );
      bool ok = del.computeConvexHull( Delaunay::Status::VerticesCompleted );
      cell_complex.clear();
      if ( ! ok ) return false;
      
      // Build complex, as many maximal cells as convex hull facets.
      // convex hull facet -> cell of complex
      // convex hull ridge -> face of complex
      // (1) Get cell vertices, count ridges/faces and compute their vertices
      std::map< Ridge, Index > r2f;
      computeFacetAndRidgeVertices( del,
                                    cell_complex.cell_vertices,
                                    r2f,
                                    cell_complex.true_face_vertices );
      // (2) assign ridges/faces to cell and conversely
      const Index  nb_r = r2f.size();
      std::cout << "assign ridges/faces to cell and conversely" << std::endl;
      cell_complex.true_face_cell .resize( nb_r, cell_complex.infiniteCell() );
      cell_complex.false_face_cell.resize( nb_r, cell_complex.infiniteCell() );
      cell_complex.true_face_vertices.resize( nb_r );
      for ( Index cur_f = 0; cur_f < del.nbFiniteFacets(); ++cur_f ) { 
        const auto& facet = del.facets[ cur_f ];
        FaceRange current_faces;
        for ( auto neigh_f : facet.neighbors ) {
          const Ridge r { std::min( cur_f, neigh_f ), std::max( cur_f, neigh_f ) };
          const bool pos = cur_f < neigh_f;
          const Index cur_r = r2f[ r ];
          cell_complex.true_face_cell [ cur_r ] = r.first;
          if ( r.second >= del.nbFiniteFacets() )
            cell_complex.false_face_cell[ cur_r ] = cell_complex.infiniteCell();
          else
            cell_complex.false_face_cell[ cur_r ] = r.second;
          current_faces.emplace_back( cur_r, pos );
        }
        cell_complex.cell_faces.push_back( current_faces );
      }
      // (3) Takes care of vertex positions
      std::cout << "takes care of vertex positions" << std::endl;
      del.getVertexPositions( cell_complex.vertex_position );
      return true;
    }
    
    /// @}
    
    // ----------------- utility services -------------------------
  public:
    /// @name Utility services
    /// @{

    template < typename QHull >
    static
    void
    computeFacetAndRidgeVertices( const QHull& hull,
                                  std::vector< IndexRange >& cell_vertices,
                                  std::map< typename QHull::Ridge, Index >& r2f,
                                  std::vector< IndexRange >& face_vertices )
    {
      typedef typename QHull::Ridge             Ridge;
      typedef typename QHull::IndexRange        IndexRange;

      // Get cell vertices and sort them
      hull.getFacetVertices( cell_vertices );
      std::vector< IndexRange > sorted_cell_vertices = cell_vertices;
      for ( auto& vtcs : sorted_cell_vertices )
        std::sort( vtcs.begin(), vtcs.end() );
      cell_vertices.resize( hull.nbFiniteFacets() );
      
      // Count ridges/faces and compute their vertices
      Index  nb_r = 0;
      face_vertices.clear();
      for ( Index cur_f = 0; cur_f < hull.nbFiniteFacets(); ++cur_f )  { 
        const auto& facet = hull.facets[ cur_f ];
        for ( auto neigh_f : facet.neighbors ) {
          const Ridge r { std::min( cur_f, neigh_f ), std::max( cur_f, neigh_f ) };
          auto itr = r2f.find( r );
          if ( itr == r2f.end() ) {
            IndexRange result;
            std::set_intersection( sorted_cell_vertices[ cur_f ].cbegin(),
                                   sorted_cell_vertices[ cur_f ].cend(),
                                   sorted_cell_vertices[ neigh_f ].cbegin(),
                                   sorted_cell_vertices[ neigh_f ].cend(), 
                                   std::back_inserter( result ) );
            face_vertices.push_back( result );
            r2f[ r ] = nb_r++;
          }
        }
      }
    }

    /// @}
    
  }; // class ConvexityHelper

  
} // namespace DGtal

#endif // !defined ConvexityHelper_h

#undef ConvexityHelper_RECURSES
#endif // else defined(ConvexityHelper_RECURSES)
